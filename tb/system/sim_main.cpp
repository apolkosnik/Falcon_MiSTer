// Full-system simulation driver for the Falcon core.
//
//   ./obj_dir/Vtb_top +rom=etos512us.hex [options]
//     --ms N            simulated milliseconds to run (default 3000)
//     --ide0 FILE       IDE master image      --ide1 FILE  IDE slave image
//     --fda FILE        floppy A image (.ST)  --fdb FILE   floppy B image
//     --scsi0/--scsi1 FILE  SCSI hard disks    --scsicd FILE SCSI ID 2 CD-ROM (.iso)
//     --frames DIR      write every captured frame as DIR/frame_NNNN.ppm
//     --frame-every N   only every Nth frame (default 1)
//     --monitor M       0 mono, 1 RGB, 2 VGA (default), 3 TV
//     --ram MB          4 or 14 (default 14)
//     --ramtos          the ROM image is a RAM TOS behind its loader (TOS 4.92)
//     --pctrace N       print the PC every N clocks
//     --berrtrace       print every bus error cycle
//     --iotrace A:B     print device bus accesses with address in [A,B] (hex)
//     --key T:CODE      at time T ms press PS/2 set 2 CODE (hex, E0xx = extended) for 50 ms
//     --mouse T:DX:DY:B at time T ms send a mouse packet
//     --fpu             play the HPS FPU service (tools/falcon_fpu): MAGIC,
//                       VERSION 4 and a heartbeat every 10 ms in the mailbox
//                       at $E90000, and every request executed by the same
//                       68882 engine (libfpe) the ARM service uses
//     --cptrace         print every coprocessor (FPU) interface register access
//     --cookies         print the TOS cookie jar at the end
//     --text ADDR       print the NUL-terminated text at guest ADDR (hex) at the end
//     --done ADDR       end the run once the long at guest ADDR (hex) reads $D0E0600D
#include "Vtb_top.h"
#include "Vtb_top__Dpi.h"
#include "verilated.h"
#include "svdpi.h"
#include "fpu_request.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <sys/stat.h>

static const double CLK_HZ = 32e6;

struct Disk {
    FILE *f = nullptr;
    uint64_t size = 0;
    bool ro = false;
    // block transfer state
    int phase = 0;       // 0 idle, 1 delay, 2 read stream, 3 write stream, 4 finish
    int delay = 0;
    int idx = 0;
    uint32_t lba = 0;
    uint8_t buf[16384];
    int nblk = 1;
};

struct Event { double t; int kind; int a, b, c; };

// guest memory through the DDR3 model (DDR3 byte k = guest byte k)
static uint8_t guest_rd8(uint32_t a)
{
    return (uint8_t)(ddr_peek(a >> 3) >> (8 * (a & 7)));
}
static uint32_t guest_rd32(uint32_t a)
{
    return (uint32_t)guest_rd8(a) << 24 | (uint32_t)guest_rd8(a + 1) << 16 | (uint32_t)guest_rd8(a + 2) << 8 | guest_rd8(a + 3);
}
static void guest_wr16(uint32_t a, uint16_t v)   // a even
{
    uint64_t d = ((uint64_t)(v >> 8) << (8 * (a & 7))) | ((uint64_t)(v & 0xFF) << (8 * ((a + 1) & 7)));
    ddr_poke(a >> 3, d, (uint8_t)(3u << (a & 7)));
}
static void guest_wr8(uint32_t a, uint8_t v)
{
    ddr_poke(a >> 3, (uint64_t)v << (8 * (a & 7)), (uint8_t)(1u << (a & 7)));
}
static uint16_t guest_rd16(uint32_t a) { return (uint16_t)(guest_rd8(a) << 8 | guest_rd8(a + 1)); }

// the HPS FPU service (tools/falcon_fpu/falcon_fpu.c) on the DDR3 model
static uint16_t fpu_seen;
static void fpu_serve(void)
{
    const uint32_t MB = 0xE90000;
    uint16_t seq = guest_rd16(MB + 0x100);
    if (seq == fpu_seen) return;
    fpu_seen = seq;
    uint8_t in[FPU_REQ_MAX], out[FPE_MAXIO];
    uint16_t kind = guest_rd16(MB + 0x102), cmd = guest_rd16(MB + 0x104), aux = guest_rd16(MB + 0x106);
    int n = fpu_kind_has_data(kind) ? guest_rd16(MB + 0x108) : 0, out_len = 0;
    uint32_t iaddr = (uint32_t)guest_rd16(MB + 0x10A) << 16 | guest_rd16(MB + 0x10C);
    if (n > FPU_REQ_MAX) n = FPU_REQ_MAX;
    for (int i = 0; i < n; i++) in[i] = guest_rd8(MB + 0x110 + i);
    uint16_t flags = fpu_request(kind, cmd, aux, iaddr, in, n, out, &out_len);
    for (int i = 0; i < out_len; i++) guest_wr8(MB + 0x210 + i, out[i]);
    uint32_t fpsr = fpe_fpsr();
    guest_wr16(MB + 0x202, flags);
    guest_wr16(MB + 0x200, fpu_status(seq, flags, fpsr));      // STATUS last
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vtb_top *top = new Vtb_top;

    double run_ms = 3000;
    std::string frames_dir;
    int frame_every = 1, monitor = 2, ram = 14, ramtos = 0;
    long pctrace = 0;
    bool berrtrace = false;
    unsigned io_lo = 1, io_hi = 0;
    double io_from = 0;
    Disk disk[7];
    bool fpu_service = false, cptrace = false, cookies = false;
    uint32_t text_addr = 0, done_addr = 0;
    double wrlat_ms = 0, rdlat_ms = 0;
    std::vector<Event> events;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
        auto open_disk = [&](int n) {
            std::string fn = next();
            disk[n].f = fopen(fn.c_str(), "r+b");
            if (!disk[n].f) { disk[n].f = fopen(fn.c_str(), "rb"); disk[n].ro = true; }
            if (!disk[n].f) { fprintf(stderr, "cannot open %s\n", fn.c_str()); exit(1); }
            fseek(disk[n].f, 0, SEEK_END); disk[n].size = ftell(disk[n].f);
        };
        if (a == "--wrlat") wrlat_ms = atof(next().c_str());
        else if (a == "--rdlat") rdlat_ms = atof(next().c_str());
        else if (a == "--ms") run_ms = atof(next().c_str());
        else if (a == "--fda") open_disk(0);
        else if (a == "--fdb") open_disk(1);
        else if (a == "--ide0") open_disk(2);
        else if (a == "--ide1") open_disk(3);
        else if (a == "--scsi0") open_disk(4);
        else if (a == "--scsi1") open_disk(5);
        else if (a == "--scsicd") open_disk(6);
        else if (a == "--frames") frames_dir = next();
        else if (a == "--frame-every") frame_every = atoi(next().c_str());
        else if (a == "--monitor") monitor = atoi(next().c_str());
        else if (a == "--ram") ram = atoi(next().c_str());
        else if (a == "--ramtos") ramtos = 1;
        else if (a == "--pctrace") pctrace = atol(next().c_str());
        else if (a == "--berrtrace") berrtrace = true;
        else if (a == "--iotrace") sscanf(next().c_str(), "%x:%x", &io_lo, &io_hi);
        else if (a == "--iofrom") io_from = atof(next().c_str());
        else if (a == "--key") { double t; unsigned c; sscanf(next().c_str(), "%lf:%x", &t, &c); events.push_back({t, 0, (int)c, 0, 0}); }
        else if (a == "--mouse") { double t; int dx, dy, b; sscanf(next().c_str(), "%lf:%d:%d:%d", &t, &dx, &dy, &b); events.push_back({t, 1, dx, dy, b}); }
        else if (a == "--fpu") fpu_service = true;
        else if (a == "--cptrace") cptrace = true;
        else if (a == "--cookies") cookies = true;
        else if (a == "--text") text_addr = (uint32_t)strtoul(next().c_str(), nullptr, 16);
        else if (a == "--done") done_addr = (uint32_t)strtoul(next().c_str(), nullptr, 16);
    }
    if (!frames_dir.empty()) mkdir(frames_dir.c_str(), 0755);
    svSetScope(svGetScopeFromName("TOP.tb_top.ddr"));
    uint16_t fpu_hb = 0;
    bool cp_pend = false, cp_we = false;
    unsigned cp_off = 0;
    uint32_t cp_wd = 0;

    static const int mon_map[4] = {0, 1, 2, 3};
    top->monitor = mon_map[monitor & 3];
    top->ram_mb = ram;
    top->ram_tos = ramtos;
    top->joy0 = 0;
    top->ps2_key = 0;
    top->ps2_mouse = 0;
    // RTC: 2026-10-01 12:00:00, Thursday (BCD), bit 64 toggles to latch
    top->rtc[0] = 0x12000000u | 0x00;          // [7:0] sec, [15:8] min, [23:16] hour, [31:24] day
    top->rtc[0] = 0x01120000u;
    top->rtc[1] = 0x00042610u;                 // [39:32] month, [47:40] year, [55:48] weekday
    top->rtc[2] = 0;
    top->img_readonly = 0;
    top->img_size = 0;
    top->img_mounted = 0;
    top->sd_ack = 0;
    top->sd_buff_wr = 0;
    top->sd_buff_addr = 0;
    top->sd_buff_dout = 0;

    uint64_t cyc = 0;
    const uint64_t total = (uint64_t)(run_ms * 1e-3 * CLK_HZ);
    int mount_step = 0;

    // video capture
    std::vector<uint8_t> fb;
    int fx = 0, fy = 0, maxx = 0, frame_no = 0;
    bool prev_hb = true, prev_vb = true;
    fb.resize(2048 * 1200 * 3);

    // keyboard/mouse
    bool key_toggle = false, mouse_toggle = false;
    uint32_t prev_pc = 0;
    bool prev_berr = false, rtc_toggle = false;

    auto tick = [&]() {
        top->clk = 0; top->eval();
        top->clk = 1; top->eval();
        cyc++;
    };

    while (cyc < total && !Verilated::gotFinish()) {
        double now_ms = cyc / CLK_HZ * 1e3;
        top->por = cyc < 300;
        top->cold_reset = cyc < 400;
        top->reset = cyc < 600;

        // the MiSTer announces the RTC once after start
        if (cyc == 1000) { rtc_toggle = !rtc_toggle; top->rtc[2] = rtc_toggle; }

        // mount the images once, one slot per clock pulse like hps_io
        if (cyc >= 2000 && mount_step < 7) {
            int n = mount_step;
            if (top->img_mounted) top->img_mounted = 0;
            else {
                if (disk[n].f) {
                    top->img_size = disk[n].size;
                    top->img_readonly = disk[n].ro;
                    top->img_mounted = 1 << n;
                }
                mount_step++;
            }
        } else if (mount_step >= 7 && top->img_mounted) top->img_mounted = 0;

        // scripted input
        for (auto &e : events) {
            if (e.kind < 0) continue;
            if (now_ms >= e.t) {
                if (e.kind == 0) {          // key press
                    key_toggle = !key_toggle;
                    top->ps2_key = (key_toggle << 10) | (1 << 9) | ((e.a & 0xE000) == 0xE000 ? 1 << 8 : 0) | (e.a & 0xFF);
                    e.kind = 2; e.t += 50;
                } else if (e.kind == 2) {   // key release
                    key_toggle = !key_toggle;
                    top->ps2_key = (key_toggle << 10) | ((e.a & 0xE000) == 0xE000 ? 1 << 8 : 0) | (e.a & 0xFF);
                    e.kind = -1;
                } else if (e.kind == 1) {
                    mouse_toggle = !mouse_toggle;
                    int st = (e.c & 7) | 8 | (e.a < 0 ? 0x10 : 0) | (e.b < 0 ? 0x20 : 0);
                    top->ps2_mouse = (mouse_toggle << 24) | ((e.b & 0xFF) << 16) | ((e.a & 0xFF) << 8) | st;
                    e.kind = -1;
                }
            }
        }

        // hps_io block protocol
        uint32_t lba[7];
        for (int n = 0; n < 7; n++) lba[n] = top->sd_lba_f[n];
        for (int n = 0; n < 7; n++) {
            Disk &d = disk[n];
            bool rd = (top->sd_rd >> n) & 1, wr = (top->sd_wr >> n) & 1;
            if (d.phase == 0 && (rd || wr)) {
                d.lba = lba[n];
                d.nblk = (int)((top->sd_blk_cnt_f >> (6 * n)) & 63) + 1;
                d.phase = 1; d.delay = 200 + (rand() & 255); d.idx = 0;
                if (rd) {
                    memset(d.buf, 0, sizeof d.buf);
                    if (d.f && (uint64_t)d.lba * 512 < d.size) { fseek(d.f, (long)d.lba * 512, SEEK_SET); if (fread(d.buf, 1, 512 * d.nblk, d.f) != 512u * d.nblk) {} }
                }
                d.idx = rd ? 0 : -1000;   // negative: write
                if (n >= 4) d.delay += (int)((rd ? rdlat_ms : wrlat_ms) * 1e-3 * CLK_HZ);
            }
        }
        top->sd_buff_wr = 0;
        for (int n = 0; n < 7; n++) {
            Disk &d = disk[n];
            if (d.phase == 1) {
                if (--d.delay <= 0) { d.phase = d.idx < 0 ? 3 : 2; d.idx = 0; top->sd_ack |= 1 << n; }
                break;
            }
            if (d.phase == 2) {           // HPS -> core
                if (cyc & 1) {
                    top->sd_buff_addr = d.idx; top->sd_buff_dout = d.buf[d.idx]; top->sd_buff_wr = 1;
                    if (++d.idx == 512 * d.nblk) d.phase = 4;
                }
                break;
            }
            if (d.phase == 3) {           // core -> HPS: address now, data next clock
                if (d.idx > 0) d.buf[d.idx - 1] = (uint8_t)(top->sd_buff_din_f >> (8 * n));
                if (d.idx == 512 * d.nblk) {
                    if (n >= 4 && getenv("HPSLOG")) printf("[%9.3f ms] HPS WR slot %d lba %u n %d\n", cyc/CLK_HZ*1e3, n, d.lba, d.nblk);
                    if (d.f && !d.ro) { fseek(d.f, (long)d.lba * 512, SEEK_SET); fwrite(d.buf, 1, 512 * d.nblk, d.f); fflush(d.f); }
                    d.phase = 4;
                } else { top->sd_buff_addr = d.idx; d.idx++; }
                break;
            }
            if (d.phase == 4) { top->sd_ack &= ~(1 << n); d.phase = 5; break; }
            if (d.phase == 5) { if (!(((top->sd_rd | top->sd_wr) >> n) & 1)) d.phase = 0; }
        }

        bool fpu_was = top->dbg_fpu_present;
        tick();
        // the service starts after the DDR3 model's initial block has cleared
        // the memory (first evaluation)
        if (fpu_service && cyc == 100) {
            fpe_reset();
            fpu_seen = guest_rd16(0xE90100);
            guest_wr16(0xE90200, fpu_seen);
            guest_wr16(0xE90004, 4);          // VERSION
            guest_wr16(0xE90000, 0x4650);     // MAGIC "FP"
        }
        if (fpu_service && cyc % 320000 == 0) guest_wr16(0xE90002, ++fpu_hb);   // 10 ms
        if (fpu_service && cyc > 100 && (cyc & 31) == 0) fpu_serve();       // ~1 us poll
        if (top->dbg_fpu_present != fpu_was)
            printf("[%10.3f ms] FPU %s\n", cyc / CLK_HZ * 1e3, top->dbg_fpu_present ? "present" : "absent");
        if (cptrace) {
            if (top->dbg_cp_req) { cp_pend = true; cp_we = top->dbg_cp_we; cp_off = top->dbg_cp_off; cp_wd = top->dbg_cp_wdata; }
            if (top->dbg_cp_ack && cp_pend) {
                cp_pend = false;
                if (top->dbg_cp_berr) printf("[%10.3f ms] CP %s CIR %02X  BERR  pc=%08x\n", cyc / CLK_HZ * 1e3, cp_we ? "W" : "R", cp_off, top->dbg_pc);
                else if (cp_we) printf("[%10.3f ms] CP W CIR %02X = %08x  pc=%08x\n", cyc / CLK_HZ * 1e3, cp_off, cp_wd, top->dbg_pc);
                else printf("[%10.3f ms] CP R CIR %02X -> %08x  pc=%08x\n", cyc / CLK_HZ * 1e3, cp_off, (uint32_t)top->dbg_cp_rdata, top->dbg_pc);
            }
        }

        // video
        if (top->ce_pix) {
            bool hb = top->hblank, vb = top->vblank;
            if (!hb && !vb) {
                if (fx < 2048 && fy < 1200) {
                    uint8_t *p = &fb[(fy * 2048 + fx) * 3];
                    p[0] = top->r; p[1] = top->g; p[2] = top->b;
                }
                fx++;
            }
            if (hb && !prev_hb) { if (fx > 0) { if (fx > maxx) maxx = fx; fy++; } fx = 0; }
            if (vb && !prev_vb) {
                if (fy > 0) {
                    if (!frames_dir.empty() && (frame_no % frame_every) == 0) {
                        char fn[512]; snprintf(fn, sizeof fn, "%s/frame_%04d.ppm", frames_dir.c_str(), frame_no);
                        FILE *o = fopen(fn, "wb");
                        fprintf(o, "P6\n%d %d\n255\n", maxx, fy);
                        for (int y = 0; y < fy; y++) fwrite(&fb[y * 2048 * 3], 1, maxx * 3, o);
                        fclose(o);
                    }
                    printf("[%9.3f ms] frame %d: %dx%d\n", now_ms, frame_no, maxx, fy);
                    frame_no++;
                }
                fx = 0; fy = 0; maxx = 0;
            }
            prev_hb = hb; prev_vb = vb;
        }

        if (pctrace && (cyc % pctrace) == 0 && top->dbg_pc != prev_pc) {
            printf("[%9.3f ms] pc=%08x ipl=%d\n", now_ms, top->dbg_pc, top->dbg_ipl);
            prev_pc = top->dbg_pc;
        }
        if (berrtrace && top->dbg_berr && !prev_berr)
            printf("[%9.3f ms] BERR a=%08x fc=%d %s pc=%08x\n", now_ms, top->dbg_a, top->dbg_fc, top->dbg_rw ? "rd" : "wr", top->dbg_pc);
        prev_berr = top->dbg_berr;

        // device bus trace: print on the acknowledge (read data valid)
        {
            static unsigned t_addr; static bool t_we, t_pend; static unsigned t_din; static int t_u, t_l;
            if (top->dbg_dev_stb) { t_addr = top->dbg_dev_addr; t_we = top->dbg_dev_we; t_din = top->dbg_dev_din; t_u = top->dbg_dev_uds; t_l = top->dbg_dev_lds; t_pend = true; }
            if (t_pend && top->dbg_dev_ack) {
                t_pend = false;
                if (t_addr >= io_lo && t_addr <= io_hi && now_ms >= io_from) {
                    if (t_we) printf("[%9.3f ms] IOW %06x %s = %04x pc=%08x\n", now_ms, t_addr, t_u && t_l ? "w" : t_u ? "e" : "o", t_din, top->dbg_pc);
                    else printf("[%9.3f ms] IOR %06x %s -> %04x pc=%08x\n", now_ms, t_addr, t_u && t_l ? "w" : t_u ? "e" : "o", top->dbg_dev_dout, top->dbg_pc);
                }
            }
        }

        if (top->dbg_halted) { printf("[%9.3f ms] CPU HALTED (double bus fault) pc=%08x\n", now_ms, top->dbg_pc); break; }
        if (done_addr && (cyc & 32767) == 0 && guest_rd32(done_addr) == 0xD0E0600Du) {
            printf("[%9.3f ms] done marker at %06x\n", now_ms, done_addr);
            break;
        }
    }
    printf("simulated %.3f ms, %llu clocks\n", cyc / CLK_HZ * 1e3, (unsigned long long)cyc);
    if (fpu_service)
        printf("FPU mailbox: MAGIC %04x HEARTBEAT %04x\n", guest_rd32(0xE90000) >> 16, guest_rd32(0xE90000) & 0xFFFF);
    if (text_addr) {
        printf("text at %06x:\n", text_addr);
        for (uint32_t i = 0; i < 65536; i++) {
            uint8_t c = guest_rd8(text_addr + i);
            if (!c) break;
            if (c != 13) putchar(c >= 32 || c == 10 ? c : '?');
        }
        printf("\n");
    }
    if (cookies) {
        uint32_t jar = guest_rd32(0x5A0) & 0xFFFFFF;
        printf("cookie jar at %06x\n", jar);
        for (int i = 0; jar && i < 64; i++) {
            uint32_t id = guest_rd32(jar + 8 * i), val = guest_rd32(jar + 8 * i + 4);
            if (!id) break;
            char n[5] = { (char)(id >> 24), (char)(id >> 16), (char)(id >> 8), (char)id, 0 };
            for (int k = 0; k < 4; k++) if (n[k] < 32 || n[k] > 126) n[k] = '?';
            printf("  %s = %08x\n", n, val);
        }
    }
    top->final();
    delete top;
    return 0;
}
