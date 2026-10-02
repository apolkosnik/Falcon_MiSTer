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
//     --pctrace N       print the PC every N clocks
//     --berrtrace       print every bus error cycle
//     --iotrace A:B     print device bus accesses with address in [A,B] (hex)
//     --key T:CODE      at time T ms press PS/2 set 2 CODE (hex, E0xx = extended) for 50 ms
//     --mouse T:DX:DY:B at time T ms send a mouse packet
#include "Vtb_top.h"
#include "verilated.h"
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
    uint8_t buf[512];
};

struct Event { double t; int kind; int a, b, c; };

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vtb_top *top = new Vtb_top;

    double run_ms = 3000;
    std::string frames_dir;
    int frame_every = 1, monitor = 2, ram = 14;
    long pctrace = 0;
    bool berrtrace = false;
    unsigned io_lo = 1, io_hi = 0;
    double io_from = 0;
    Disk disk[7];
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
        if (a == "--ms") run_ms = atof(next().c_str());
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
        else if (a == "--pctrace") pctrace = atol(next().c_str());
        else if (a == "--berrtrace") berrtrace = true;
        else if (a == "--iotrace") sscanf(next().c_str(), "%x:%x", &io_lo, &io_hi);
        else if (a == "--iofrom") io_from = atof(next().c_str());
        else if (a == "--key") { double t; unsigned c; sscanf(next().c_str(), "%lf:%x", &t, &c); events.push_back({t, 0, (int)c, 0, 0}); }
        else if (a == "--mouse") { double t; int dx, dy, b; sscanf(next().c_str(), "%lf:%d:%d:%d", &t, &dx, &dy, &b); events.push_back({t, 1, dx, dy, b}); }
    }
    if (!frames_dir.empty()) mkdir(frames_dir.c_str(), 0755);

    static const int mon_map[4] = {0, 1, 2, 3};
    top->monitor = mon_map[monitor & 3];
    top->ram_mb = ram;
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
                d.phase = 1; d.delay = 200 + (rand() & 255); d.idx = 0;
                if (rd) {
                    memset(d.buf, 0, 512);
                    if (d.f && (uint64_t)d.lba * 512 < d.size) { fseek(d.f, (long)d.lba * 512, SEEK_SET); if (fread(d.buf, 1, 512, d.f) != 512) {} }
                }
                d.idx = rd ? 0 : -1000;   // negative: write
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
                    if (++d.idx == 512) d.phase = 4;
                }
                break;
            }
            if (d.phase == 3) {           // core -> HPS: address now, data next clock
                if (d.idx > 0) d.buf[d.idx - 1] = (uint8_t)(top->sd_buff_din_f >> (8 * n));
                if (d.idx == 512) {
                    if (d.f && !d.ro) { fseek(d.f, (long)d.lba * 512, SEEK_SET); fwrite(d.buf, 1, 512, d.f); fflush(d.f); }
                    d.phase = 4;
                } else { top->sd_buff_addr = d.idx; d.idx++; }
                break;
            }
            if (d.phase == 4) { top->sd_ack &= ~(1 << n); d.phase = 5; break; }
            if (d.phase == 5) { if (!(((top->sd_rd | top->sd_wr) >> n) & 1)) d.phase = 0; }
        }

        tick();

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
    }
    printf("simulated %.3f ms, %llu clocks\n", cyc / CLK_HZ * 1e3, (unsigned long long)cyc);
    top->final();
    delete top;
    return 0;
}
