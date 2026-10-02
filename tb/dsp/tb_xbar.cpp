// Falcon DSP + real falcon_crossbar integration test (SSI slot link).
// The crossbar routes the DSP transmitter to the DSP receiver ($FF8932
// bits 6:5 = 01, $FF8930 bit 7 = 1, 25 MHz clock, divider 1, one track).
// The DSP program receives each word and transmits it + 1, so the words
// seen on the link must count up by one per slot.  Plain ASCII.
#include <cstdio>
#include <cstdint>
#include <vector>
#include "Vtb_xbar_top.h"
#include "Vtb_xbar_top___024root.h"
#include "verilated.h"

double sc_time_stamp() { return 0; }

static Vtb_xbar_top *top;
static void clk1() { top->clk = 1; top->eval(); top->clk = 0; top->eval(); }

static void xb_write(int word_addr, bool uds, bool lds, uint16_t d) {
    top->xb_cs = 1; top->xb_stb = 1; top->xb_we = 1; top->xb_addr = word_addr;
    top->xb_uds = uds; top->xb_lds = lds; top->xb_din = d;
    top->eval();
    for (int i = 0; i < 8; i++) {
        bool ack = top->xb_ack;
        clk1();
        top->xb_stb = 0;
        top->eval();
        if (ack || top->xb_ack) { if (top->xb_ack) clk1(); break; }
    }
    top->xb_cs = 0; top->xb_we = 0; top->xb_uds = 0; top->xb_lds = 0;
    top->eval();
}

static void hp_write(int off, uint8_t v) {
    top->hp_cs = 1; top->hp_stb = 1; top->hp_we = 1; top->hp_addr = off >> 1;
    top->hp_uds = !(off & 1); top->hp_lds = off & 1;
    top->hp_din = (off & 1) ? v : (uint16_t)(v << 8);
    top->eval();
    clk1();
    top->hp_cs = 0; top->hp_stb = 0; top->hp_we = 0; top->hp_uds = 0; top->hp_lds = 0;
    top->eval();
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vtb_xbar_top;
    top->reset = 1;
    for (int i = 0; i < 8; i++) clk1();
    top->reset = 0;
    top->eval();

    // DSP program (P internal RAM, backdoor): see the file header
    static const uint32_t prog[] = {
        0x08f4ac, 0x004100,   // $40 movep #$4100,x:$ffec   CRA: 16 bit, 2 words/frame
        0x08f4ad, 0x003800,   // $42 movep #$3800,x:$ffed   CRB: RE TE, network mode
        0x44f400, 0x000100,   // $44 move #$000100,x0
        0x0aae87, 0x000046,   // $46 jclr #7,x:$ffee,$46    wait RDF
        0x084e2f,             // $48 movep x:$ffef,a
        0x200040,             // $49 add x0,a
        0x0aae86, 0x00004a,   // $4a jclr #6,x:$ffee,$4a    wait TDE
        0x08ce2f,             // $4c movep a,x:$ffef
        0x0c0046,             // $4d jmp $46
    };
    auto &pint = top->rootp->tb_xbar_top__DOT__u_dsp__DOT__u_core__DOT__u_pint__DOT__mem;
    pint[0] = 0x0c0040;       // jmp $40
    for (unsigned i = 0; i < sizeof(prog) / sizeof(prog[0]); i++) pint[0x40 + i] = prog[i];
    top->eval();
    hp_write(0, 0x08);        // ICR.HF0: start the DSP (skips the bootstrap)
    hp_write(0, 0x00);

    // crossbar: 1 track, DSP xmit connected on the 25 MHz clock, DSP
    // receive fed from the DSP transmitter, divider 1 (49170 Hz)
    xb_write(0x10, true, false, 0x0000);     // $FF8920: 1 track
    xb_write(0x18, true, true, 0x0080);      // $FF8930: DSP xmit on, 25 MHz
    xb_write(0x19, true, true, 0x00a0);      // $FF8932: DSP rcv on, source DSP xmit
    xb_write(0x1a, false, true, 0x0001);     // $FF8935: divider 1

    std::vector<uint16_t> tx;
    int loop_err = 0, slots = 0, gap_min = 1 << 30, last = -1;
    for (long c = 0; c < 400000 && tx.size() < 400; c++) {
        top->eval();
        if (top->mon_stb) {
            slots++;
            if (last >= 0 && c - last < gap_min) gap_min = (int)(c - last);
            last = (int)c;
            if (top->mon_tx_en) tx.push_back(top->mon_tx_data);
            if (top->mon_rx_en && top->mon_tx_en && top->mon_rx_data != top->mon_tx_data) loop_err++;
        }
        clk1();
    }
    int bad = 0, start = -1;
    for (size_t k = 1; k < tx.size(); k++) {
        if (start < 0) { if (tx[k] != 0 && tx[k] == (uint16_t)(tx[k - 1] + 1)) start = (int)k; continue; }
        if (tx[k] != (uint16_t)(tx[k - 1] + 1)) {
            if (bad < 5) printf("  CHECK FAILED slot %zu: transmitted %04x after %04x (expected +1)\n", k, tx[k], tx[k - 1]);
            bad++;
        }
    }
    bool pass = tx.size() >= 400 && start >= 0 && start < 10 && bad == 0 && loop_err == 0;
    printf("%s: DSP + falcon_crossbar SSI loopback (%zu transmit slots, counting from slot %d, "
           "%d count errors, %d loopback errors, min slot spacing %d clocks)\n",
           pass ? "PASS" : "FAIL", tx.size(), start, bad, loop_err, gap_min);
    top->final();
    delete top;
    return pass ? 0 : 1;
}
