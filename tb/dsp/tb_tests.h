// Falcon DSP testbench - test list. Plain ASCII.
#ifndef TB_TESTS_H
#define TB_TESTS_H

#include "tb_lockstep.h"

struct TestConfig {
    int seeds = 600;
    int insns_per_seed = 3000;
    int seed0 = 1;
    bool verbose = true;
    std::string only;
};

void report(const std::string &name, bool pass, const std::string &info);
void run_all_tests(Rtl &rtl, const TestConfig &cfg);

#endif
