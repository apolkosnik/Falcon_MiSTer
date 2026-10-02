// Falcon DSP testbench main: runs every test and prints a PASS/FAIL summary.
// Plain ASCII.
#include "tb_lockstep.h"
#include "tb_tests.h"

static double sc_time = 0;
double sc_time_stamp() { return sc_time; }

struct Result { std::string name; bool pass; std::string info; };
static std::vector<Result> results;

void report(const std::string &name, bool pass, const std::string &info) {
    results.push_back({name, pass, info});
    printf("%s: %s %s\n", pass ? "PASS" : "FAIL", name.c_str(), info.c_str());
    fflush(stdout);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    TestConfig cfg;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc) cfg.seeds = atoi(argv[++i]);
        else if (a == "--insns" && i + 1 < argc) cfg.insns_per_seed = atoi(argv[++i]);
        else if (a == "--seed0" && i + 1 < argc) cfg.seed0 = atoi(argv[++i]);
        else if (a == "--only" && i + 1 < argc) cfg.only = argv[++i];
        else if (a == "--quiet") cfg.verbose = false;
    }
    golden_init_once();
    Rtl rtl;

    run_all_tests(rtl, cfg);

    int fails = 0;
    printf("\n==================== SUMMARY ====================\n");
    for (auto &r : results) {
        printf("%-4s %-44s %s\n", r.pass ? "PASS" : "FAIL", r.name.c_str(), r.info.c_str());
        if (!r.pass) fails++;
    }
    printf("=================================================\n");
    printf("%s: %d test(s), %d failure(s)\n", fails ? "FAIL" : "PASS", (int)results.size(), fails);
    return fails ? 1 : 0;
}
