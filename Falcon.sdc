derive_pll_clocks
derive_clock_uncertainty

# core specific constraints
# The core runs on clk_sys (32 MHz, PLL output 0).  The ap060 build's
# processor runs on clk_cpu (50 MHz, PLL output 1): falcon_cpubus060 crosses
# between the two with toggles and synchronisers and holds the crossing
# fields steady (ap030_async_fifo for the snoops), so the clocks are
# asynchronous.  (The 030 build leaves output 1 unused.)
set_clock_groups -asynchronous -group [get_clocks {emu|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}]

# LTC2308: gated 8 MHz SCK from clk_sys, with 2 us CONVST and two
# clk_sys cycles (62.5 ns) before the first SDO capture / between SCK edges.
# Budget: 18 ns output Tco + 2 ns board + 15 ns ADC enable/data-valid
# + 2 ns board + 18 ns input Tsu = 55 ns. The board allowance is assumed.
# Clock-referenced budgets include PLL clock insertion; SCK is not a
# continuous generated clock. SDI changes on SCK falling, sampled on rising.
set_max_delay 18.000 -from [get_ports ADC_SDO]
set_min_delay 0.000 -from [get_ports ADC_SDO]
set_max_delay 18.000 -to [get_ports {ADC_SCK ADC_SDI ADC_CONVST}]
set_min_delay 0.000 -to [get_ports {ADC_SCK ADC_SDI ADC_CONVST}]
