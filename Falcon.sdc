derive_pll_clocks
derive_clock_uncertainty

# core specific constraints
# Everything in the core runs on clk_sys (32 MHz, PLL output 0); there are
# no core clock domain crossings besides the sys/ framework's own.
