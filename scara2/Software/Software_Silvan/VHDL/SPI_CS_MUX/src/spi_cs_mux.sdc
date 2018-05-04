# Constrain clock port clk with a 10-ns requirement

create_clock -period 10 [get_ports isl_clk]
create_clock -period 125 [get_ports isl_SCLK]

# Automatically apply a generate clock on the output of phase-locked loops (PLLs)
# This command can be safely left in the SDC even if no PLLs exist in the design

derive_pll_clocks
