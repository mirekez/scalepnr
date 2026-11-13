# Apply after loading this design, before Estimate/placement.
create_clock -name clk_a -period 8 [get_ports clk_a]
create_clock -name clk_b -period 12 [get_ports clk_b]
set_max_delay -datapath_only -from [get_clocks clk_a] -to [get_clocks clk_b] 6
set_max_delay -datapath_only -from [get_clocks clk_b] -to [get_clocks clk_a] 7
