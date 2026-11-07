# Apply after loading this design, before Estimate/placement.
create_clock -name clk_a -period 6 [get_ports clk_a]
create_clock -name clk_b -period 9 [get_ports clk_b]
