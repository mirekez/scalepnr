# Apply after loading this design, before Estimate/placement.
create_clock -name clk_a -period 8 [get_ports clk_a]
create_clock -name clk_b -period 12 [get_ports clk_b]
