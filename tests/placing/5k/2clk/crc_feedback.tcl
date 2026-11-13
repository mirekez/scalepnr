# Apply after loading this design, before Estimate/placement.
create_clock -name clk_a -period 5 [get_ports clk_a]
create_clock -name clk_b -period 7.5 [get_ports clk_b]
