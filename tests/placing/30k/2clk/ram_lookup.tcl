# Apply after loading this design, before Estimate/placement.
create_clock -name clk_a -period 10 [get_ports clk_a]
create_clock -name clk_b -period 15 [get_ports clk_b]
