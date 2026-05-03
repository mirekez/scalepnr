set test_dir [file dirname [file normalize [info script]]]
set db_dir [file join $test_dir db]

load_cb_spec [file join $db_dir tile_type_INT_R.json]
load_cb_spec [file join $db_dir tile_type_INT_L.json]

load_tiles_spec [file join $db_dir tile_type_CLBLL_L.json]
load_tiles_spec [file join $db_dir tile_type_CLBLL_R.json]
load_tiles_spec [file join $db_dir tile_type_CLBLM_L.json]
load_tiles_spec [file join $db_dir tile_type_CLBLM_R.json]

load_spec [file join $db_dir tilegrid.json] [file join $db_dir package_pins.csv]

load_design [file join $test_dir test.json] test
create_clock -name clk -period 5.0 [get_ports clk]

set_property IOSTANDARD LVCMOS33 [get_ports clk]
set_property PACKAGE_PIN C9 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports rst]
set_property PACKAGE_PIN K6 [get_ports rst]
set_property IOSTANDARD LVCMOS33 [get_ports in_valid]
set_property PACKAGE_PIN U1 [get_ports in_valid]
set_property IOSTANDARD LVCMOS33 [get_ports in_ready]
set_property PACKAGE_PIN B1 [get_ports in_ready]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[0]]
set_property PACKAGE_PIN N17 [get_ports data_in[0]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[1]]
set_property PACKAGE_PIN C17 [get_ports data_in[1]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[2]]
set_property PACKAGE_PIN T16 [get_ports data_in[2]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[3]]
set_property PACKAGE_PIN K13 [get_ports data_in[3]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[4]]
set_property PACKAGE_PIN R7 [get_ports data_in[4]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[5]]
set_property PACKAGE_PIN J4 [get_ports data_in[5]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[6]]
set_property PACKAGE_PIN N2 [get_ports data_in[6]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[7]]
set_property PACKAGE_PIN G3 [get_ports data_in[7]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[0]]
set_property PACKAGE_PIN E17 [get_ports data_out[0]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[1]]
set_property PACKAGE_PIN H14 [get_ports data_out[1]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[2]]
set_property PACKAGE_PIN E6 [get_ports data_out[2]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[3]]
set_property PACKAGE_PIN R11 [get_ports data_out[3]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[4]]
set_property PACKAGE_PIN B3 [get_ports data_out[4]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[5]]
set_property PACKAGE_PIN H17 [get_ports data_out[5]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[6]]
set_property PACKAGE_PIN N14 [get_ports data_out[6]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[7]]
set_property PACKAGE_PIN H6 [get_ports data_out[7]]

open_design
place_design
route_design
#print_design .*slice.20647.* 1000

puts "ROUTED_NETS=[get_nets *]"
puts "ROUTED_WIRES=[get_wires [get_nets *]]"

#check_timing
