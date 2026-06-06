set test_dir [file dirname [file normalize [info script]]]
set db_dir [file join $test_dir db]

load_cb_spec [file join $db_dir tile_type_INT_R.json]
load_cb_spec [file join $db_dir tile_type_INT_L.json]

load_tiles_spec $db_dir

load_spec [file join $db_dir tilegrid.json] [file join $db_dir package_pins.csv]

load_design [file join $test_dir test.json] test
create_clock -name clk -period 5.0 [get_ports clk]

set_property IOSTANDARD LVCMOS33 [get_ports clk]
set_property PACKAGE_PIN G4 [get_ports clk]
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
set_property PACKAGE_PIN T14 [get_ports data_in[2]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[3]]
set_property PACKAGE_PIN K1 [get_ports data_in[3]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[4]]
set_property PACKAGE_PIN R7 [get_ports data_in[4]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[5]]
set_property PACKAGE_PIN J4 [get_ports data_in[5]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[6]]
set_property PACKAGE_PIN N2 [get_ports data_in[6]]
set_property IOSTANDARD LVCMOS33 [get_ports data_in[7]]
set_property PACKAGE_PIN G1 [get_ports data_in[7]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[0]]
set_property PACKAGE_PIN E17 [get_ports data_out[0]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[1]]
set_property PACKAGE_PIN H14 [get_ports data_out[1]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[2]]
set_property PACKAGE_PIN E6 [get_ports data_out[2]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[3]]
set_property PACKAGE_PIN R14 [get_ports data_out[3]]
set_property IOSTANDARD LVCMOS33 [get_ports data_out[4]]
set_property PACKAGE_PIN B2 [get_ports data_out[4]]
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
set wires_before [get_wires [get_nets *]]
puts "ROUTED_WIRES=$wires_before"

set state_file [file join $test_dir design_state.db]
write_design $state_file
read_design $state_file
set wires_after [get_wires [get_nets *]]
puts "ROUTED_WIRES_AFTER_READ=$wires_after"
if {$wires_before ne $wires_after} {
    puts "WARN: read_design did not restore routed wires exactly"
}

#check_timing
