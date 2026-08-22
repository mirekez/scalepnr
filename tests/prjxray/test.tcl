set test_dir [file dirname [file normalize [info script]]]
set db_dir [file join $test_dir db]

set route_cb_types {
    BRAM_INT_INTERFACE_L
    BRAM_INT_INTERFACE_R
    CFG_CENTER_BOT
    CFG_CENTER_MID
    CFG_CENTER_TOP
    HCLK_BRAM
    HCLK_CLB
    HCLK_CMT
    HCLK_CMT_L
    HCLK_DSP_L
    HCLK_DSP_R
    HCLK_FEEDTHRU_1
    HCLK_FEEDTHRU_2
    HCLK_FIFO_L
    HCLK_GTX
    HCLK_INT_INTERFACE
    HCLK_IOB
    HCLK_IOI3
    HCLK_L
    HCLK_L_BOT_UTURN
    HCLK_R
    HCLK_R_BOT_UTURN
    HCLK_TERM
    HCLK_TERM_GTX
    HCLK_VBRK
    HCLK_VFRAME
    INT_FEEDTHRU_1
    INT_FEEDTHRU_2
    INT_INTERFACE_L
    INT_INTERFACE_R
    INT_L
    INT_R
    IO_INT_INTERFACE_L
    IO_INT_INTERFACE_R
    LIOI3
    LIOI3_SING
    LIOI3_TBYTESRC
    LIOI3_TBYTETERM
    L_TERM_INT
    MONITOR_BOT
    MONITOR_MID
    MONITOR_TOP
    PCIE_INT_INTERFACE_L
    PCIE_INT_INTERFACE_R
    RIOI3
    RIOI3_SING
    RIOI3_TBYTESRC
    RIOI3_TBYTETERM
    R_TERM_INT
    R_TERM_INT_GTX
    T_TERM_INT
    VBRK
    VBRK_EXT
    VFRAME
}

foreach cb_type $route_cb_types {
    set cb_file [file join $db_dir "tile_type_${cb_type}.json"]
    if {[file exists $cb_file]} {
        load_cb_spec $cb_file -constant-one VCC_WIRE -constant-zero GND_WIRE
    }
}

load_tiles_spec $db_dir

load_spec [file join $db_dir tilegrid.json] [file join $db_dir package_pins.csv]

# Dedicated local fabrics are loaded after generic jump subtypes are fixed so
# they cannot multiply ordinary routing subtypes.
foreach cb_type {
    BRKH_CLK
    CLK_BUFG_BOT_R
    CLK_BUFG_REBUF
    CLK_BUFG_TOP_R
    CLK_FEED
    CLK_HROW_BOT_R
    CLK_HROW_TOP_R
} {
    load_cb_spec [file join $db_dir "tile_type_${cb_type}.json"]
}
load_local_cb_spec [file join $db_dir tile_type_BRKH_INT.json]
foreach cb_type {
    CLK_MTBF2
    CLK_PMV
    CLK_PMV2
    CLK_PMV2_SVT
    CLK_PMVIOB
    CLK_TERM
} {
    set cb_file [file join $db_dir "tile_type_${cb_type}.json"]
    if {[file exists $cb_file]} {
        load_local_cb_spec $cb_file
    }
}
foreach cb_type $route_cb_types {
    if {[string match "HCLK_*" $cb_type]} {
        load_cb_spec [file join $db_dir "tile_type_${cb_type}.json"]
    }
}
load_tiles_spec [file join $db_dir tile_type_CLK_BUFG_BOT_R.json]
load_tiles_spec [file join $db_dir tile_type_CLK_BUFG_TOP_R.json]

set design_json [file join $test_dir test.json]
if {[info exists ::env(SCALEPNR_TEST_JSON)] && $::env(SCALEPNR_TEST_JSON) ne ""} {
    set design_json [file normalize $::env(SCALEPNR_TEST_JSON)]
}
set design_top test
if {[info exists ::env(SCALEPNR_TEST_TOP)] && $::env(SCALEPNR_TEST_TOP) ne ""} {
    set design_top $::env(SCALEPNR_TEST_TOP)
}
load_design $design_json $design_top

if {[info exists ::env(SCALEPNR_TEST_CONSTRAINTS)] && $::env(SCALEPNR_TEST_CONSTRAINTS) ne ""} {
    source [file normalize $::env(SCALEPNR_TEST_CONSTRAINTS)]
} else {
create_clock -name clk -period 5.0 [get_ports clk]

set_property IOSTANDARD LVCMOS33 [get_ports clk]
set_property PACKAGE_PIN G4 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports rst]
set_property PACKAGE_PIN K6 [get_ports rst]
set_property IOSTANDARD LVCMOS33 [get_ports in_valid]
set_property PACKAGE_PIN U1 [get_ports in_valid]
set_property IOSTANDARD LVCMOS33 [get_ports in_ready]
set_property PACKAGE_PIN B1 [get_ports in_ready]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[0]}]
set_property PACKAGE_PIN N17 [get_ports {data_in[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[1]}]
set_property PACKAGE_PIN C17 [get_ports {data_in[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[2]}]
set_property PACKAGE_PIN T14 [get_ports {data_in[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[3]}]
set_property PACKAGE_PIN K1 [get_ports {data_in[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[4]}]
set_property PACKAGE_PIN R7 [get_ports {data_in[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[5]}]
set_property PACKAGE_PIN J4 [get_ports {data_in[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[6]}]
set_property PACKAGE_PIN N2 [get_ports {data_in[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[7]}]
set_property PACKAGE_PIN G1 [get_ports {data_in[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[0]}]
set_property PACKAGE_PIN E17 [get_ports {data_out[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[1]}]
set_property PACKAGE_PIN H14 [get_ports {data_out[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[2]}]
set_property PACKAGE_PIN E6 [get_ports {data_out[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[3]}]
set_property PACKAGE_PIN P14 [get_ports {data_out[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[4]}]
set_property PACKAGE_PIN B2 [get_ports {data_out[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[5]}]
set_property PACKAGE_PIN H17 [get_ports {data_out[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[6]}]
set_property PACKAGE_PIN N14 [get_ports {data_out[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[7]}]
set_property PACKAGE_PIN H6 [get_ports {data_out[7]}]
}

open_design
set placement_loaded 0
if {[info exists ::env(SCALEPNR_PLACEMENT_DB)] &&
    $::env(SCALEPNR_PLACEMENT_DB) ne ""} {
    set placement_db [file normalize $::env(SCALEPNR_PLACEMENT_DB)]
    if {[file exists $placement_db]} {
        read_design $placement_db
        set placement_loaded 1
        puts "PLACEMENT_CACHE loaded=$placement_db"
    }
}
if {!$placement_loaded} {
    place_design
    if {[info exists ::env(SCALEPNR_PLACEMENT_DB)] &&
        $::env(SCALEPNR_PLACEMENT_DB) ne ""} {
        write_design $placement_db
        puts "PLACEMENT_CACHE written=$placement_db"
    }
}
if {[info exists ::env(SCALEPNR_CLOCK_ONLY)] && $::env(SCALEPNR_CLOCK_ONLY) ne ""} {
    route_clocks
} else {
    route_design
}
#print_design .*slice.20647.* 1000

set print_routed_wires 1
if {[info exists ::env(SCALEPNR_QUIET_ROUTES)] && $::env(SCALEPNR_QUIET_ROUTES) ne ""} {
    set print_routed_wires 0
}
if {$print_routed_wires} {
    puts "ROUTED_NETS=[get_nets *]"
    set wires_before [get_wires [get_nets *]]
    puts "ROUTED_WIRES=$wires_before"
}

if {![info exists ::env(SCALEPNR_SKIP_WRITE_DESIGN)] || $::env(SCALEPNR_SKIP_WRITE_DESIGN) eq ""} {
    set state_file [file join $test_dir design_state.db]
    if {[info exists ::env(SCALEPNR_DESIGN_DB)] && $::env(SCALEPNR_DESIGN_DB) ne ""} {
        set state_file [file normalize $::env(SCALEPNR_DESIGN_DB)]
    }
    write_design $state_file
    if {![info exists ::env(SCALEPNR_SKIP_READBACK)] || $::env(SCALEPNR_SKIP_READBACK) eq ""} {
        read_design $state_file
        if {$print_routed_wires} {
            set wires_after [get_wires [get_nets *]]
            puts "ROUTED_WIRES_AFTER_READ=$wires_after"
            if {$wires_before ne $wires_after} {
                puts "WARN: read_design did not restore routed wires exactly"
            }
        }
    }
}

#check_timing
