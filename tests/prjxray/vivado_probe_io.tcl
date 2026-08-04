create_project scalepnr_io_probe [file join [file dirname [file normalize [info script]]] vivado_io_probe_project] -part xc7a100tfgg676-1 -force
read_edif [file join [file dirname [file normalize [info script]]] vivado_export test.edf]
link_design -top test

set start [get_nodes -quiet {INT_L_X0Y116/IMUX_L34}]
set target [get_nodes -quiet {LIOI3_X0Y117/IOI_OLOGIC0_D1}]
set placed_target [get_nodes -quiet {LIOI3_X0Y117/IOI_OLOGIC1_D1}]
puts "PROBE start=$start target=$target"
puts "PROBE placed_target=$placed_target uphill=[get_nodes -quiet -uphill -of_objects $placed_target]"
puts "PROBE start_downhill=[get_nodes -quiet -downhill -of_objects $start]"
puts "PROBE target_uphill=[get_nodes -quiet -uphill -of_objects $target]"
foreach node [get_nodes -quiet -downhill -of_objects $start] {
    puts "PROBE second_from=$node downhill=[get_nodes -quiet -downhill -of_objects $node]"
}
close_project
