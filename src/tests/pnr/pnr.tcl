# Exercise the real command registration and argument validation.
foreach args {{-run_cells 0} {-run_cells -1} {-run_cells nope} {-other 3} {-run_cells} {extra}} {
    if {![catch {pnr_design {*}$args} message]} {
        error "pnr_design accepted invalid arguments: $args"
    }
}
if {![catch {pnr_design} message] || ![string match {*load a design*} $message]} {
    error "pnr_design did not reject an unloaded design: $message"
}
puts "pnr Tcl regression passed"
