#include "tclInt.h"

int TclPnr_Init(Tcl_Interp *interp);
int Tcl_AppInit(Tcl_Interp *interp);

int get_tiles_cmd(ClientData unused, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]);
int get_ports_cmd(ClientData unused, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]);
int create_clock_cmd(ClientData unused, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]);
int check_timing_cmd(ClientData unused, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]);
int load_design_cmd(ClientData unused, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]);
int open_design_cmd(ClientData unused, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]);
int place_design_cmd(ClientData unused, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]);
int print_design_cmd(ClientData unused, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]);
int set_property_cmd(ClientData unused, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]);

