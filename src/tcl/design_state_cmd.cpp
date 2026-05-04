#include "Tech.h"

#include "tcl_pnr.h"

using namespace technology;

int
write_design_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "filename");
        return TCL_ERROR;
    }

    Tech::current().writeDesignState(Tcl_GetString(objv[1]));
    return TCL_OK;
}

int
read_design_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "filename");
        return TCL_ERROR;
    }

    Tech::current().readDesignState(Tcl_GetString(objv[1]));
    return TCL_OK;
}
