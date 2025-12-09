#include "Design.h"
#include "Clocks.h"
#include "Tech.h"

#include "tcl_pnr.h"

#include <ranges>

int
check_timing_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 1) {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    auto& tech = Tech::current();
    tech.estimateTimings();

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;

}
