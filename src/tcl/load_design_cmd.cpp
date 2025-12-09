#include "Tech.h"

#include "tcl_pnr.h"

#include <ranges>

int
load_design_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 3) {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    std::string filename = Tcl_GetString(objv[1]);
    std::string top_module = Tcl_GetString(objv[2]);

    Tech::current().loadDesign(filename, top_module);

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;

}
