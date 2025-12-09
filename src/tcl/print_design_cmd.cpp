#include "Design.h"
#include "Clocks.h"
#include "Tech.h"

#include "tcl_pnr.h"

#include <ranges>

int
print_design_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 3) {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    std::string inst_name = Tcl_GetString(objv[1]);
    int limit = atoi(Tcl_GetString(objv[2]));

    auto& tech = Tech::current();
    tech.printDesign(inst_name, limit);

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;

}
