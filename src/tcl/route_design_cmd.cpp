#include "Design.h"
#include "Tech.h"
#include "pnr.h"

#include "tcl_pnr.h"

#include <ranges>

using namespace technology;

int pnr_design_cmd(ClientData, Tcl_Interp* interp, int objc, Tcl_Obj* const objv[])
{
    pnr::Pnr flow(Tech::current());
    if (objc != 1 && objc != 3) {
        Tcl_WrongNumArgs(interp, 1, objv, "?-run_cells positive_integer?");
        return TCL_ERROR;
    }
    if (objc == 3) {
        Tcl_WideInt count = 0;
        if (std::string_view(Tcl_GetString(objv[1])) != "-run_cells" ||
            Tcl_GetWideIntFromObj(interp, objv[2], &count) != TCL_OK || count <= 0) {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("expected -run_cells positive_integer", -1));
            return TCL_ERROR;
        }
        flow.run_cells = static_cast<size_t>(count);
    }
    if (!flow.run()) {
        Tcl_SetObjResult(interp, Tcl_NewStringObj(flow.failure.c_str(), -1));
        return TCL_ERROR;
    }
    Tcl_SetObjResult(interp, Tcl_NewWideIntObj(flow.runs.size()));
    return TCL_OK;
}

int
route_design_cmd(
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
    tech.routeDesign();

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;

}

int
route_clocks_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 1) {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    bool routed = Tech::current().routeClocks();
    Tcl_SetObjResult(interp, Tcl_NewBooleanObj(routed));
    std::print("\n");
    return routed ? TCL_OK : TCL_ERROR;
}
