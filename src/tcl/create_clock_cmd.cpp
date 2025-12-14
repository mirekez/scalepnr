#include "Design.h"
#include "Clocks.h"
#include "Tech.h"

#include "tcl_pnr.h"

#include <ranges>

using namespace technology;

int
create_clock_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 6) {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    if (std::string(Tcl_GetString(objv[1])) != "-name") {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    std::string name = Tcl_GetString(objv[2]);

    if (std::string(Tcl_GetString(objv[3])) != "-period") {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    double period = atof(Tcl_GetString(objv[4]));
    std::string port = Tcl_GetString(objv[5]);

    auto& tech = Tech::current();
    bool ret = false;
    if (port.length()) {
        ret = tech.clocks.addClocks(Tech::current().design, name, port, period, 50);
        if (!ret)  {
            tech.prepareTimingLists();
        }
    }
    else {
        std::print("\ncant find port for clock '{}'\n", name);
    }

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    if (ret) {
        Tcl_Obj *wordObj = Tcl_NewStringObj(name.c_str(), -1);
        Tcl_ListObjAppendElement(interp, list_obj, wordObj);
    }
    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;

}
