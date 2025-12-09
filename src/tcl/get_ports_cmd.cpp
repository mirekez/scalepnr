#include "Design.h"
#include "getConns.h"
#include "Tech.h"

#include "tcl_pnr.h"

#include <ranges>

int
get_ports_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc < 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    bool regexp = false;
    if (std::string(Tcl_GetString(objv[1])) == "-regexp") {
        regexp = true;
        --objc;
        ++objv;
    }

    if (objc < 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    std::string mask = Tcl_GetString(objv[1]);
    bool partial_name = false;
    if (mask[0] == '*') {
        partial_name = true;
        mask = mask.substr(1);
    }
    if (mask.length() && mask.back() == '*') {
        partial_name = true;
        mask.pop_back();
    }

    std::vector<Referable<rtl::Conn>*> conns;
    rtl::connFilter filter;
    filter.partial = partial_name;
    filter.regexp = regexp;
    filter.port_name = mask;
    rtl::getConns(&conns, std::move(filter), &Tech::current().design.top);

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    for (auto* conn : conns) {
        auto name = conn->makeName();
        Tcl_Obj *wordObj = Tcl_NewStringObj(name.c_str(), -1);
        Tcl_ListObjAppendElement(interp, list_obj, wordObj);
    }

    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;

}
