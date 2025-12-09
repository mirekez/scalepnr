#include "Design.h"
#include "Clocks.h"
#include "Tech.h"

#include "tcl_pnr.h"

#include <ranges>

int
set_property_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 4) {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    std::string prop = Tcl_GetString(objv[1]);
    std::string val = Tcl_GetString(objv[2]);
    std::string port = Tcl_GetString(objv[3]);

    if (port.length() > 2 && port.front() == '{' && port.back() == '}') {
        port = port.substr(1, port.length()-2);
    }

    if (!port.length()) {
        std::print("\ncan't find obj for value '{}'\n", val);
    }

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    if (prop == "PACKAGE_PIN" && port.length()) {
        std::vector<Referable<rtl::Conn>*> conns;
        rtl::connFilter filter;
        filter.partial = false;
        filter.regexp = false;
//        filter.skip_braces = true;
        filter.port_name = port;
        rtl::getConns(&conns, std::move(filter), &Tech::current().design.top);
        if (conns.size()) {
            auto& tech = Tech::current();
            auto& assignments = tech.assignments;
            assignments[port] = val;
            std::print("\nassignment '{}' to '{}'", val, port);

            Tcl_Obj *wordObj = Tcl_NewStringObj("OK", -1);
            Tcl_ListObjAppendElement(interp, list_obj, wordObj);
        }
    }
    Tcl_SetObjResult(interp, list_obj);

    std::print("\n");
    return TCL_OK;

}
