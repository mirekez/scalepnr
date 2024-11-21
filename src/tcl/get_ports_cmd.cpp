#include "Design.h"
#include "getInsts.h"
#include "tcl_pnr.h"

#include <ranges>

int
get_ports_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 2) {
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

    std::vector<rtl::Inst*> insts;
    rtl::instFilter filter;
    filter.partial = partial_name;
    filter.port_name = mask;
    rtl::getInsts(&insts, std::move(filter), &rtl::Design::current().top);

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    for (auto* inst : insts) {
        for (auto& conn : inst->conns) {
            auto iname = inst->makeName();
            std::string name = iname + (iname.length()?".":"") + conn.port_ref->name;
            if (name == mask || (partial_name && name.find(mask) != std::string::npos)) {
                Tcl_Obj *wordObj = Tcl_NewStringObj(name.c_str(), -1);
                Tcl_ListObjAppendElement(interp, list_obj, wordObj);
            }
        }
    }

    Tcl_SetObjResult(interp, list_obj);
    return TCL_OK;

}
