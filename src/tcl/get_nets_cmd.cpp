#include "Tech.h"
#include "Wire.h"
#include "getConns.h"

#include "tcl_pnr.h"

#include <set>

using namespace technology;

namespace {

void collectRoutedNetNames(rtl::Inst& inst, std::set<std::string>& nets)
{
    for (const auto& route : inst.wires) {
        if (!route.empty() && !route.front().net_name.empty()) {
            nets.insert(route.front().net_name);
        }
    }

    for (auto& sub_inst : inst.insts) {
        collectRoutedNetNames(sub_inst, nets);
    }
}

}

int
get_nets_cmd(
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

    re2::RE2 regex(mask);
    std::set<std::string> nets;
    collectRoutedNetNames(Tech::current().design.top, nets);

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    for (const std::string& net : nets) {
        if (!rtl::compare(net, mask, partial_name, regexp, regex)) {
            continue;
        }
        Tcl_Obj *wordObj = Tcl_NewStringObj(net.c_str(), -1);
        Tcl_ListObjAppendElement(interp, list_obj, wordObj);
    }

    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;
}
