#include "Device.h"
#include "tcl_pnr.h"

#include <ranges>

int
get_tiles_cmd(
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
    if (mask[0] == '*') {
        mask = mask.substr(1);
    }
    if (mask.back() == '*') {
        mask.pop_back();
    }
    Tcl_Obj *listObj = Tcl_NewListObj(0, NULL);
    for (const auto& tile : std::views::reverse(gear::Device::current().tileGrid)) {
        std::string name = tile.getName();
        if (name.find(mask) != (size_t)-1) {
            Tcl_Obj *wordObj = Tcl_NewStringObj(name.c_str(), -1);
            Tcl_ListObjAppendElement(interp, listObj, wordObj);
        }
    }

    Tcl_SetObjResult(interp, listObj);
    return TCL_OK;

}
