#include "Device.h"
#include "Tech.h"

#include "tcl_pnr.h"

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace {

TechMap& cbTechMap()
{
    static TechMap map;
    static bool inited = false;
    if (!inited) {
        technology::readTechMap(technology::a7CBTechMapText(), map);
        inited = true;
    }
    return map;
}

TechMap& tilePortsTechMap()
{
    static TechMap map;
    static bool inited = false;
    if (!inited) {
        technology::readTechMap(technology::a7TilePortsTechMapText(), map);
        inited = true;
    }
    return map;
}

}

int
load_spec_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 3) {
        Tcl_WrongNumArgs(interp, 1, objv, "tilegrid_filename package_pins_filename");
        return TCL_ERROR;
    }

    std::string tilegrid_filename = Tcl_GetString(objv[1]);
    std::string package_pins_filename = Tcl_GetString(objv[2]);
    fpga::Device::current().loadFromSpec(tilegrid_filename, package_pins_filename);

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;
}

int
load_cb_spec_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "filename");
        return TCL_ERROR;
    }

    std::string filename = Tcl_GetString(objv[1]);
    fpga::Device::current().loadCBFromSpec(filename, cbTechMap());

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;
}

int
load_tiles_spec_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "filename");
        return TCL_ERROR;
    }

    technology::Tech::current();
    std::string filename = Tcl_GetString(objv[1]);
    std::vector<std::filesystem::path> specs;
    std::filesystem::path path(filename);
    if (std::filesystem::is_directory(path)) {
        std::filesystem::path tileconn = path / "tileconn.json";
        if (std::filesystem::exists(tileconn)) {
            fpga::Device::current().loadTileConnFromSpec(tileconn.string());
        }
        for (const auto& entry : std::filesystem::directory_iterator(path)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            std::string name = entry.path().filename().string();
            if (name.rfind("tile_type_", 0) == 0 && entry.path().extension() == ".json") {
                specs.push_back(entry.path());
            }
        }
        std::sort(specs.begin(), specs.end());
    }
    else {
        std::filesystem::path tileconn = path.parent_path() / "tileconn.json";
        if (std::filesystem::exists(tileconn)) {
            fpga::Device::current().loadTileConnFromSpec(tileconn.string());
        }
        specs.push_back(path);
    }

    for (const auto& spec : specs) {
        fpga::Device::current().loadTypeFromSpec(spec.string(), tilePortsTechMap());
    }

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;
}
