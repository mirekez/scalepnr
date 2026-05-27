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
        std::string xraymap = "BEG=SRC;END=DST;_S0=_SA;_S3=_SD;_N3=_ND;BOUNCE=JOINTA;ALT=JOINTB\nW=6:1,2,4,6;E=2:1,2,4,6;NW=7:1,1,2,3;NE=1:1,1,2,3;N=0:1,2,4,6;SW=5:1,2,2,3;SE=3:1,2,2,3;S=4:1,2,4,6\nLOGIC_OUTS=0;IMUX=1;BYP=2;GFAN=2";
        technology::readTechMap(xraymap, map);
        inited = true;
    }
    return map;
}

TechMap& tilePortsTechMap()
{
    static TechMap map;
    static bool inited = false;
    if (!inited) {
        std::string xraymapports =
        "39WE,17AI,16A,17A1,18A2,19A3,20A4,21A5,22A6,17AMUX,1AQ,31AX,80B,81B1,82B2,83B3,84B4,85B5,86B6,81BMUX,"
        "65BQ,95BX,144C,145C1,146C2,147C3,148C4,149C5,150C6,1CE,9CIN,0CLK,145CMUX,63COUT,129CQ,130CX,212D,213D1,"
        "214D2,215D3,216D4,217D5,218D6,213DMUX,197DQ,198DX,199SR";
        technology::readTechMap(xraymapports, map);
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
