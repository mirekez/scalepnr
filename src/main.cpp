#include "Device.h"
#include "Design.h"
#include "RtlFormat.h"
#include "Tech.h"

#include <iostream>
#include <filesystem>
#include <string>

#include <fcntl.h>
#include "tcl_pnr.h"  // TCL headers fight with std::regexp

using namespace fpga;

#include <re2/re2.h>

using namespace std;

int main(int argc, char** argv)
{
#ifdef WIN32  // switch \r off
    _setmode(_fileno(stdout), _O_BINARY);
    _setmode(_fileno(stderr), _O_BINARY);
#endif

    debug_module[CALC_MOD_MASK_INDEX("ROUT")] |= CALC_MOD_MASK_VALUE("ROUT");
    debug_module[CALC_MOD_MASK_INDEX("FRMT")] |= CALC_MOD_MASK_VALUE("FRMT");
    debug_module[CALC_MOD_MASK_INDEX("FPGA")] |= CALC_MOD_MASK_VALUE("FPGA");
    debug_module[CALC_MOD_MASK_INDEX("PLCE")] |= CALC_MOD_MASK_VALUE("PLCE");
//    debug_module[CALC_MOD_MASK_INDEX("CBAR")] |= CALC_MOD_MASK_VALUE("CBAR");
    debug_level = 1;

    TechMap map;  // ; = , :
    std::string xraymapports =
    "39WE,17AI,16A,17A1,18A2,19A3,20A4,21A5,22A6,17AMUX,1AQ,31AX,80B,81B1,82B2,83B3,84B4,85B5,86B6,81BMUX,"
    "65BQ,95BX,144C,145C1,146C2,147C3,148C4,149C5,150C6,1CE,9CIN,0CLK,145CMUX,63COUT,129CQ,130CX,212D,213D1,"
    "214D2,215D3,216D4,217D5,218D6,213DMUX,197DQ,198DX,199SR";
    technology::readTechMap(xraymapports, map);

    std::string xraymap = "BEG=SRC;END=DST;_S0=_SA;_S3=_SD;_N3=_ND;BOUNCE=JOINTA;ALT=JOINTB\nW=6:1,2,4,6;E=2:1,2,4,6;NW=7:1,1,2,3;NE=1:1,1,2,3;N=0:1,2,4,6;SW=5:1,2,2,3;SE=3:1,2,2,3;S=4:1,2,4,6";
    technology::readTechMap(xraymap, map);

    std::filesystem::path dir = "db";  // directory to search
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        std::string name = entry.path().filename().string();
        if (name.find("_type_I") != std::string::npos) {
            fpga::Device::current().loadCBFromSpec(std::string("db/") + name, map);
        }
    }

    technology::Tech::current();  // to init device tile types
    fpga::Device::current().loadFromSpec("db/tilegrid.json", "db/package_pins.csv");

    std::print("\nscalepnr");
    Tcl_Main(argc, argv, Tcl_AppInit);
}

//size_t allocated = 0;
