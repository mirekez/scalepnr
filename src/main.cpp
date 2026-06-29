#include "Device.h"
#include "Design.h"
#include "RtlFormat.h"
#include "Tech.h"

#include <cstdlib>
#include <iostream>
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
    debug_module[CALC_MOD_MASK_INDEX("CLKS")] |= CALC_MOD_MASK_VALUE("CLKS");
//    debug_module[CALC_MOD_MASK_INDEX("CBAR")] |= CALC_MOD_MASK_VALUE("CBAR");
//    debug_module[CALC_MOD_MASK_INDEX("OUTL")] |= CALC_MOD_MASK_VALUE("OUTL");
//    debug_module[CALC_MOD_MASK_INDEX("ESTM")] |= CALC_MOD_MASK_VALUE("ESTM");
    debug_level = 1;
    if (const char* env_debug_level = std::getenv("SCALEPNR_DEBUG_LEVEL")) {
        char* end = nullptr;
        long parsed = std::strtol(env_debug_level, &end, 10);
        if (end != env_debug_level && parsed >= 0 && parsed <= 4) {
            debug_level = static_cast<int>(parsed);
        }
    }

    std::print("\nscalepnr");
    Tcl_Main(argc, argv, Tcl_AppInit);
}

//size_t allocated = 0;
