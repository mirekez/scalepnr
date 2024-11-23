#include "Device.h"
#include "Design.h"
#include "RtlFormat.h"
#include "XC7Tech.h"

#include <io.h>
#include <fcntl.h>
#include "tcl_pnr.h"  // TCL headers fight with std::regexp

using namespace gear;

#include <re2/re2.h>

using namespace std;

int main(int argc, char** argv)
{
    _setmode(_fileno(stdout), _O_BINARY);
    _setmode(_fileno(stderr), _O_BINARY);

    gear::Device::current().loadFromSpec("xc7a100t");
    rtl::Design& rtl = rtl::Design::current();
    RtlFormat rtl_format;
    rtl_format.loadFromJson("TestPipeline.json", &rtl);
    rtl.build("TestPipeline");
    rtl.printReport();

    std::print("\nscalepnr");
    Tcl_Main(argc, argv, Tcl_AppInit);
}
