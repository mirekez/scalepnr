#include "Device.h"
#include "tcl_pnr.h"

#include <io.h>
#include <fcntl.h>

#include "Design.h"
#include "RtlFormat.h"

using namespace gear;

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
