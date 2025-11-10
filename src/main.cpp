#include "Device.h"
#include "Design.h"
#include "RtlFormat.h"
#include "Tech.h"

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

    Tech::current();  // to init device tile types
    fpga::Device::current().loadFromSpec();

    std::print("\nscalepnr");
    Tcl_Main(argc, argv, Tcl_AppInit);
}

//size_t allocated = 0;
