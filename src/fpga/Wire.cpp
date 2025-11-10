#include "Wire.h"

using namespace fpga;

void Wire::assign(rtl::Net* net)
{
    PNR_ASSERT(net->wire.peer == nullptr, "assigning wire {} to already assigned net {}", name, net->makeName());
    net->wire.set(static_cast<Referable<Wire>*>(this));
}

