#include "Device.h"

using namespace fpga;

Device& Device::current()
{
    static Device current;
    return current;
}

