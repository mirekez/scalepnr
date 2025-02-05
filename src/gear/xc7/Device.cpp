#include "Device.h"

using namespace gear;

Device& Device::current()
{
    static Device current;
    return current;
}

