#include "Clocks.h"

using namespace clocks;

Clocks& Clocks::current()
{
    static Clocks current;
    return current;
}

bool Clocks::getClocks(std::vector<Clock*>* insts, std::string name, bool partial_name)
{
    return true;
}

