#pragma once

#include "Module.h"
#include "Cell.h"
#include "referable.h"

#include <list>

namespace rtl
{

struct Design
{
    std::list<Referable<Module>> modules;
    std::list<Referable<Cell>> cells;


};





}
