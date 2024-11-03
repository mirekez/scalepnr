#pragma once

#include "Module.h"
#include "Cell.h"
#include "referable.h"

namespace rtl
{

struct Design
{
    std::vector<Referable<Module>> modules;
//    std::vector<Referable<Cell>> cells;


};





}
