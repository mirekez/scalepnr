#pragma once

#include "Module.h"
#include "Conn.h"
#include "referable.h"

namespace rtl
{

struct Cell
{
    std::string name;
    std::string type;
    int depth = -1;
    int height = -1;
    std::vector<Referable<Conn>> conns;
    std::vector<std::unique_ptr<Referable<Cell>>> cells;
    Ref<Module> module;
};





}
