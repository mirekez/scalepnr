#pragma once

#include "Module.h"
#include "Conn.h"
#include "referable.h"

namespace rtl
{

struct Cell
{
    std::string name;
    Ref<Module> module;
    std::vector<Referable<Conn>> conns;
    std::vector<std::unique_ptr<Referable<Cell>>> cells;
    int depth;
    int height;
    int primitive;
};





}
