#pragma once

#include "Module.h"
#include "Conn.h"
#include "referable.h"

#include <vector>
#include <list>

namespace rtl
{

struct Cell
{
    std::string name;
    std::string type;
    std::vector<Port> conns;
};

struct CellInst
{
    int depth = -1;
    int height = -1;
    Ref<Cell> cell;
    Ref<Module> module;
    std::vector<Referable<Conn>> conns;
    std::list<std::unique_ptr<Referable<CellInst>>> insts;
};



}
