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
    std::vector<Referable<Port>> ports;
};

struct CellInst
{
    int depth = -1;
    int height = -1;
    Ref<Cell> cell_ref;
    Ref<Module> module_ref;
    std::vector<Referable<Conn>> conns;

    Ref<CellInst> parent;
    std::list</*std::unique_ptr<*/Referable<CellInst>/*>*/> insts;
};



}
