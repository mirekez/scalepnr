#pragma once

#include "Module.h"
#include "Conn.h"
#include "referable.h"

namespace gear
{

class Cell: public Referable
{
    std::string name;
    Ref<Module> module;
    std::vector<Conn> conns;
    std::vector<std::unique_ptr<Cell>> cells;
};





}
