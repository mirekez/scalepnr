#pragma once

#include "Module.h"
#include "Inst.h"
#include "referable.h"
#include "debug.h"
#include "reporter.h"

#include <list>
#include <unordered_map>

namespace re2
{
    class RE2;
}

namespace rtl
{

struct Design
{
    std::list<Referable<Module>> modules;
    Referable<Inst> top;
    Referable<Cell> top_cell;
    std::unordered_map<std::string,std::pair<Referable<Port>,Referable<Conn>>> global_ports;
    Referable<Conn>* GND;
    Referable<Conn>* VCC;

    bool build(const std::string& top_module);
    int build_hier(Referable<Inst>* inst, Referable<Cell>& cell, int level = 0, std::string hier_name = "");
    bool connect_hier(Referable<Inst>& inst, int level = 0);
    bool check_conns(Referable<Inst>& inst, int level = 0);

    void countBlackboxes(std::map<std::string,size_t>* report, Referable<Inst>* inst);
    void printReport(reporter::builder* report = nullptr, Referable<Inst>* inst = nullptr, std::vector<std::pair<double,std::string>>* keys = 0);
//    void printDesign(Inst* inst = 0, bool noident = false, int level = 0);
};

}
