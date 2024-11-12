#pragma once

#include "Module.h"
#include "Cell.h"
#include "referable.h"

#include <list>
#include <unordered_map>

namespace rtl
{

struct Design
{
    std::list<Referable<Module>> modules;
    Referable<CellInst> top;
    Referable<Cell> top_cell;
    std::unordered_map<std::string,std::pair<Referable<Port>,Referable<Conn>>> special_ports;
    Referable<Conn>* GND;
    Referable<Conn>* VCC;

    bool build(const std::string& top_module)
    {
        auto bus0_it = special_ports.emplace("GND", std::make_pair(Port{"GND", -1, 0, Port::PORT_OUT, true},Conn{})).first;
        bus0_it->second.second.port_ref.set(&bus0_it->second.first);
        bus0_it->second.second.inst_ref.set(&top);
        GND = &bus0_it->second.second;
        auto busV_it = special_ports.emplace("VCC", std::make_pair(Port{"VCC", -2, 0, Port::PORT_OUT, true},Conn{})).first;
        busV_it->second.second.port_ref.set(&busV_it->second.first);
        busV_it->second.second.inst_ref.set(&top);
        VCC = &busV_it->second.second;

        PNR_LOG("RTL ", "building hierarchy...");
        std::unordered_map<std::string,Referable<Module>&> modules_map;
        for (auto& module : modules) {
            modules_map.emplace(module.name, module);
        }
        // set cells to modules refs
        for (auto& module : modules) {
            PNR_LOG1("RTL ", "checking module '{}'...", module.name);
            for (auto& cell : module.cells) {
                auto submod_it = modules_map.find(cell.type);
                if (submod_it == modules_map.end()) {
                    PNR_ERROR("Cant find module with name '{}' for cell '{}'\n", cell.name, cell.type);
                    return false;
                }
                cell.module_ref.set(&submod_it->second);
                bool found = false;
                for (auto& submod : module.submodules_ref) {
                    if (submod->name == cell.type) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    module.submodules_ref.emplace_back(Ref<Module>());
                    module.submodules_ref.back().set(&submod_it->second);
                    PNR_LOG2("RTL ", "adding submodule '{}' to {}", submod_it->second.name, module.name);
                }
            }
        }

        auto topmod_it = modules_map.find(top_module);
        if (topmod_it == modules_map.end()) {
            PNR_ERROR("Cant find module with name '{}' for being a top module\n", top_module);
            return false;
        }

        PNR_LOG("RTL ", "found top module: '{}', creating top cell...", topmod_it->second.name);
        top_cell.name = "top";
        top_cell.type = topmod_it->second.name;
        top_cell.module_ref.set(&topmod_it->second);
        top_cell.ports.reserve(topmod_it->second.up_ports.size());
        for (auto& port : topmod_it->second.up_ports) {
            PNR_LOG1("RTL ", "creating top cell port '{}'[{}] ({})", port.name, port.designator, port.getType());

            top_cell.ports.emplace_back(Port(port.name, port.designator, port.bitnum, port.type));
        }

        if (build_hier(&top, top_cell) == 0) {
            return false;
        }

        if (!connect_hier(top)) {
            return false;
        }
        return true;
    }

    int build_hier(Referable<CellInst>* inst, Referable<Cell>& cell, int level = 0, std::string hier_name = "")
    {
        inst->depth = level++;
        inst->cell_ref.set(&cell);
        hier_name = level != 0 ? (hier_name != "" ? hier_name + "|" : "") + cell.name : hier_name;

        PNR_LOG2("RTL ", "instantiating cell: '{}' ({}) level: {}: ", hier_name, cell.type, level);

        inst->conns.reserve(cell.ports.size());
        std::string delim = "";
        for (auto& port : cell.ports) {  // repeat connections in CellInst after Cell
            PNR_LOG3("RTL ", "{}'{}'[{}]", delim, port.name, port.bitnum);
            delim = ", ";
            auto* conn = &inst->conns.emplace_back(Conn{});
            conn->port_ref.set(&port);
            conn->inst_ref.set(inst);
        }

        int max_height = 0;
        for (auto& sub_cell : inst->cell_ref->module_ref->cells) {
            auto* sub_inst = &inst->insts.emplace_back(CellInst{});
            sub_inst->cell_ref.set(&sub_cell);
            sub_inst->parent_ref.set(inst);

//            auto mod_it = modules_map.find(sub_cell.type);
//            if (mod_it == modules_map.end()) {
//                PNR_ERROR("Cant find module with name '{}' for cell '{}' in inst '{}'\n", sub_cell.type, sub_cell.name, hier_name == "" ? "top" : hier_name);
//                return 0;
//            }

            int height = 0;
            if ((height = build_hier(sub_inst, sub_cell, level, hier_name)) == 0) {
                return 0;
            }
            if (height > max_height) {
                max_height = height;
            }
//            sub_inst->parent.set(inst);
        }
        inst->height = max_height;
        return max_height + 1;
    }

    bool connect_hier(Referable<CellInst>& inst, int level = 0)
    {
        if (inst.cell_ref->module_ref->blackbox) {
            return true;
        }

        std::unordered_map<int,Referable<Conn>*> conns_map;

        // making a map of designators on current level
        // ports
        for (auto& conn : inst.conns) {
            bool found = false;
            for (auto& up_port : inst.cell_ref->module_ref->up_ports) {
                if (up_port.name == conn.port_ref->name && up_port.bitnum == conn.port_ref->bitnum) {
                    if (up_port.designator >= 0) {
                        if (up_port.type == Port::PORT_IN) {  // we make negative key for inputs (we dont want to search them)
                            conns_map[up_port.designator] = &conn;  // input port becomes output inside module
                        }
                        else {  // output port becomes input inside module, inout ports are considered as inputs
                            conns_map[-up_port.designator] = &conn;
                        }
                        if (up_port.type == Port::PORT_IO) {
                            std::string inst_name = inst.makeName();
                            std::string bus_name = inst_name + "|" + std::to_string(up_port.designator);
                            PNR_LOG2("RTL ", "creating special bus '{}' for inout signal '{}'[{}] of cell '{}' ({}) designator {}", bus_name,
                                up_port.name, up_port.bitnum, inst.cell_ref->name, inst.cell_ref->type, up_port.designator);
                            auto bus_it = special_ports.emplace(bus_name,
                                std::make_pair(Port{bus_name, up_port.designator, up_port.bitnum, Port::PORT_IO, true},Conn{})
                                ).first;
                            bus_it->second.second.port_ref.set(&bus_it->second.first);
                            bus_it->second.second.inst_ref.set(&top);
                        }
                    } else {
                        PNR_ERROR("designator of port {}[{}] of cell {} ({}) is set to const", up_port.name, up_port.bitnum, inst.cell_ref->name,
                            inst.cell_ref->type);
                        return false;
                    }
                    found = true;
                    break;
                }
            }
            if (!found) {
                PNR_ERROR("internal error: cant find up port '{}'[{}] in '{}' for connection of inst '{}'", conn.port_ref->name, conn.port_ref->bitnum,
                    inst.cell_ref->type, inst.cell_ref->name);
                return false;
            }
        }

        // cells
        for (auto& sub_inst : inst.insts) {
            for (auto& conn : sub_inst.conns) {
                if (conn.port_ref->designator >= 0) {
                    if (conn.port_ref->type == Port::PORT_OUT) {  // we make negative key for inputs (we dont want to search them)
                        conns_map[conn.port_ref->designator] = &conn;  // outputs
                    }
                    else {  // inputs, inout ports are considered as inputs and become outputs
                        conns_map[-conn.port_ref->designator] = &conn;
                    }
                    if (conn.port_ref->type == Port::PORT_IO) {
                        std::string inst_name = inst.makeName();
                        std::string bus_name = inst_name + "|" + std::to_string(conn.port_ref->designator);
                        PNR_LOG2("RTL ", "creating special bus '{}' for inout signal '{}'[{}] of subcell '{}' ({}) designator {}", bus_name,
                            conn.port_ref->name, conn.port_ref->bitnum, sub_inst.cell_ref->name, sub_inst.cell_ref->type, conn.port_ref->designator);
                        auto bus_it = special_ports.emplace(bus_name,
                            std::make_pair(Port{bus_name, conn.port_ref->designator, conn.port_ref->bitnum, Port::PORT_IO, true},Conn{})
                            ).first;
                        bus_it->second.second.port_ref.set(&bus_it->second.first);
                        bus_it->second.second.inst_ref.set(&top);
                    }
                }
            }
        }

        PNR_LOG1("RTL ", "connecting cell: '{}' ({}), level: {}... ", inst.cell_ref->name, inst.cell_ref->type, level);
        level++;

        // linking connections
        // ports
        for (auto& conn : inst.conns) {
            if (conn.port_ref->type != Port::PORT_IN) {  // need OUTs and IOs, output becomes input inside module, inputs/ios need to be connected and possibly to one point
                if (conn.port_ref->designator < 0) {  // tied to const
                    if (conn.port_ref->designator == -1) {
                        PNR_LOG2("RTL ", "output port '{}'[{}] connected to '{}' (designator: {})", conn.port_ref->name,
                            conn.port_ref->bitnum, GND->port_ref->name, conn.port_ref->designator);
                        conn.output_ref.set(GND);
                    }
                    if (conn.port_ref->designator == -2) {
                        PNR_LOG2("RTL ", "output port '{}'[{}] connected to '{}' (designator: {})", conn.port_ref->name,
                            conn.port_ref->bitnum, VCC->port_ref->name, conn.port_ref->designator);
                        conn.output_ref.set(VCC);
                    }
                    continue;
                }
                auto it = conns_map.find(conn.port_ref->designator);  // looking for outputs
                bool found = false;
                while (it != conns_map.end() && it->first == conn.port_ref->designator) {
                    PNR_LOG2("RTL ", "output port '{}'[{}] connected to cell '{}' ({}) input port '{}'[{}] (designator: {})", conn.port_ref->name,
                        conn.port_ref->bitnum, it->second->inst_ref->cell_ref->name, it->second->inst_ref->cell_ref->type, it->second->port_ref->name,
                        it->second->port_ref->bitnum, conn.port_ref->designator);
                    conn.output_ref.set(it->second);
                    ++it;
                    found = true;
                }
                if (!found) {
                    PNR_WARNING("cant find input for output '{}'[{}] designator {}\n", conn.port_ref->name, conn.port_ref->bitnum, conn.port_ref->designator);
                }
            }
        }

        // cells
        for (auto& sub_inst : inst.insts) {
            for (auto& conn : sub_inst.conns) {
                if (conn.port_ref->type != Port::PORT_OUT) {  // need INs and IOs, they all need to be connected and possibly to one point
                    if (conn.port_ref->designator < 0) {  // tied to const
                        if (conn.port_ref->designator == -1) {
                            PNR_LOG2("RTL ", "cell '{}' ({}) input port '{}'[{}] connected to '{}' (designator: {})", sub_inst.cell_ref->name,
                                sub_inst.cell_ref->type, conn.port_ref->name, conn.port_ref->bitnum, GND->port_ref->name, conn.port_ref->designator);
                            conn.output_ref.set(GND);
                        }
                        if (conn.port_ref->designator == -2) {
                            PNR_LOG2("RTL ", "cell '{}' ({}) input port '{}'[{}] connected to '{}' (designator: {})", sub_inst.cell_ref->name,
                                sub_inst.cell_ref->type, conn.port_ref->name, conn.port_ref->bitnum, VCC->port_ref->name, conn.port_ref->designator);
                            conn.output_ref.set(VCC);
                        }
                        continue;
                    }
                    auto it = conns_map.find(conn.port_ref->designator);  // looking for outputs
                    bool found = false;
                    while (it != conns_map.end() && it->first == conn.port_ref->designator) {
                        PNR_LOG2("RTL ", "cell '{}' ({}) input port '{}'[{}] connected to cell '{}' ({}) output port '{}'[{}] (designator: {})",
                            sub_inst.cell_ref->name, sub_inst.cell_ref->type, conn.port_ref->name, conn.port_ref->bitnum, it->second->inst_ref->cell_ref->name,
                            it->second->inst_ref->cell_ref->type, it->second->port_ref->name, it->second->port_ref->bitnum, conn.port_ref->designator);
                        conn.output_ref.set(it->second);
                        ++it;
                        found = true;
                    }
                    if (!found) {
                        PNR_WARNING("cant find output for input '{}'[{}] designator {} of cell '{}' ({})\n", conn.port_ref->name, conn.port_ref->bitnum,
                            sub_inst.cell_ref->name, sub_inst.cell_ref->type, conn.port_ref->designator);
                    }
                }
            }
        }

        // recursion
        for (auto& sub_inst : inst.insts) {
            if (!connect_hier(sub_inst, level)) {
                return false;
            }
        }

        return true;
    }
};





}
