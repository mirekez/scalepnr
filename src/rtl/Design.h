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
    std::unordered_map<std::string,std::pair<Referable<Port>,Referable<Conn>>> global_ports;
    Referable<Conn>* GND;
    Referable<Conn>* VCC;

    bool build(const std::string& top_module)
    {
        auto bus0_it = global_ports.emplace("GND", std::make_pair(Port{.name = "GND", .bitnum = 0, .designator = -1, .type = Port::PORT_OUT, .global = true},Conn{})).first;
        bus0_it->second.second.port_ref.set(&bus0_it->second.first);
        bus0_it->second.second.inst_ref.set(&top);
        GND = &bus0_it->second.second;
        auto busV_it = global_ports.emplace("VCC", std::make_pair(Port{.name = "VCC", .bitnum = 0, .designator = -2, .type = Port::PORT_OUT, .global = true},Conn{})).first;
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

        // find top module
        auto topmod_it = modules_map.find(top_module);
        if (topmod_it == modules_map.end()) {
            PNR_ERROR("Cant find module with name '{}' for being a top module\n", top_module);
            return false;
        }

        PNR_LOG("RTL ", "found top module: '{}', creating top cell...", topmod_it->second.name);
        top_cell.name = "top";
        top_cell.type = topmod_it->second.name;
        top_cell.module_ref.set(&topmod_it->second);
        top_cell.ports.reserve(topmod_it->second.ports.size());
        for (auto& port : topmod_it->second.ports) {
            PNR_LOG1("RTL ", "creating top cell port '{}'[{}] ({})", port.name, port.bitnum, port.getType());

            top_cell.ports.emplace_back(
                Port{.name = port.name, .bitnum = port.bitnum, .designator = -1 /*top is alone*/, .type = port.type}
                );
        }

        if (build_hier(&top, top_cell) == 0) {
            return false;
        }

        if (!connect_hier(top)) {
            return false;
        }

        check_conns(top);
        return true;
    }

    int build_hier(Referable<CellInst>* inst, Referable<Cell>& cell, int level = 0, std::string hier_name = "")
    {
        inst->depth = level++;
        inst->cell_ref.set(&cell);
        hier_name = level != 0 ? (hier_name != "" ? hier_name + "|" : "") + cell.name : hier_name;

        PNR_LOG2("RTL ", "instantiating cell: '{}' ({}) level: {}...", hier_name, cell.type, level);

        // repeat connections in inst after cell
        inst->conns.reserve(cell.ports.size());
        for (auto& port : cell.ports) {
            PNR_LOG3("RTL ", "'{}'[{}] ", port.name, port.bitnum);
            auto* conn = &inst->conns.emplace_back(Conn{});
            conn->port_ref.set(&port);
            conn->inst_ref.set(inst);
        }

        // set up insts
        int max_height = 0;
        for (auto& sub_cell : inst->cell_ref->module_ref->cells) {
            auto* sub_inst = &inst->insts.emplace_back(CellInst{});
            sub_inst->cell_ref.set(&sub_cell);
            sub_inst->parent_ref.set(inst);

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

        PNR_LOG1("RTL ", "mapping connections in '{}' ({}), level: {}... ", inst.cell_ref->name, inst.cell_ref->type, level);
        level++;

        // making a map of designators on current level
        // ports
        for (auto& conn : inst.conns) {  // looking for internal designators corresponding to cell's external connections
            bool found = false;
            for (auto& mod_port : inst.cell_ref->module_ref->ports) {  // corresponding module ports
                if (mod_port.name == conn.port_ref->name && mod_port.bitnum == conn.port_ref->bitnum) {
                    conn.port_ref->sub_designator = mod_port.designator;

                    if (mod_port.designator >= 0) {
                        if (mod_port.type == Port::PORT_IN) {  // we make negative key for inputs (we dont want to search them)
                            PNR_LOG3("RTL ", "<{}>='{}'[{}] ", mod_port.designator, mod_port.name, mod_port.bitnum);
                            conns_map[mod_port.designator] = &conn;  // input port becomes output inside module
                        }
                        else {  // output port becomes input inside module, inout ports are considered as inputs
                            conns_map[-mod_port.designator] = &conn;
                        }
                        if (mod_port.type == Port::PORT_IO) {
                            std::string inst_name = inst.makeName();
                            std::string bus_name = inst_name + "|" + std::to_string(mod_port.designator);
                            PNR_LOG2("RTL ", "creating global bus '{}' for inout signal '{}'[{}] of cell '{}' ({}) designator <{}>", bus_name,
                                mod_port.name, mod_port.bitnum, inst.cell_ref->name, inst.cell_ref->type, mod_port.designator);
                            auto bus_it = global_ports.emplace(bus_name,
                                std::make_pair(
                                    Port{.name = bus_name, .bitnum = mod_port.bitnum, .designator = mod_port.designator, .type = Port::PORT_OUT, .global = true},
                                    Conn{}
                                    )
                                ).first;
                            bus_it->second.second.port_ref.set(&bus_it->second.first);
                            bus_it->second.second.inst_ref.set(&top);
                        }
                    } else {
                        PNR_ERROR("designator for port '{}'[{}] of module '{}' is set to GND/VCC/X/Z", mod_port.name, mod_port.bitnum, inst.cell_ref->type);
                        return false;
                    }
                    found = true;
                    break;
                }
            }
            if (!found) {
                PNR_ERROR("cant find port '{}'[{}] in module '{}' for connection of inst '{}'", conn.port_ref->name, conn.port_ref->bitnum,
                    inst.cell_ref->type, inst.cell_ref->name);
                return false;
            }
        }

        // cells
        for (auto& sub_inst : inst.insts) {
            for (auto& conn : sub_inst.conns) {
                if (conn.port_ref->designator >= 0) {
                    if (conn.port_ref->type == Port::PORT_OUT) {  // we make negative key for inputs (we dont want to search them)
                        PNR_LOG3("RTL ", "<{}>='{}/{}'[{}]{} ", conn.port_ref->designator, sub_inst.cell_ref->name, conn.port_ref->name,
                            conn.port_ref->bitnum, conn.port_ref->getTypeChar());
                        conns_map[conn.port_ref->designator] = &conn;  // outputs
                    }
                    else {  // inputs, inout ports are considered as inputs and become outputs
                        conns_map[-conn.port_ref->designator] = &conn;
                    }
                    if (conn.port_ref->type == Port::PORT_IO) {
                        std::string inst_name = inst.makeName();
                        std::string bus_name = inst_name + "|" + std::to_string(conn.port_ref->designator);
                        PNR_LOG2("RTL ", "creating global bus '{}' for inout signal '{}'[{}] of subcell '{}' ({}) designator <{}>", bus_name,
                            conn.port_ref->name, conn.port_ref->bitnum, sub_inst.cell_ref->name, sub_inst.cell_ref->type, conn.port_ref->designator);
                        auto bus_it = global_ports.emplace(bus_name,
                            std::make_pair(
                                Port{.name = bus_name, .bitnum = conn.port_ref->bitnum, .designator = conn.port_ref->designator, .type = Port::PORT_OUT, .global = true},
                                Conn{}
                                )
                            ).first;
                        bus_it->second.second.port_ref.set(&bus_it->second.first);
                        bus_it->second.second.inst_ref.set(&top);
                    }
                }
            }
        }

        PNR_LOG1("RTL ", "linking connections in '{}' ({}), level: {}... ", inst.cell_ref->name, inst.cell_ref->type, level);

        // linking connections
        // ports
        for (auto& conn : inst.conns) {  // we use internal sub_designator inside a cell
            if (conn.port_ref->type != Port::PORT_IN) {  // need OUTs and IOs, output becomes input inside module, inputs/ios need to be connected and possibly to one point
                if (conn.port_ref->sub_designator < 0) {  // tied to const
                    if (conn.port_ref->sub_designator == -1) {
                        PNR_LOG2("RTL ", "output port '{}'[{}] connected to '{}' (sub_designator: <{}>)", conn.port_ref->name,
                            conn.port_ref->bitnum, GND->port_ref->name, conn.port_ref->sub_designator);
                        conn./*output_ref.*/set(GND);
                    }
                    if (conn.port_ref->sub_designator == -2) {
                        PNR_LOG2("RTL ", "output port '{}'[{}] connected to '{}' (sub_designator: <{}>)", conn.port_ref->name,
                            conn.port_ref->bitnum, VCC->port_ref->name, conn.port_ref->sub_designator);
                        conn./*output_ref.*/set(VCC);
                    }
                    continue;
                }
                auto it = conns_map.find(conn.port_ref->sub_designator);  // looking for corresponding outputs
                bool found = false;
                while (it != conns_map.end() && it->first == conn.port_ref->sub_designator) {
                    PNR_LOG2("RTL ", "output port '{}'[{}] connected to cell '{}' ({}) input port '{}'[{}] (sub_designator: <{}>)", conn.port_ref->name,
                        conn.port_ref->bitnum, it->second->inst_ref->cell_ref->name, it->second->inst_ref->cell_ref->type, it->second->port_ref->name,
                        it->second->port_ref->bitnum, conn.port_ref->sub_designator);
                    conn./*output_ref.*/set(it->second);
                    ++it;
                    found = true;
                }
                if (!found) {
                    PNR_WARNING("cant find input for output '{}'[{}] sub_designator <{}>\n", conn.port_ref->name, conn.port_ref->bitnum, conn.port_ref->sub_designator);
                }
            }
        }

        // cells
        for (auto& sub_inst : inst.insts) {
            for (auto& conn : sub_inst.conns) {
                if (conn.port_ref->type != Port::PORT_OUT) {  // need INs and IOs, they all need to be connected and possibly to one point
                    if (conn.port_ref->designator < 0) {  // tied to const
                        if (conn.port_ref->designator == -1) {
                            PNR_LOG2("RTL ", "cell '{}' ({}) input port '{}'[{}] connected to '{}' (designator: <{}>)", sub_inst.cell_ref->name,
                                sub_inst.cell_ref->type, conn.port_ref->name, conn.port_ref->bitnum, GND->port_ref->name, conn.port_ref->designator);
                            conn./*output_ref.*/set(GND);
                        }
                        if (conn.port_ref->designator == -2) {
                            PNR_LOG2("RTL ", "cell '{}' ({}) input port '{}'[{}] connected to '{}' (designator: <{}>)", sub_inst.cell_ref->name,
                                sub_inst.cell_ref->type, conn.port_ref->name, conn.port_ref->bitnum, VCC->port_ref->name, conn.port_ref->designator);
                            conn./*output_ref.*/set(VCC);
                        }
                        continue;
                    }
                    auto it = conns_map.find(conn.port_ref->designator);  // looking for corresponding outputs
                    bool found = false;
                    while (it != conns_map.end() && it->first == conn.port_ref->designator) {
                        PNR_LOG2("RTL ", "cell '{}' ({}) input port '{}'[{}] connected to cell '{}' ({}) output port '{}'[{}] (designator: <{}>)",
                            sub_inst.cell_ref->name, sub_inst.cell_ref->type, conn.port_ref->name, conn.port_ref->bitnum, it->second->inst_ref->cell_ref->name,
                            it->second->inst_ref->cell_ref->type, it->second->port_ref->name, it->second->port_ref->bitnum, conn.port_ref->designator);
                        conn./*output_ref.*/set(it->second);
                        ++it;
                        found = true;
                    }
                    if (!found) {
                        PNR_WARNING("cant find output for input '{}'[{}] designator <{}> of cell '{}' ({})\n", conn.port_ref->name, conn.port_ref->bitnum,
                            sub_inst.cell_ref->name, sub_inst.cell_ref->type, conn.port_ref->designator);
                    }
                }
            }
        }

        ++level;
        // recursion
        for (auto& sub_inst : inst.insts) {
            if (!connect_hier(sub_inst, level)) {
                return false;
            }
        }

        return true;
    }


    bool check_conns(Referable<CellInst>& inst)
    {
        if (inst.cell_ref->name != "top") {
            for (auto& conn : inst.conns) {
                if (conn.port_ref->type != Port::PORT_OUT && conn./*output_ref.*/get() == nullptr) {
                    if (conn.port_ref->designator >= -2) {  // dont report on 'x' and 'z'
                        PNR_WARNING("connection '{}'[{}]{} of cell '{}' ({})' is floating, tied to GND", conn.port_ref->name, conn.port_ref->bitnum, conn.port_ref->getTypeChar(),
                            conn.inst_ref->cell_ref->name, conn.inst_ref->cell_ref->type);
                    }
                    conn./*output_ref.*/set(GND);
                }
                else {
                    for (auto* peer_ref: conn.dependencies) {
                        auto& peer = static_cast<Conn&>(*peer_ref);
                        if (peer.inst_ref.ref == conn.inst_ref->parent_ref.ref) {
                            if (peer.port_ref->type != Port::PORT_OUT && conn.port_ref->type == Port::PORT_OUT ) {
                                PNR_ERROR("internal error: input port '{}'[{}] of '{}' ({}) is connected to output port '{}'[{}] of '{}'",
                                    peer.port_ref->name, peer.port_ref->bitnum, peer.inst_ref->cell_ref->name, peer.inst_ref->cell_ref->type,
                                    conn.port_ref->name, conn.port_ref->bitnum, conn.inst_ref->cell_ref->name, conn.inst_ref->cell_ref->type);
                                return false;
                            }
                            if (peer.port_ref->type == Port::PORT_OUT && conn.port_ref->type != Port::PORT_OUT ) {
                                PNR_ERROR("internal error: output port '{}'[{}] of '{}' ({}) is connected to input port '{}'[{}] of '{}'",
                                    peer.port_ref->name, peer.port_ref->bitnum, peer.inst_ref->cell_ref->name, peer.inst_ref->cell_ref->type,
                                    conn.port_ref->name, conn.port_ref->bitnum, conn.inst_ref->cell_ref->name, conn.inst_ref->cell_ref->type);
                                return false;
                            }
                        } else
                        if (conn.inst_ref.ref == peer.inst_ref->parent_ref.ref) {
                            if (conn.port_ref->type != Port::PORT_OUT && peer.port_ref->type == Port::PORT_OUT ) {
                                PNR_ERROR("internal error: input port '{}'[{}] of '{}' ({}) is connected to output port '{}'[{}] of '{}'",
                                    conn.port_ref->name, conn.port_ref->bitnum, conn.inst_ref->cell_ref->name, conn.inst_ref->cell_ref->type,
                                    peer.port_ref->name, peer.port_ref->bitnum, peer.inst_ref->cell_ref->name, peer.inst_ref->cell_ref->type);
                                return false;
                            }
                            if (conn.port_ref->type == Port::PORT_OUT && peer.port_ref->type != Port::PORT_OUT ) {
                                PNR_ERROR("internal error: output port '{}'[{}] of '{}' ({}) is connected to input port '{}'[{}] of '{}'",
                                    conn.port_ref->name, conn.port_ref->bitnum, conn.inst_ref->cell_ref->name, conn.inst_ref->cell_ref->type,
                                    peer.port_ref->name, peer.port_ref->bitnum, peer.inst_ref->cell_ref->name, peer.inst_ref->cell_ref->type);
                                return false;
                            }
                        } else
                        if (conn.inst_ref->parent_ref.ref == peer.inst_ref->parent_ref.ref) {
                            if (conn.port_ref->type != Port::PORT_OUT && peer.port_ref->type != Port::PORT_OUT ) {
                                PNR_ERROR("internal error: input port '{}'[{}] of '{}' ({}) is connected to input port '{}'[{}] of '{}'",
                                    conn.port_ref->name, conn.port_ref->bitnum, conn.inst_ref->cell_ref->name, conn.inst_ref->cell_ref->type,
                                    peer.port_ref->name, peer.port_ref->bitnum, peer.inst_ref->cell_ref->name, peer.inst_ref->cell_ref->type);
                                return false;
                            }
                            if (conn.port_ref->type == Port::PORT_OUT && peer.port_ref->type == Port::PORT_OUT ) {
                                PNR_ERROR("internal error: output port '{}'[{}] of '{}' ({}) is connected to output port '{}'[{}] of '{}'",
                                    conn.port_ref->name, conn.port_ref->bitnum, conn.inst_ref->cell_ref->name, conn.inst_ref->cell_ref->type,
                                    peer.port_ref->name, peer.port_ref->bitnum, peer.inst_ref->cell_ref->name, peer.inst_ref->cell_ref->type);
                                return false;
                            }
                        } else {
                            // only case of global Conns possible or error
                        }
                    }
                }
            }
        }

        for (auto& sub_inst : inst.insts) {
            if (!check_conns(sub_inst)) {
                return false;
            }
        }
        return true;
    }

};





}
