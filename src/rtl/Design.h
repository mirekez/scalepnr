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
    Referable<CellInst> insts;
    std::unordered_map<std::string,Referable<Module>&> modules_map;

    bool build(const std::string& top_module)
    {
        PNR_LOG("RTL ", "building hierarchy...");
        for (auto& module : modules) {
            modules_map.emplace(module.name, module);
        }
        for (auto& module : modules) {
            PNR_LOG1("RTL ", "checking module '{}'...", module.name);
            for (auto& cell : module.cells) {
                auto submod_it = modules_map.find(cell.type);
                if (submod_it == modules_map.end()) {
                    PNR_ERROR("Cant find module with name '{}' for cell '{}'\n", cell.name, cell.type);
                    return false;
                }
                module.submodules.emplace_back(Ref<Module>());
                module.submodules.back().set(&submod_it->second);
                PNR_LOG2("RTL ", "adding submodule '{}' to {}", submod_it->second.name, module.name);
            }
        }

        auto topmod_it = modules_map.find(top_module);
        if (topmod_it == modules_map.end()) {
            PNR_ERROR("Cant find module with name '{}' for being a top module\n", top_module);
            return false;
        }
        PNR_LOG("RTL ", "found top module: '{}'", topmod_it->second.name);
        return build_hier(&insts, topmod_it->second, 0) != 0;
    }

    int build_hier(Referable<CellInst>* inst, Referable<Module>& mod, int level, Referable<Cell>* cell = 0, std::string hier_name = "")
    {
//        PNR_LOG1("RTL ", "instantiating module: '{}'...", mod.name);
        inst->depth = level++;
        if (cell) {
            inst->cell.set(cell);
        }
        hier_name = cell ? (hier_name != "" ? hier_name + "|" : "") + cell->name : hier_name;
        inst->module.set(&mod);
        int max_height = 0;
        for (auto& sub_cell : mod.cells) {
            inst->insts.emplace_back(std::make_unique<Referable<CellInst>>(CellInst{}));
            auto mod_it = modules_map.find(sub_cell.type);
            if (mod_it == modules_map.end()) {
                PNR_ERROR("Cant find module with name '{}' for cell '{}' in inst '{}'\n", sub_cell.type, sub_cell.name, hier_name == "" ? "top" : hier_name);
                return 0;
            }
            PNR_LOG2("RTL ", "instantiating cell: '{}' ({}) of module: {}, level: {}", hier_name != "" ? hier_name + "|" + sub_cell.name : sub_cell.name, sub_cell.type, mod.name, level);
            int height = 0;
            if ((height = build_hier(inst->insts.back().get(), mod_it->second, level, &sub_cell, hier_name)) == 0) {
                return 0;
            }
            if (height > max_height) {
                max_height = height;
            }
        }
        inst->height = max_height;
        return max_height + 1;
    }

};





}
