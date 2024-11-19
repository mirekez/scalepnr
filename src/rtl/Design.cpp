#include "Design.h"

using namespace rtl;

Design& Design::current()
{
    static Design current;
    return current;
}

void Design::getInsts(std::vector<Inst*>* insts, const std::string& name, const std::string& port_name, const std::string& cell_name, bool partial_name, Referable<Inst>* inst)
{
    if (!inst) {
        inst = &top;
    }

    std::string inst_name = inst->makeName();
    if (inst_name == name || (name.length() && partial_name && inst_name.find(name) != std::string::npos)) {
        insts->push_back(inst);
    }
    else
    if (inst->cell_ref->name == cell_name || (cell_name.length() && partial_name && inst->cell_ref->name.find(cell_name) != std::string::npos)) {
        insts->push_back(inst);
    }
    else
    for (auto& conn : inst->conns) {
        std::string port = inst_name + "." + conn.port_ref->name;
        if (port_name == port || (port_name.length() && partial_name && port_name.find(name) != std::string::npos)) {
            insts->push_back(inst);
            break;
        }
    }

    for (auto& sub_inst : inst->insts) {
        getInsts(insts, name, port_name, cell_name, partial_name, &sub_inst);
    }
}

void Design::countBlackboxes(std::map<std::string,size_t>* report, Referable<Inst>* inst)
{
    for (auto& sub_inst : inst->insts) {
        if (sub_inst.cell_ref->module_ref->blackbox) {
            ++(*report)[sub_inst.cell_ref->type];
            continue;
        }
        countBlackboxes(report, &sub_inst);
    }
}

void Design::printReport(reporter::builder* report, Referable<Inst>* inst, std::vector<std::pair<double,std::string>>* keys)
{
    bool master = false;
    ;
    if (!inst && !report && !keys) {
        inst = &top;
        report = new reporter::builder{};
        keys = new std::vector<std::pair<double,std::string>>();
        master = true;
    }

    for (auto& sub_inst : inst->insts) {
        if (sub_inst.cell_ref->module_ref->blackbox) {
            continue;
        }

        std::map<std::string,size_t> cnt;
        countBlackboxes(&cnt, &sub_inst);

        std::string summary;
        size_t overal = 0;
        for (auto& count : cnt)
        {
            cnt[count.first] += count.second;
            summary += count.first + ": " + std::to_string(count.second) + " ";
            ++overal;
        }
        std::string name = sub_inst.makeName();
        keys->push_back({overal, ""});
        reporter::key_line line{*keys, {std::move(name), std::move(summary)}, true};
        report->insert(std::move(line));

        printReport(report, &sub_inst, keys);
    }

    if (master) {
        std::print("\nRtl usage:\n");
        report->print_table();
        delete report;
        delete keys;
    }
}
