#include "Design.h"

using namespace rtl;

void Design::countBlackboxes(std::map<std::string,size_t>* report, Referable<Inst>* inst)
{
    for (auto& sub_inst : inst->insts) {
        if (sub_inst.cell_ref->module_ref->is_blackbox) {
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

    std::map<std::string,size_t> cnt;
    countBlackboxes(&cnt, inst);

    std::string summary;
    size_t overal = 0;
    for (auto& count : cnt)
    {
        cnt[count.first] += count.second;
        summary += count.first + ": " + std::to_string(count.second) + " ";
        ++overal;
    }
    std::string name = inst->makeName();
    keys->push_back({overal, ""});
    reporter::key_line line{*keys, {std::move(name), std::move(summary)}, true};
    report->insert(std::move(line));

    for (auto& sub_inst : inst->insts) {
        if (sub_inst.cell_ref->module_ref->is_blackbox) {
            continue;
        }

        printReport(report, &sub_inst, keys);
    }

    if (master) {
        std::print("\nRtl usage:\n");
        report->print_table();
        delete report;
        delete keys;
    }
}

