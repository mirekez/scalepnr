#pragma once

#include "Inst.h"

#include <re2/re2.h>
#include <memory>
#include <vector>
#include <string>

namespace rtl
{

struct instFilter
{
    bool partial = false;
    bool regexp = false;
    bool blackbox = false;
    // AND
    std::string name;
    std::string port_name;
    std::string cell_name;
    std::string cell_type;
    std::unique_ptr<re2::RE2> name_regex;
    std::unique_ptr<re2::RE2> port_regex;
    std::unique_ptr<re2::RE2> cell_regex;
    std::unique_ptr<re2::RE2> type_regex;
};

bool compare(const std::string& value, const std::string& mask, bool partial, bool regexp, re2::RE2& regex);
void getInsts(std::vector<Inst*>* insts, std::vector<instFilter> &filters, Referable<Inst>* inst = nullptr);
inline void getInsts(std::vector<Inst*>* insts, instFilter&& filter, Referable<Inst>* inst = nullptr)
{
    std::vector<instFilter> filters;
    filters.emplace_back(std::move(filter));
    getInsts(insts, filters, inst);
}


}
