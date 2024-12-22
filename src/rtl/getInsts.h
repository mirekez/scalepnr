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

    std::string format()
    {
        return std::format("name: '{}', port_name: '{}', cell_name: '{}', cell_type: '{}', partial: {}, regexp: {}, blackbox: {}",
            name, port_name, cell_name, cell_type, partial, regexp, blackbox);
    }
};

bool compare(const std::string& value, const std::string& mask, bool partial, bool regexp, re2::RE2& regex);

void getInsts(std::vector<Inst*>* insts, std::vector<instFilter>& filters, Referable<Inst>* inst = nullptr, int depth = 0);
void getInsts(std::vector<Inst*>* insts, instFilter&& filter, Referable<Inst>* inst = nullptr);

}
