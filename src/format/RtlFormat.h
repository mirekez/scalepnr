#include <print>
#include <fstream>
#include <iostream>
#include <vector>
#include "json/json.h"
#include "Design.h"

struct RtlFormat
{
    bool loadFromJson(const std::string& filename, rtl::Design* design)
    {
        PNR_LOG("RTLF", "loadFromJson(), filename: '{}'", filename);
        std::ifstream infile(filename);
        if (!infile) {
            throw std::runtime_error(std::string("cant open file: ") + filename);
        }
        std::string line;
        std::string mod_json = "{";

        size_t start_indent = 0;
        size_t mods_indent = 0;
        int line_number = -1;
        while (std::getline(infile, line)) {
            ++line_number;
            size_t indent = 0;
            for (char ch : line) {
                if (ch == ' ') {
                    ++indent;
                }
                else break;
            }
            if (!start_indent && line.find("\"modules\": {") != std::string::npos) {
                start_indent = indent;
            }
            if (start_indent == 0) {
                continue;
            }
            if (!mods_indent && indent > start_indent) {
                mods_indent = indent;
            }
            if (mods_indent == 0) {
                continue;
            }
            if (indent >= mods_indent) {
                mod_json += line.c_str() + indent;
                if (line[mods_indent] == '}') {  // we collected all object
                    if (mod_json.back() == ',') {
                        mod_json.pop_back();
                    }
                    mod_json += '}';
                    try {
                        Json::Value root;
                        Json::Reader reader;
                        reader.parse(mod_json, root);
                        std::string mod_name = root.getMemberNames()[0];

                        auto* mod_ptr = &design->modules.emplace_back(
                            rtl::Module{.name = mod_name, .blackbox = atoi(root[mod_name]["attributes"]["blackbox"].asString().c_str()) != 0}
                            );
                        PNR_LOG1("RTLF", "loading module '{}': {}...", mod_name, atoi(root[mod_name]["attributes"]["blackbox"].asString().c_str()) != 0 ? "(blackbox)" : "");

                        // ports
                        if (root[mod_name].isMember("ports")) {

                            int count = 0;
                            for (auto it = root[mod_name]["ports"].begin(); it != root[mod_name]["ports"].end() ; it++) {
                                if ((*it).isMember("bits")) {
                                    count += (*it)["bits"].size();
                                }
                            }
                            mod_ptr->interface.reserve(count);
                            for (auto it = root[mod_name]["ports"].begin(); it != root[mod_name]["ports"].end() ; it++) {
                                if ((*it).isMember("bits")) {
                                    PNR_LOG2("RTLF", "creating port '{}' ({})...", it.key().asString(), (*it)["direction"].asString());

                                    int bitnum = -1;
                                    for (auto it1 = (*it)["bits"].begin(); it1 != (*it)["bits"].end() ; it1++) {
                                        if ((*it)["bits"].size() > 1) {
                                            ++bitnum;
                                        }
                                        int designator = -1;
                                        if ((*it1).type() == Json::ValueType::stringValue) {
                                            designator = (*it1).asString() == "0" ? -1 : ((*it1).asString() == "1" ? -2 : ((*it1).asString() == "z" ? -3 : -4));
                                        }
                                        else {
                                            designator = (*it1).asInt();
                                        }
                                        PNR_LOG3("RTLF", " [{}]<{}>", bitnum, designator);

                                        auto* port_ptr = &mod_ptr->interface.emplace_back(
                                            rtl::Port{.name = it.key().asString(), .bitnum = bitnum, .designator = designator}
                                            );
                                        port_ptr->setType((*it)["direction"].asString());
                                    }
                                }
                            }
                        }

                        // cells
                        if ((!root[mod_name]["attributes"].isMember("blackbox") || atoi(root[mod_name]["attributes"]["blackbox"].asString().c_str()) == 0)
                            && root[mod_name].isMember("cells")) {

                            for (auto it = root[mod_name]["cells"].begin(); it != root[mod_name]["cells"].end() ; it++) {

                                auto* cell_ptr = &mod_ptr->cells.emplace_back(
                                    rtl::Cell{.name = it.key().asString(), .type = (*it).isMember("type") ? (*it)["type"].asString() : std::string()}
                                    );
                                PNR_LOG2("RTLF", "creating cell '{}' ({})...", cell_ptr->name, cell_ptr->type);

                                if ((*it).isMember("port_directions")) {

                                    int count = 0;
                                    for (auto it1 = (*it)["connections"].begin(); it1 != (*it)["connections"].end(); it1++) {
                                        count += (*it1).size();
                                    }
                                    cell_ptr->ports.reserve(count);

                                    int i = -1;
                                    for (auto it1 = (*it)["connections"].begin(); it1 != (*it)["connections"].end() ; it1++) {
                                        ++i;
                                        std::string dir = "inout";
                                        if ((*it).isMember("port_directions")) {
                                            std::string dir_name = (*it)["port_directions"].getMemberNames()[i];
                                            if (it1.key().asString() == dir_name) {
                                                dir = (*it)["port_directions"][dir_name].asString();
                                            }
                                            else {
                                                PNR_WARNING("module '{}' cell '{}' connections and ports names mismatch: '{}' and '{}'", mod_name, it.key().asString(), it1.key().asString(), dir_name);
                                            }
                                        }
                                        else {
                                            PNR_WARNING("module '{}' cell '{}' cant find 'port_directions' field in JSON", mod_name, it.key().asString());
                                        }
                                        int bitnum = -1;
                                        for (auto it2 = (*it1).begin(); it2 != (*it1).end() ; it2++) {
                                            if ((*it1).size() > 1) {
                                                ++bitnum;
                                            }
                                            int designator = -1;
                                            if ((*it2).type() == Json::ValueType::stringValue) {
                                                designator = (*it2).asString() == "0" ? -1 : ((*it2).asString() == "1" ? -2 : ((*it2).asString() == "z" ? -3 : -4));
                                            }
                                            else {
                                                designator = (*it2).asInt();
                                            }

                                            auto* port_ptr = &cell_ptr->ports.emplace_back(
                                                rtl::Port{.name = it1.key().asString(), .bitnum = bitnum, .designator = designator}
                                                );
                                            port_ptr->setType(dir);

                                            PNR_LOG3("RTLF", " '{}'[{}]{}<{}>", it1.key().asString(), bitnum, port_ptr->getTypeChar(), designator);
                                        }
                                    }
                                }
                            }
                        }

                        if ((!root[mod_name]["attributes"].isMember("blackbox") || atoi(root[mod_name]["attributes"]["blackbox"].asString().c_str()) == 0)
                            && root[mod_name].isMember("netnames")) {

                            for (auto it = root[mod_name]["netnames"].begin(); it != root[mod_name]["netnames"].end() ; it++) {
                                auto* net_ptr = &mod_ptr->nets.emplace_back(
                                    rtl::Net{.name = it.key().asString()}
                                    );
                                PNR_LOG2("RTLF", "creating net '{}'...", net_ptr->name);
                                if ((*it).isMember("bits")) {
                                    int bitnum = -1;
                                    for (auto it1 = (*it)["bits"].begin(); it1 != (*it)["bits"].end() ; it1++) {
                                        if ((*it)["bits"].size() > 1) {
                                           ++bitnum;
                                        }
                                        int designator = -1;
                                        if ((*it1).type() == Json::ValueType::stringValue) {
                                            designator = (*it1).asString() == "0" ? -1 : ((*it1).asString() == "1" ? -2 : ((*it1).asString() == "z" ? -3 : -4));
                                        }
                                        else {
                                            designator = (*it1).asInt();
                                        }
                                        PNR_LOG3("RTLF", " [{}]<{}>", bitnum, designator);
                                        net_ptr->designators.push_back(designator);
                                    }
                                }
                            }
                        }
                    }
                    catch (Json::Exception& ex) {
                        PNR_ERROR("Rtl::loadFromJson('{}') cant parse JSON at line {}, exception: '{}'", filename, line_number, ex.what());
                        return false;
                    }
                    mod_json = "{";
                }
            }
        }
        if (start_indent == 0 || mods_indent == 0) {
            return false;
        }
        return true;
    }
};
