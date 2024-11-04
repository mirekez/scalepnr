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
        PNR_LOG("RTL ", "loadFromJson(), filename: '{}'", filename);
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
                        auto* mod_ref = &design->modules.emplace_back(rtl::Module{mod_name});
                        PNR_LOG1("RTL ", "module '{0}': {1}", mod_name, atoi(root[mod_name]["attributes"]["blackbox"].asString().c_str()) != 0 ? "(blackbox)" : "... ");

                        if (root[mod_name].isMember("ports")) {

                            mod_ref->ports.reserve(root[mod_name]["ports"].size());
                            for (auto it = root[mod_name]["ports"].begin(); it != root[mod_name]["ports"].end() ; it++) {
                                auto* port_ref = &mod_ref->ports.emplace_back(rtl::Port{it.key().asString(), (*it).isMember("bits") ? (int)(*it)["bits"].size() : -1,
                                    (*it)["direction"].asString() == "input" ? rtl::Port::PORT_IN :
                                        ((*it)["direction"].asString() == "output" ? rtl::Port::PORT_OUT : rtl::Port::PORT_IO )});
                                PNR_LOG2("RTL ", "port '{}'({}): ", it.key().asString(), (*it)["direction"].asString());
                                if ((*it).isMember("bits")) {
                                    port_ref->designators.reserve((*it)["bits"].size());
                                    std::string delim = "";
                                    for (auto it1 = (*it)["bits"].begin(); it1 != (*it)["bits"].end() ; it1++) {
                                        port_ref->designators.emplace_back((*it1).asInt());
                                        PNR_LOG3("RTL ", "{}{}", delim, (*it1).asInt());
                                        delim = ", ";
                                    }
                                }
                            }
                        }

                        if ((!root[mod_name]["attributes"].isMember("blackbox") || atoi(root[mod_name]["attributes"]["blackbox"].asString().c_str()) == 0)
                            && root[mod_name].isMember("cells")) {

                            for (auto it = root[mod_name]["cells"].begin(); it != root[mod_name]["cells"].end() ; it++) {
                                auto* cell_ref = &design->cells.emplace_back(rtl::Cell{it.key().asString(), (*it).isMember("type") ? (*it)["type"].asString() : std::string()});
                                PNR_LOG2("RTL ", "cell '{}'({}): ", it.key().asString(), (*it)["type"].asString());
                                if ((*it).isMember("port_directions")) {
                                    PNR_LOG3("RTL ", "{{");
                                    std::string delim = "";
                                    int i = -1;
                                    cell_ref->conns.reserve((*it)["connections"].size());
                                    for (auto it1 = (*it)["connections"].begin(); it1 != (*it)["connections"].end() ; it1++) {
                                        std::string dir = "inout";
                                        if ((*it).isMember("port_directions")) {
                                            std::string dir_name = (*it)["port_directions"].getMemberNames()[++i];
                                            if (it1.key().asString() == dir_name) {
                                                dir = (*it)["port_directions"][dir_name].asString();
                                            }
                                            else {
                                                PNR_WARNING("module '{}' cell '{}' connections and ports names mismatch: '{}' and '{}'", mod_name, it.key().asString(), it1.key().asString(), dir_name);
                                            }
                                        }
                                        auto* conn_ref = &cell_ref->conns.emplace_back(rtl::Conn(it1.key().asString(),
                                            dir == "input" ? rtl::Conn::CONN_IN : (dir == "output" ? rtl::Conn::CONN_OUT : rtl::Conn::CONN_IO)));
                                        PNR_LOG3("RTL ", "{}{}{}", delim, dir == "input" ? '\\' : (dir == "output" ? '/' : '|'), it1.key().asString());
                                        if ((*it1).type() == Json::ValueType::stringValue) {
                                            PNR_LOG3("RTL ", "='{}'", (*it1).asString());
                                        }
                                        else {
                                            PNR_LOG3("RTL ", "={}", (*it1).asInt());
                                        }
                                        delim = ", ";
                                    }
                                    PNR_LOG3("RTL ", "}}");
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

