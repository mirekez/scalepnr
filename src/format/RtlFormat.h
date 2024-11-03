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
//                    std::print("\n<<<{0}>>>\n", mod_json);
                    try {
                        Json::Value root;
                        Json::Reader reader;
                        reader.parse(mod_json, root);
                        std::string name = root.getMemberNames()[0];
                        design->modules.emplace_back(rtl::Module{name});
                        PNR_LOG1("RTL ", "module '{0}': {1}", name, atoi(root[name]["attributes"]["blackbox"].asString().c_str()) != 0 ? "(blackbox)" : "... ");
                        if (root[name].isMember("ports")) {
                            for (auto it = root[name]["ports"].begin() ; it != root[name]["ports"].end() ; it++) {
                                PNR_LOG2("RTL ", "port '{}'({}): {}", it.key().asString(), (*it)["direction"].asString(), (*it)["bits"].asString());
                            }
                        }
                        if (!root[name]["attributes"].isMember("blackbox") || atoi(root[name]["attributes"]["blackbox"].asString().c_str()) == 0) {
                            if (root[name].isMember("cells")) {
                                for (auto it = root[name]["cells"].begin() ; it != root[name]["cells"].end() ; it++) {
                                    PNR_LOG2("RTL ", "cell '{}'({}): ", it.key().asString(), (*it)["type"].asString());
                                    if ((*it).isMember("port_directions")) {
                                        PNR_LOG3("RTL ", "{{");
                                        std::string delim = "";
                                        for (auto it1 = (*it)["port_directions"].begin() ; it1 != (*it)["port_directions"].end() ; it1++) {
                                            PNR_LOG3("RTL ", "{}{}{}", delim, (*it1).asString()=="input" ? '>' : ((*it1).asString()=="output" ? '<' : ':'), it1.key().asString());
                                            delim = ", ";
                                        }
                                        PNR_LOG3("RTL ", "}}");
                                    }
                                }
                            }
                        }
                    }
                    catch (Json::Exception& ex) {
                        PNR_ERROR("Rtl::loadFromJson({}) cant parse JSON at line {}, exception: '{}'", filename, line_number, ex.what());
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

