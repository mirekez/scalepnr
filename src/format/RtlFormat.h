#include <print>
#include <fstream>
#include <iostream>
#include <vector>
#include "json/json.h"
#include "Rtl.h"

struct RtlFormat
{
    bool loadFromJson(const std::string& filename, Rtl* rtl)
    {
        PNR_LOG2("RTL ", "loadFromJson(), filename: '{}'", filename);
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
                        PNR_LOG2("RTL ", "module: '{0}', blackbox: {1}", name, root[name]["attributes"]["blackbox"].asString());
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

