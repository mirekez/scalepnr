#include "Conn.h"
#include "Inst.h"

using namespace rtl;

std::string Conn::makeName(std::string* inst_name_hint)
{
    std::string inst_name;
    if (!inst_name_hint) {
        inst_name = inst_ref->makeName();
    }
    return (inst_name_hint?*inst_name_hint:inst_name).length() ?
        (inst_name_hint?*inst_name_hint:inst_name) + "." + port_ref->name :
        port_ref->name;
}
