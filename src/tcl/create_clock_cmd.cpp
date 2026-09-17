#include "Tech.h"
#include "tcl_pnr.h"

#include <cmath>
#include <set>

namespace {
int error(Tcl_Interp* interp, const std::string& message)
{
    Tcl_SetObjResult(interp, Tcl_NewStringObj(message.c_str(), -1));
    return TCL_ERROR;
}
}

int create_clock_cmd(ClientData, Tcl_Interp* interp, int objc, Tcl_Obj* const objv[])
{
    std::string name;
    double period = 0;
    bool named = false, periodic = false;
    Tcl_Obj* source = nullptr;
    for (int i = 1; i < objc; ++i) {
        std::string option = Tcl_GetString(objv[i]);
        if (option == "-name" || option == "-period") {
            if (++i == objc) return error(interp, "missing value for " + option);
            if (option == "-name") {
                if (named) return error(interp, "duplicate -name");
                named = true;
                name = Tcl_GetString(objv[i]);
                if (name.empty()) return error(interp, "clock name must not be empty");
            } else {
                if (periodic) return error(interp, "duplicate -period");
                periodic = true;
                if (Tcl_GetDoubleFromObj(interp, objv[i], &period) != TCL_OK) return TCL_ERROR;
                if (!std::isfinite(period) || period < 0.000001 || period > 1e9)
                    return error(interp, "clock period must be finite and between 0.000001 and 1e9 ns");
            }
        } else if (!option.empty() && option.front() == '-') {
            return error(interp, "unsupported create_clock option: " + option);
        } else {
            if (source) return error(interp, "create_clock requires exactly one source port");
            source = objv[i];
        }
    }
    if (!periodic || !source)
        return error(interp, "usage: create_clock [-name name] -period ns source_port");
    int count;
    Tcl_Obj** ports;
    if (Tcl_ListObjGetElements(interp, source, &count, &ports) != TCL_OK) return TCL_ERROR;
    if (count != 1) return error(interp, "create_clock requires exactly one source port");
    std::string port = Tcl_GetString(ports[0]);
    if (!named) name = port;
    auto& tech = technology::Tech::current();
    if (!tech.clocks.addClocks(tech.design, name, port, period, 50))
        return error(interp, "cannot create clock: duplicate clock name/source or unknown source port: " + port);
    try {
        tech.prepareTimingLists();
        tech.timings.calculateTimings();
    } catch (const std::exception& exception) {
        // The timing builder commits only a complete, unambiguous forest.
        tech.clocks.clocks_list.pop_back();
        return error(interp, exception.what());
    }
    Tcl_SetObjResult(interp, Tcl_NewListObj(0, nullptr));
    Tcl_ListObjAppendElement(interp, Tcl_GetObjResult(interp), Tcl_NewStringObj(name.c_str(), -1));
    return TCL_OK;
}

int get_clocks_cmd(ClientData, Tcl_Interp* interp, int objc, Tcl_Obj* const objv[])
{
    if (objc > 2) return error(interp, "usage: get_clocks [patterns]");
    int count = 0;
    Tcl_Obj** patterns = nullptr;
    if (objc == 2 && Tcl_ListObjGetElements(interp, objv[1], &count, &patterns) != TCL_OK)
        return TCL_ERROR;
    Tcl_SetObjResult(interp, Tcl_NewListObj(0, nullptr));
    for (auto& clock : technology::Tech::current().clocks.clocks_list) {
        bool match = objc == 1;
        for (int i = 0; i < count; ++i)
            match |= Tcl_StringMatch(clock.name.c_str(), Tcl_GetString(patterns[i])) != 0;
        if (match) Tcl_ListObjAppendElement(interp, Tcl_GetObjResult(interp),
            Tcl_NewStringObj(clock.name.c_str(), -1));
    }
    return TCL_OK;
}

int set_clock_groups_cmd(ClientData, Tcl_Interp* interp, int objc, Tcl_Obj* const objv[])
{
    auto& tech = technology::Tech::current();
    bool asynchronous = false;
    std::vector<std::vector<std::string>> groups;
    std::set<std::string> used;
    for (int i = 1; i < objc; ++i) {
        std::string option = Tcl_GetString(objv[i]);
        if (option == "-asynchronous" && !asynchronous) {
            asynchronous = true;
        } else if (option == "-group" && ++i < objc) {
            int count;
            Tcl_Obj** names;
            if (Tcl_ListObjGetElements(interp, objv[i], &count, &names) != TCL_OK) return TCL_ERROR;
            if (!count) return error(interp, "clock groups must not be empty");
            auto& group = groups.emplace_back();
            for (int j = 0; j < count; ++j) {
                std::string name = Tcl_GetString(names[j]);
                std::vector<rtl::Clock*> found;
                tech.clocks.getClocks(&found, name, false);
                if (found.empty()) return error(interp, "unknown clock: " + name);
                if (!used.insert(name).second) return error(interp, "clock repeated in groups: " + name);
                group.push_back(name);
            }
        } else return error(interp, "usage: set_clock_groups -asynchronous -group clocks -group clocks ...");
    }
    if (!asynchronous || groups.size() < 2)
        return error(interp, "at least two explicit asynchronous groups are required");
    // Validate everything before changing existing constraints.
    for (size_t i = 0; i < groups.size(); ++i)
        for (size_t j = i + 1; j < groups.size(); ++j)
            for (auto& a : groups[i]) for (auto& b : groups[j])
                tech.clocks.asynchronous_pairs.insert(std::minmax(a, b));
    tech.prepareTimingLists();
    tech.timings.calculateTimings();
    Tcl_ResetResult(interp);
    return TCL_OK;
}
