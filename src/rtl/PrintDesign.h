#pragma once

#include "Design.h"
#include "Tech.h"

#include <map>
#include <set>
#include <unordered_set>
#include <unordered_map>

namespace rtl
{

struct PrintDesign
{
    tech::Tech* tech = nullptr;

    struct NetCtx
    {
        Conn* conn = nullptr;
        int ports_to_connect = -1;
        int ports_connected = 0;
        int merge_to_index = -1;
        int style = 0;

        bool is_free()
        {
            return ports_connected >= ports_to_connect;
        }

        bool almost_done()
        {
            return ports_connected == ports_to_connect-1;
        }
    };
    std::unordered_set<Inst*> already_shown;
    std::vector<NetCtx> nets_stack;
    bool markup_pass = false;
    std::unordered_set<Inst*> visible;
    bool debug = false;
    int style_cntr = 0;
    int limit = 100000;

    void print(Conn* out = 0, int depth = -1, bool do_recurse = true);
    void print(Inst* inst);
    void printIndent(Conn* curr, bool merge = false);
    size_t collapseIndent(size_t from, Conn* out, bool connect, size_t& connect_i);
};


}
