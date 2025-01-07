#include "PrintDesign.h"

#include <ranges>

using namespace rtl;

void PrintDesign::print(Inst* inst)
{
    for (auto& conn : inst->conns) {
        print(&conn);
    }
}

void PrintDesign::printIndent(Conn* out, bool merge)
{
    std::print("\n");
    bool connect = false;
    size_t connect_i = nets_stack.size();
    for (size_t i=0; i < nets_stack.size(); ++i) {  // check if we need to connect this line to output
        if (out == nets_stack[i].conn && !nets_stack[i].is_free()) {
            connect = true;
            connect_i = i;
            break;
        }
    }
    bool draw_connection = false;
    for (size_t i=0; i < nets_stack.size(); ++i) {
        size_t j;
        while (nets_stack[i].is_free() && i < connect_i) {  // we cant do collapse after connecting line
            j = collapseIndent(i, out, connect, connect_i);
            if (j == i) {
                break;
            }
            i = j;
        }
        if (i >= nets_stack.size()) {
            break;
        }
        if (connect) {  // has corresponding output on this line
            if (nets_stack[i].conn == out && !nets_stack[i].is_free()) {  // connect it
                if (merge && draw_connection) {  // merge is a special case
                    nets_stack[i].ports_to_connect = nets_stack[connect_i].ports_to_connect;
                    nets_stack[i].ports_connected += nets_stack[connect_i].ports_connected;
                    nets_stack[connect_i].merge_to_index = -1;
                    nets_stack[connect_i].ports_connected = nets_stack[connect_i].ports_to_connect;
                    connect_i = i;  // we jump to next column that need merge
                }
                if (!merge) {  // merge is not a connection
                    ++nets_stack[i].ports_connected;
                }

                if (nets_stack[i].is_free()) {
                    if (!draw_connection) {
                        std::print("\xE2\x94\x94");  // └
                        draw_connection = true;
                    }
                    else
                    if (merge && nets_stack[i].merge_to_index == -1) {  // this is the last column to merge (no merge_to mark)
                        draw_connection = false;
                        std::print("\xE2\x94\x98");  // ┘
                    }
                    else {
                        std::print("\xE2\x94\xb4");  // ┴
                    }
                }
                else {
                    if (merge && !draw_connection) {  // during merge we end this line but it's not free yet
                        std::print("\xE2\x94\x94");  // └
                        draw_connection = true;
                    }
                    else
                    if (!draw_connection) {
                        std::print("\xE2\x94\x9C");  // ├
                        draw_connection = true;
                    }
                    else
                    if (merge && nets_stack[i].merge_to_index == -1) {  // this is the last column to merge (no merge_to mark)
                        draw_connection = false;
                        std::print("\xE2\x94\xA4");  // ┤
                    }
                    else {
                        std::print("\xE2\x97\x8A");  // ◯AF  ◊
                    }
                }

            }
            else {  // horizontal line
                if (nets_stack[i].is_free()) {  // current column hasnt more signal?
                    if (!draw_connection) {
                        std::print(" ");
                    }
                    else {
                        std::print("\xE2\x94\x80");  // ─
                    }
                }
                else {
                    if (!draw_connection) {
                        if (nets_stack[i].style == 0) {
                            std::print("\xE2\x94\x82");  // |
                        }
                        else
                        if (nets_stack[i].style == 1) {
                            std::print("|");  // |
                        }
                    }
                    else {
                        std::print("\xE2\x94\xBC");  // ┼
                    }
                }
            }
        }
        else {  // no correponding output
            if (nets_stack[i].is_free()) {
                std::print(" ");
            }
            else {
                if (nets_stack[i].style == 0) {
                    std::print("\xE2\x94\x82");  // |
                }
                else
                if (nets_stack[i].style == 1) {
                    std::print("|");  // |
                }
            }
        }
    }
}

size_t PrintDesign::collapseIndent(size_t from, Conn* out, bool connect, size_t& connect_i)
{
    if (nets_stack.size() < from+1) {
        return from;
    }
    size_t i = from;
    size_t j;
    bool no_more_nets = true;
    int cnt_free = 0;
    for (j=i+1; j < nets_stack.size(); ++j) {  // find where to connect
        if (!nets_stack[j].is_free()) {
            no_more_nets = false;
            break;
        }
        ++cnt_free;
    }
    if (j == nets_stack.size() || (j == nets_stack.size()-1 && nets_stack[j].almost_done() && j == connect_i)) {  // if we didnt find busy column or if it's last column and can be freed
        while (nets_stack.size() > from + 1) {  // never pop 'from' column
            nets_stack.pop_back();
        }
        return from;
    }
    if (j == connect_i && j != nets_stack.size()-1) {
        return from;
    }
    std::print("\xE2\x94\x8c");  //  ┌

    // changing after was checked
    nets_stack[i] = nets_stack[j];
    nets_stack[j].ports_to_connect = -1;
    nets_stack[j].ports_connected = 0;
    nets_stack[j].merge_to_index = -1;
    if (connect_i == j) {
        connect_i = i;
    }

    if (no_more_nets) {
        if (j - i > 1) {
            for (int l = 0; l < cnt_free; ++l) {
                nets_stack.pop_back();
            }
        }
        return nets_stack.size();
    }
    for (size_t k=i+1; k < j; ++k) {
        std::print("\xE2\x94\x80");  // ─
        --cnt_free;
    }
    if (connect_i <= j) {  // connecting this
        if (nets_stack[i].ports_connected > 0) {  // has connection from top
            std::print("\xE2\x94\xb4");  // ┴
        }
        else {
            std::print("\xE2\x94\x80");  // ─
        }
        ++nets_stack[i].ports_connected;
    }
    else {  // not found connection during collapse
        if (nets_stack[i].ports_connected > 0) {  // has connection from top
            std::print("\xE2\x94\x98");  // ┘
        }
        else {
            std::print("?");  // ┘
        }
    }

    for (int l = 0; l < cnt_free; ++l) {
        std::print(" ");
    }
    return j + 1;
}

void PrintDesign::print(Conn* out, int depth, bool do_recurse)
{
    bool root = false;
    if (limit <= 1) {
        if (limit == 1) {
            std::print("\nlimit achieved");
            --limit;
        }
        return;
    }
    if (depth == -1) {  // initial
        nets_stack.clear();
        nets_stack.reserve(2048);
        already_shown.clear();
        visible.clear();

        for (auto& conn : out->inst_ref->conns) {  // I hope it's OBUF with one output
            if (conn.port_ref->type == rtl::Port::PORT_OUT) {
                out = &conn;
                break;
            }
        }

        // markup (we need to know which insts will be shown)
        markup_pass = true;
        print(out, 0);
        markup_pass = false;

        root = true;
        nets_stack.clear();
        already_shown.clear();

    }
    Inst* inst = out->inst_ref.peer;

    if (!markup_pass) {

        printIndent(out);

        size_t was_indent = nets_stack.size();
        size_t new_indent = nets_stack.size();
        for (auto& conn : std::ranges::views::reverse(inst->conns)) {  // counting inputs and making new indent
            rtl::Conn* curr = &conn;
            if (curr->port_ref->type == rtl::Port::PORT_IN) {
                auto it = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
                while (it != tech->clocked_ports.end()) {
                    if (it->second == curr->port_ref->name) {  // clock port // TODO: add support for 2-clock primitives
                        break;
                    }
                    ++it;
                }
                if (it != tech->clocked_ports.end()) {  // excluding clock ports
                    continue;
                }

                curr = curr->follow();
                if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                    continue;
                }

                if (Conn::fromRef(*static_cast<Ref<Conn>*>(curr)).peers.size() > 30) {
                    continue;
                }

                if (do_recurse) {
                    for (size_t i=0; i < nets_stack.size(); ++i) {  // if found same input conn in old rows
                        if (nets_stack[i].conn == curr && !nets_stack[i].is_free() && nets_stack[i].merge_to_index == -1) {
                            nets_stack[i].merge_to_index = nets_stack.size();  // this will be place of new net
                            break;
                        }
                    }
                    int ports_to_connect = 1;
                    for (auto* conn1 : Conn::fromRef(*static_cast<Ref<Conn>*>(curr)).peers) {  // all insts who connected to same output
                        Conn* curr1 = &Conn::fromRef(*static_cast<Ref<Conn>*>(conn1));
                        if (visible.find(curr1->inst_ref.peer) != visible.end()) {  // only who is visible
                            ++ports_to_connect;
                        }
                    }
                    nets_stack.push_back(NetCtx{.conn=curr, .ports_to_connect = ports_to_connect, .ports_connected = 1, .style = (style_cntr++)%2});
                }
                ++new_indent;
            }
        }

        int fanout = Conn::fromRef(*static_cast<Ref<Conn>*>(out)).peers.size();
        int fanout_visible = 0;
        for (auto* conn1 : Conn::fromRef(*static_cast<Ref<Conn>*>(out)).peers) {  // all insts who connected to same output
            Conn* curr1 = &Conn::fromRef(*static_cast<Ref<Conn>*>(conn1));
            if (visible.find(curr1->inst_ref.peer) != visible.end()) {  // only who is visible
                ++fanout_visible;
            }
        }

        auto it1 = tech->clocked_ports.find(inst->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
        auto it2 = tech->buffers_ports.find(inst->cell_ref->type);
        if (it1 != tech->clocked_ports.end() || it2 != tech->buffers_ports.end()) {
            for (size_t i = was_indent; i < new_indent; ++i) {
                std::print("\xE2\x96\xAF");  // ▯
            }
            std::print(" {} ({}) (fanout: {}/{})", out->makeName(), inst->cell_ref->type, fanout_visible, fanout);
        }
        else {
            for (size_t i = was_indent; i < new_indent; ++i) {
                std::print("\xE2\x97\x81");  // ◁
            }
            std::print(" {} ({}) (fanout: {}/{})", out->makeName(), inst->cell_ref->type, fanout_visible, fanout);
        }
        --limit;

        if (do_recurse && debug) {  // only for one line per inst
            std::print(", inst sources: ");
            for (auto& conn : std::ranges::views::reverse(inst->conns)) {  // debug
                if (conn.port_ref->type == rtl::Port::PORT_IN) {
                    Conn* curr = &conn;
                    auto it = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
                    while (it != tech->clocked_ports.end()) {
                        if (it->second == curr->port_ref->name) {  // clock port // TODO: add support for 2-clock primitives
                            break;
                        }
                        ++it;
                    }
                    if (it != tech->clocked_ports.end()) {  // excluding clock ports
                        continue;
                    }
                    curr = curr->follow();
                    if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                        continue;
                    }
                    std::print(" {}({})seen={}", curr->makeName(), curr->inst_ref->cell_ref->type, visible.find(curr->inst_ref.peer) != visible.end());
                }
            }
        }

        if (debug) {
            std::print(", conn sinks: ");
            for (auto* conn1 : Conn::fromRef(*static_cast<Ref<Conn>*>(out)).peers) {  // all insts who connected to same output
                Conn* curr1 = &Conn::fromRef(*static_cast<Ref<Conn>*>(conn1));
//                if (visible.find(curr1->inst_ref.peer) != visible.end()) {  // only who is visible
                    std::print(" {}({})seen={}", curr1->makeName(), curr1->inst_ref->cell_ref->type, visible.find(curr1->inst_ref.peer) != visible.end());
//                }
            }
        }

        for (size_t i=0; i < nets_stack.size(); ++i) {
            if (nets_stack[i].merge_to_index != -1) {
                printIndent(nets_stack[i].conn, true);
            }
        }
    }

    if (do_recurse)
    for (auto& conn : inst->conns) {  // doing cycle for all inputs of this inst
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            auto it = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            while (it != tech->clocked_ports.end()) {
                if (it->second == curr->port_ref->name) {  // clock port // TODO: add support for 2-clock primitives
                    break;
                }
                ++it;
            }
            if (it != tech->clocked_ports.end()) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }
            if (Conn::fromRef(*static_cast<Ref<Conn>*>(curr)).peers.size() > 30) {
                continue;
            }

            auto it1 = already_shown.find(curr->inst_ref.peer);
            if (it1 == already_shown.end()) {
                already_shown.insert(curr->inst_ref.peer);
            }
            else {
                continue;
            }

            if (markup_pass) {
                visible.insert(curr->inst_ref.peer);
            }

            int cnt_outputs = curr->inst_ref->cnt_outputs;
            for (auto& conn1 : curr->inst_ref->conns) {  // doing cycle for all outputs of it's peer

                if (conn1.port_ref->type == rtl::Port::PORT_OUT) {
                    --cnt_outputs;

                    if ((int)Conn::fromRef(*static_cast<Ref<Conn>*>(&conn1)).peers.size() > 0 || cnt_outputs == 0) {
//                        size_t free = nets_stack.size();
//                        size_t i;
//                        for (i=0; i < nets_stack.size(); ++i) {
//                            if (nets_stack[i].is_free()) {
//                                free = i;
//                            }
//                            if (nets_stack[i].conn == &conn1) {
//                                break;
//                            }
//                        }
//                        if (i == nets_stack.size()) {  // if not present yet
//                            if (free != nets_stack.size()) {
//                                nets_stack[free].conn = &conn1;
//                                nets_stack[free].ports_to_connect = 2/*(int)Conn::fromRef(*static_cast<Ref<Conn>*>(&conn1)).peers.size() + 1*/;
//                                nets_stack[free].ports_connected = 0;
//                            }
//                            else {
//                                nets_stack.push_back(NetCtx{.conn=&conn1, .ports_to_connect=2/*(int)Conn::fromRef(*static_cast<Ref<Conn>*>(&conn1)).peers.size() + 1*/});
//                            }
//                        }
                        print(&conn1, depth + 1, cnt_outputs == 0);
                    }
                }
            }
        }
    }
    if (root) {
        if (debug) {
            if (nets_stack.size()) {
                std::print("\nRest nets:");
            }
            for (auto& net : nets_stack) {
                if (!net.is_free()) {
                    std::print("\n{} ({}), to_connect: {}, connected: {}, merge: {}, peers: ", net.conn->makeName(), net.conn->inst_ref->cell_ref->type, net.ports_to_connect, net.ports_connected, net.merge_to_index);
                    for (auto* peer_ptr: Conn::fromRef(*static_cast<Ref<Conn>*>(net.conn)).peers) {
                        Referable<rtl::Conn>& peer = rtl::Conn::fromRef(*static_cast<Ref<rtl::Conn>*>(peer_ptr));
                        std::print(" {} ({})", peer.makeName(), peer.inst_ref->cell_ref->type);
                    }
                }
            }
        }

        nets_stack.clear();
        already_shown.clear();
        visible.clear();
    }
}
