#include "Crossbar.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
}

void shared_conn_names_regression()
{
    using namespace fpga;
    CBType base;
    base.rememberConnName(CB_NODE_DST, 9, CB_NODE_SRC, 7, "arrival_alpha", "exit_alpha");
    base.rememberConnName(CB_NODE_DST, 9, CB_NODE_SRC, 7, "arrival_beta", "exit_beta");
    const auto* original = base.connNames(CB_NODE_DST, 9, CB_NODE_SRC, 7);
    require(original && original->size() == 2, "missing connection aliases");
    const auto* original_table = &base.conn_names.read();

    // The real device had 1,400 subtype tables with 5.22 million entries,
    // all identical to their bases. Sharing strings alone still copied every
    // hash node/vector. Verify the connection vectors themselves are shared.
    std::vector<CBType> subtypes(1400, base);
    for (const auto& subtype : subtypes) {
        require(&subtype.conn_names.read() == original_table,
                "subtype copied hash storage instead of sharing the whole table");
        require(subtype.connNames(CB_NODE_DST, 9, CB_NODE_SRC, 7) == original,
                "BUG: subtype duplicated the connection-name table");
        require(subtype.connName(CB_NODE_LOCAL, 9, CB_NODE_SRC, 7) == nullptr,
                "connection lookup crossed node namespaces");
    }

    // Device construction can rediscover an already registered annotation.
    // This is a read, not a reason to duplicate a complete subtype table.
    subtypes[0].outgoing_srcs.clear();
    subtypes[0].rememberConnName(CB_NODE_DST, 9, CB_NODE_SRC, 7, "arrival_alpha", "exit_alpha");
    require(subtypes[0].connNames(CB_NODE_DST, 9, CB_NODE_SRC, 7) == original,
            "duplicate registration detached shared annotations");
    require(&subtypes[0].conn_names.read() == original_table,
            "duplicate registration copied shared hash storage");
    const auto* srcs = subtypes[0].srcNodes(CB_NODE_DST, 9);
    require(srcs && srcs->size() == 1 && srcs->front() == 7,
            "duplicate registration failed to restore outgoing SRC index");

    subtypes[0].conn_names[{CB_NODE_DST, 9, CB_NODE_SRC, 7}][0].to = "private_exit";
    require(subtypes[0].connNames(CB_NODE_DST, 9, CB_NODE_SRC, 7) != original,
            "subtype edit did not detach");
    require(subtypes[0].connName(CB_NODE_DST, 9, CB_NODE_SRC, 7)->to.str() == "private_exit",
            "subtype edit was not applied");
    require(base.connName(CB_NODE_DST, 9, CB_NODE_SRC, 7)->to.str() == "exit_alpha" &&
            subtypes[1].connNames(CB_NODE_DST, 9, CB_NODE_SRC, 7) == original,
            "subtype edit changed base or sibling annotations");

    base.rememberConnName(CB_NODE_DST, 9, CB_NODE_SRC, 7, "arrival_gamma", "exit_gamma");
    require(base.connNames(CB_NODE_DST, 9, CB_NODE_SRC, 7)->size() == 3 && original->size() == 2,
            "base edit changed an existing subtype");
    subtypes[2].rememberConnName(CB_NODE_LOCAL, 4, CB_NODE_JOINT, 2, "local_delta", "joint_delta");
    require(subtypes[2].connName(CB_NODE_LOCAL, 4, CB_NODE_JOINT, 2) &&
            !subtypes[1].connName(CB_NODE_LOCAL, 4, CB_NODE_JOINT, 2),
            "new subtype connection leaked into sibling table");

    CBType assigned;
    assigned = subtypes[1];
    require(assigned.connNames(CB_NODE_DST, 9, CB_NODE_SRC, 7) == original,
            "copy assignment duplicated annotations");
    CBType moved = std::move(assigned);
    require(moved.connNames(CB_NODE_DST, 9, CB_NODE_SRC, 7) == original,
            "move lost shared annotations");
    require(!assigned.connName(CB_NODE_DST, 9, CB_NODE_SRC, 7),
            "moved-from annotation table is not readable as empty");
    assigned.rememberConnName(CB_NODE_LOCAL, 4, CB_NODE_JOINT, 2, "local_reused", "joint_reused");
    require(assigned.connName(CB_NODE_LOCAL, 4, CB_NODE_JOINT, 2), "cannot reuse moved-from table");
    moved.conn_names.clear();
    require(!moved.connName(CB_NODE_DST, 9, CB_NODE_SRC, 7) && original->size() == 2,
            "clearing one shared table changed its siblings");

    base = CBType{};
    subtypes.reserve(2800);
    require(subtypes[1].connNames(CB_NODE_DST, 9, CB_NODE_SRC, 7) == original &&
            original->front().to.str() == "exit_alpha", "shared annotations did not survive base destruction/relocation");

    // Reloading a type must discard only its annotations, not the old copies.
    CBTypeSpec empty;
    TechMap map;
    subtypes[2].loadFromSpec(empty, map);
    require(!subtypes[2].connName(CB_NODE_DST, 9, CB_NODE_SRC, 7) && original->size() == 2,
            "reload invalidated another subtype's connection names");
}
