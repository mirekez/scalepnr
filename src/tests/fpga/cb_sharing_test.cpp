#include "Crossbar.h"

#include <iostream>
#include <random>
#include <stdexcept>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

// Construct independently allocated but equal tables in different insertion orders.
fpga::CBType makeType(bool reverse, unsigned seed)
{
    fpga::CBType type;
    for (int i = 0; i < 16; ++i) {
        int node = reverse ? 15 - i : i;
        auto bit = NodeMask{0, 1} << (node * 239);
        type.local_src[node].jump = bit;
        type.local_joint[node].joint = bit;
        type.local_local[node].local = bit;
        type.src_joint[node].joint = bit;
        type.joint_src[node].jump = bit;
        type.joint_local[node].local = bit;
        type.joint_joint[node].joint = bit;
        type.dst_src[node].jump = bit;
        type.dst_local[node].local = bit;
        type.dst_joint[node].joint = bit;
        type.rememberNodeName(fpga::CB_NODE_SRC, node, "ABC_SOURCE_" + std::to_string(node));
        type.rememberNodeName(fpga::CB_NODE_DST, node, "XYZ_ARRIVAL_" + std::to_string(node));
        type.rememberConnName(fpga::CB_NODE_DST, node, fpga::CB_NODE_SRC, node,
                             "XYZ_ARRIVAL_" + std::to_string(node), "ABC_SOURCE_" + std::to_string(node));
    }
    fpga::CBType::ResolvedJump jump;
    jump.delta = {1, static_cast<int>(seed % 3)};
    jump.target_cb_type_id = static_cast<uint16_t>(seed);
    jump.dsts.jump = NodeMask{0, 1} << 3000;
    jump.dst_wires[3000] = "XYZ_REMOTE_ARRIVAL";
    type.dst_by_src[0].push_back(jump);
    fpga::CBType::ResolvedLocal local;
    local.delta = {0, 1};
    local.target_cb_type_id = 42;
    local.target_nodes = NodeMask{0, 1} << 3001;
    type.local_by_local[3].push_back(local);
    type.rebuildOutgoingSrcs();
    return type;
}

void checkSubtypeSharing()
{
    std::mt19937 random(3819);
    std::vector<fpga::CBType> types;
    {
        fpga::CBType::SharedDataPool pool;
        for (unsigned i = 0; i < 48; ++i) {
            types.push_back(makeType(random() & 1, i));
            types.back().shareCommonData(pool);
        }
        const auto& base = types.front();
        for (const auto& type : types) {
            // Distinct subtypes must reuse every equal base and derived mask container.
            require(type.local_src.values.sharesStorageWith(base.local_src.values), "local SRC tables were copied");
            require(type.dst_src.values.sharesStorageWith(base.dst_src.values), "DST SRC tables were copied");
            require(type.local_joint.values.sharesStorageWith(base.local_joint.values), "joint tables were copied");
            require(type.dst_local.values.sharesStorageWith(base.dst_local.values), "local tables were copied");
            require(type.dsts_reaching_local.values.sharesStorageWith(base.dsts_reaching_local.values), "derived tables were copied");
            require(type.conn_names.sharesStorageWith(base.conn_names), "annotation containers were copied");
            require(type.node_names.sharesStorageWith(base.node_names), "name containers were copied");
            require(type.src_nodes_by_name.sharesStorageWith(base.src_nodes_by_name), "reverse name containers were copied");
            require(type.local_by_local.values.sharesStorageWith(base.local_by_local.values), "local transition containers were copied");
        }
        // Different target mappings cannot merge even though their other tables are shared.
        require(!types[0].dst_by_src.values.sharesStorageWith(types[1].dst_by_src.values), "different landings merged");
        auto equal = makeType(true, 0);
        equal.shareCommonData(pool);
        require(equal.dst_by_src.values.sharesStorageWith(base.dst_by_src.values), "equal landings did not share");
        require(equal.src_priority_deltas.sharesStorageWith(base.src_priority_deltas), "equal numeric priorities did not share");
    }
    // Destroying the temporary hash index cannot destroy any published table.
    const auto& original = types[0];
    require(original.dst_src[15].jump != NodeMask{}, "pool destruction lost routing data");
    auto changed = types[1];
    require(changed.dst_src.values.sharesStorageWith(original.dst_src.values), "subtype copying deep-copied masks");
    changed.dst_src[15].jump = {};
    require(original.dst_src[15].jump != NodeMask{}, "mask override corrupted another subtype");
    require(!changed.dst_src.values.sharesStorageWith(original.dst_src.values), "mask mutation failed to detach");
    require(changed.dst_local.values.sharesStorageWith(original.dst_local.values), "mask override detached unrelated tables");
    changed.conn_names[{fpga::CB_NODE_DST, 0, fpga::CB_NODE_SRC, 0}][0].to = "OVERRIDE";
    require(original.connName(fpga::CB_NODE_DST, 0, fpga::CB_NODE_SRC, 0)->to != std::string("OVERRIDE"), "annotation override corrupted another subtype");
    changed.dst_by_src[0][0].delta.x = 7;
    require(std::as_const(types[1]).dst_by_src[0][0].delta.x == 1, "landing override corrupted another subtype");
    changed.local_by_local[3][0].target_nodes = {};
    require(original.local_by_local[3][0].target_nodes != NodeMask{}, "local transition override corrupted another subtype");
    changed.node_names.clear();
    require(original.nodeName(fpga::CB_NODE_SRC, 0) != nullptr, "clearing names corrupted another subtype");
}

void checkCollisionsAndWrites()
{
    using Map = SharedMap<std::unordered_map<int, int>>;
    Map::Pool pool;
    auto collision = [](const auto&) { return size_t{17}; };
    Map left, right, equal;
    left[1] = 11;
    right[1] = 12;
    equal[1] = 11;
    pool.share(left, collision);
    pool.share(right, collision);
    pool.share(equal, collision);
    // A hash collision must never equate different contents.
    require(!left.sharesStorageWith(right), "hash collision merged different maps");
    require(left.sharesStorageWith(equal), "equal maps did not merge in a collision bucket");
    auto copy = left;
    copy.write().find(1)->second = 13;
    require(left.at(1) == 11, "explicit mutable access modified another owner");
    copy = left;
    copy.erase(1);
    require(left.contains(1), "erase modified another owner");
    copy = left;
    copy.try_emplace(2, 22);
    require(!left.contains(2), "insertion modified another owner");
    copy = left;
    copy.insert_or_assign(1, 14);
    pool.share(copy, collision);
    require(!copy.sharesStorageWith(left), "mutation reused a stale content hash");
    copy = left;
    copy.clear();
    require(left.size() == 1 && copy.empty(), "clear modified another owner");
    Map absent;
    require(absent.find(8) == absent.end() && absent.empty(), "read allocated an entry");
    // Expired candidates are weak references and must not be reused as stale data.
    { Map temporary; temporary[5] = 6; pool.share(temporary, collision); }
    absent[5] = 7;
    pool.share(absent, collision);
    require(absent.at(5) == 7, "expired candidate corrupted an entry");

    Map::Pool content_pool;
    auto content_hash = [](const auto& values) { return static_cast<size_t>(values.at(1)); };
    Map changing;
    changing[1] = 1;
    content_pool.share(changing, content_hash);
    // A unique owner must invalidate its cached hash too, not just detached copies.
    changing[1] = 2;
    content_pool.share(changing, content_hash);
    Map expected;
    expected[1] = 2;
    content_pool.share(expected, content_hash);
    require(changing.sharesStorageWith(expected), "unique-owner mutation retained a stale hash");
}

void checkRebuildAndFullMaskEquality()
{
    fpga::CBType::SharedDataPool pool;
    auto base = makeType(false, 0);
    base.shareCommonData(pool);
    auto copy = base;
    // Derived-cache rebuilding must be isolated even when every input table is shared.
    copy.rebuildOutgoingSrcs();
    copy.shareCommonData(pool);
    require(copy.dsts_reaching_src.values.sharesStorageWith(base.dsts_reaching_src.values), "equal rebuilt masks did not share");
    copy.dst_src[0].jump |= NodeMask{0, 1} << 4095;
    copy.shareCommonData(pool);
    require(!copy.dst_src.values.sharesStorageWith(base.dst_src.values), "highest mask bit was ignored");
    copy = base;
    copy.dst_by_src[0][0].target_tile_coord = true;
    copy.shareCommonData(pool);
    require(!copy.dst_by_src.values.sharesStorageWith(base.dst_by_src.values), "landing coordinate mode was ignored");
    copy = base;
    copy.local_by_local[3][0].target_node_type = fpga::CB_NODE_DST;
    copy.shareCommonData(pool);
    require(!copy.local_by_local.values.sharesStorageWith(base.local_by_local.values), "local target role was ignored");
}

void checkRoutingReadsStayShared()
{
    fpga::CBType::SharedDataPool pool;
    auto base = makeType(false, 0);
    base.shareCommonData(pool);
    auto copy = base;
    for (int node = 0; node < 32; ++node) {
        int joint = -1;
        copy.canOut(node, 0, 0, joint);
        copy.canJump(node, 0, 0, joint);
        copy.canIn(node, 0, joint);
        copy.canInAvoidingJoint(node, 0);
        copy.terminalEntries(node);
    }
    fpga::CBState state{};
    state.type = &copy;
    state.iterate(true, 0, {0, 0}, {1, 0}, -1);
    state.hasFreeOut(0);
    // Routing queries must neither detach tables nor insert missing empty rows.
    require(copy.local_src.values.sharesStorageWith(base.local_src.values), "takeoff read detached its table");
    require(copy.dst_src.values.sharesStorageWith(base.dst_src.values), "transit read detached its table");
    require(copy.dst_local.values.sharesStorageWith(base.dst_local.values), "grounding read detached its table");
    require(copy.dst_joint.values.sharesStorageWith(base.dst_joint.values), "joint read detached its table");
    require(copy.dsts_reaching_local.values.sharesStorageWith(base.dsts_reaching_local.values), "terminal read detached its table");
    require(copy.local_src.values.size() == 16 && copy.dst_src.values.size() == 16, "routing inserted missing mask rows");
}

}

int main()
{
    try {
        checkSubtypeSharing();
        checkCollisionsAndWrites();
        checkRebuildAndFullMaskEquality();
        checkRoutingReadsStayShared();
        std::cout << "Crossbar common-data sharing regressions passed\n";
    }
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
