#include "PnrDb.h"

#include "json/json.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct Failure : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw Failure(message);
    }
}

std::string randomToken(std::mt19937& rng, const std::string& prefix)
{
    static constexpr char alphabet[] = "abcdefghijklmnopqrstuvwxyz";
    std::uniform_int_distribution<int> pick(0, 25);
    std::string out = prefix;
    for (int i = 0; i < 10; ++i) {
        out.push_back(alphabet[pick(rng)]);
    }
    return out;
}

Json::Value placedInst(const std::string& name, const std::string& type, int x, int y, int pos)
{
    Json::Value inst(Json::objectValue);
    inst["name"] = name;
    inst["type"] = type;
    inst["placed"] = true;
    inst["pos"] = pos;
    Json::Value coord(Json::arrayValue);
    coord.append(x);
    coord.append(y);
    inst["coord"] = coord;
    return inst;
}

Json::Value wirePayload(const std::string& label)
{
    Json::Value wire(Json::objectValue);
    wire["label"] = label;
    wire["type"] = "crossbar";
    return wire;
}

Json::Value routeEdgePayload(const std::string& label, int from_type, int from_node,
                             int to_type, int to_node)
{
    Json::Value wire = wirePayload(label);
    wire["type"] = "route_edge";
    wire["from_node_type"] = from_type;
    wire["from_node"] = from_node;
    wire["to_node_type"] = to_type;
    wire["to_node"] = to_node;
    return wire;
}

db::PnrDbRouteNode routeNode(uint32_t id, int x, int y, const std::string& kind, int node, const std::string& name)
{
    return db::PnrDbRouteNode{
        id,
        db::PnrDbCoord{x, y},
        kind,
        node,
        name,
    };
}

db::PnrDbRouteEdge routeEdge(uint32_t from, uint32_t to, const std::string& label)
{
    return db::PnrDbRouteEdge{from, to, wirePayload(label)};
}

db::PnrDbRouteTree singleSinkTree(std::mt19937& rng)
{
    db::PnrDbRouteTree tree;
    tree.net = randomToken(rng, "net_");
    tree.source = {"top.unit0.src", "out"};
    tree.sinks.push_back({"top.unit1.dst", "in"});
    tree.nodes.push_back(routeNode(0, 1, 1, "local", 3, randomToken(rng, "node_")));
    tree.nodes.push_back(routeNode(1, 1, 1, "source", 7, randomToken(rng, "node_")));
    tree.nodes.push_back(routeNode(2, 2, 1, "target", 11, randomToken(rng, "node_")));
    tree.edges.push_back(routeEdge(0, 1, randomToken(rng, "edge_")));
    tree.edges.push_back(routeEdge(1, 2, randomToken(rng, "edge_")));
    return tree;
}

db::PnrDbRouteTree forkedTree(std::mt19937& rng)
{
    db::PnrDbRouteTree tree;
    tree.id = randomToken(rng, "physical_net_");
    tree.net = randomToken(rng, "net_");
    tree.aliases = {tree.net, randomToken(rng, "alias_")};
    tree.source = {"top.block.producer", "q", 0};
    tree.sinks.push_back({"top.block.consumer0", "a", 3});
    tree.sinks.push_back({"top.block.consumer1", "b", 4});
    tree.nodes.push_back(routeNode(0, 4, 2, "local", 19, randomToken(rng, "node_")));
    tree.nodes.push_back(routeNode(1, 4, 2, "source", 23, randomToken(rng, "node_")));
    tree.nodes.push_back(routeNode(2, 5, 2, "transit", 29, randomToken(rng, "node_")));
    tree.nodes.push_back(routeNode(3, 6, 1, "target", 31, randomToken(rng, "node_")));
    tree.nodes.push_back(routeNode(4, 6, 3, "target", 37, randomToken(rng, "node_")));
    tree.edges.push_back(routeEdge(0, 1, randomToken(rng, "edge_")));
    tree.edges.push_back(routeEdge(1, 2, randomToken(rng, "edge_")));
    tree.edges.push_back(routeEdge(2, 3, randomToken(rng, "edge_")));
    tree.edges.push_back(routeEdge(2, 4, randomToken(rng, "edge_")));
    db::PnrDbRouteBranch left;
    left.logical_net_index = 7;
    left.owner_route_index = 3;
    left.logical_net = tree.aliases[0];
    left.route_name = tree.aliases[0] + ".left";
    left.owner = tree.sinks[0].inst;
    left.source = tree.source;
    left.sink = tree.sinks[0];
    left.wires = {wirePayload("left_0"),
                  routeEdgePayload("left_1", 0, 19, 2, 29)};
    tree.branches.push_back(left);
    db::PnrDbRouteBranch right = left;
    right.logical_net_index = 8;
    right.owner_route_index = 5;
    right.logical_net = tree.aliases[1];
    right.route_name = tree.aliases[1] + ".right";
    right.owner = tree.sinks[1].inst;
    right.sink = tree.sinks[1];
    right.wires = {wirePayload("right_0"), wirePayload("right_1")};
    tree.branches.push_back(right);
    return tree;
}

db::PnrDbRouteTree deeperHierarchyTree(std::mt19937& rng)
{
    db::PnrDbRouteTree tree;
    tree.net = randomToken(rng, "net_");
    tree.source = {"top.a.b.c.driver", "p"};
    tree.sinks.push_back({"top.x.y.z.load0", "s0"});
    tree.sinks.push_back({"top.x.y.z.load1", "s1"});
    for (uint32_t id = 0; id < 8; ++id) {
        tree.nodes.push_back(routeNode(id, static_cast<int>(id), static_cast<int>(id % 3),
            id == 0 ? "local" : (id + 1 == 8 ? "target" : "transit"),
            static_cast<int>(100 + id), randomToken(rng, "node_")));
    }
    for (uint32_t id = 0; id < 5; ++id) {
        tree.edges.push_back(routeEdge(id, id + 1, randomToken(rng, "edge_")));
    }
    tree.edges.push_back(routeEdge(5, 6, randomToken(rng, "edge_")));
    tree.edges.push_back(routeEdge(5, 7, randomToken(rng, "edge_")));
    return tree;
}

std::vector<db::PnrDbRouteTree> distributedRootTrees(std::mt19937& rng)
{
    std::vector<db::PnrDbRouteTree> trees;
    const std::string logical_net = randomToken(rng, "distributed_net_");
    const db::PnrDbEndpoint logical_source{"top.static_source", "out"};
    for (int component = 0; component < 2; ++component) {
        db::PnrDbRouteTree tree;
        tree.id = logical_net + ".component" + std::to_string(component);
        tree.net = logical_net;
        tree.aliases = {logical_net};
        tree.source = logical_source;
        tree.source.node = 0;
        tree.sinks.push_back({"top.distributed_load" + std::to_string(component), "in", 1});
        tree.nodes.push_back(routeNode(0, component * 4, 3, "source", 40 + component,
                                       randomToken(rng, "root_")));
        tree.nodes.push_back(routeNode(1, component * 4, 3, "local", 50 + component,
                                       randomToken(rng, "load_")));
        tree.edges.push_back(routeEdge(0, 1, randomToken(rng, "edge_")));

        db::PnrDbRouteBranch branch;
        branch.logical_net_index = 12;
        branch.owner_route_index = static_cast<uint32_t>(component);
        branch.logical_net = logical_net;
        branch.route_name = logical_net + ".load" + std::to_string(component);
        branch.owner = tree.sinks.front().inst;
        branch.source = tree.source;
        branch.sink = tree.sinks.front();
        branch.wires = {routeEdgePayload(randomToken(rng, "branch_"), 0,
                                         40 + component, 0, 50 + component)};
        tree.branches.push_back(std::move(branch));
        trees.push_back(std::move(tree));
    }
    return trees;
}

void assertNoThirdPartyNames(const db::PnrDbRouteTree& tree)
{
    static const std::vector<std::string> banned = {"INT", "CLB", "BRAM", "IOB", "SLICE"};
    auto check = [&](const std::string& value) {
        for (const std::string& item : banned) {
            require(value.find(item) == std::string::npos,
                "test route tree accidentally used third-party-like token '" + item + "' in '" + value + "'");
        }
    };
    check(tree.net);
    check(tree.source.inst);
    check(tree.source.port);
    for (const db::PnrDbEndpoint& sink : tree.sinks) {
        check(sink.inst);
        check(sink.port);
    }
    for (const db::PnrDbRouteNode& node : tree.nodes) {
        check(node.kind);
        check(node.name);
    }
}

void compareTrees(const db::PnrDbRouteTree& expected, const db::PnrDbRouteTree& actual)
{
    require(expected.id == actual.id, "physical net id changed during route tree round-trip");
    require(expected.net == actual.net, "net name changed during route tree round-trip");
    require(expected.aliases == actual.aliases, "physical net aliases changed during route tree round-trip");
    require(expected.source.inst == actual.source.inst && expected.source.port == actual.source.port,
        "source endpoint changed during route tree round-trip");
    require(expected.source.node == actual.source.node, "source route node changed during route tree round-trip");
    require(expected.sinks.size() == actual.sinks.size(), "sink count changed during route tree round-trip");
    require(expected.branches.size() == actual.branches.size(), "branch count changed during route tree round-trip");
    require(expected.nodes.size() == actual.nodes.size(), "node count changed during route tree round-trip");
    require(expected.edges.size() == actual.edges.size(), "edge count changed during route tree round-trip");
    require(db::routeTreeHasConsistentIds(actual), "round-tripped route tree has inconsistent node ids");

    for (size_t i = 0; i < expected.sinks.size(); ++i) {
        require(expected.sinks[i].inst == actual.sinks[i].inst && expected.sinks[i].port == actual.sinks[i].port,
            "sink endpoint changed during route tree round-trip");
        require(expected.sinks[i].node == actual.sinks[i].node,
            "sink route node changed during route tree round-trip");
    }
    for (size_t i = 0; i < expected.branches.size(); ++i) {
        const db::PnrDbRouteBranch& left = expected.branches[i];
        const db::PnrDbRouteBranch& right = actual.branches[i];
        require(left.logical_net_index == right.logical_net_index
                && left.owner_route_index == right.owner_route_index
                && left.logical_net == right.logical_net,
            "branch logical net identity changed during route tree round-trip");
        require(left.route_name == right.route_name && left.owner == right.owner,
            "branch route ownership changed during route tree round-trip");
        require(left.source.inst == right.source.inst && left.source.port == right.source.port
                && left.source.node == right.source.node,
            "branch source changed during route tree round-trip");
        require(left.sink.inst == right.sink.inst && left.sink.port == right.sink.port
                && left.sink.node == right.sink.node,
            "branch sink changed during route tree round-trip");
        require(left.wires.size() == right.wires.size(), "branch wire count changed during route tree round-trip");
        for (size_t j = 0; j < left.wires.size(); ++j) {
            require(left.wires[j]["label"].asString() == right.wires[j]["label"].asString(),
                "branch wire payload changed during route tree round-trip");
            require(left.wires[j]["type"].asString() == right.wires[j]["type"].asString(),
                "branch wire type changed during route tree round-trip");
            if (left.wires[j]["type"].asString() == "route_edge") {
                require(left.wires[j]["from_node_type"].asInt() == right.wires[j]["from_node_type"].asInt()
                        && left.wires[j]["from_node"].asInt() == right.wires[j]["from_node"].asInt()
                        && left.wires[j]["to_node_type"].asInt() == right.wires[j]["to_node_type"].asInt()
                        && left.wires[j]["to_node"].asInt() == right.wires[j]["to_node"].asInt(),
                    "typed route-edge identity changed during route tree round-trip");
            }
        }
    }
    for (size_t i = 0; i < expected.nodes.size(); ++i) {
        require(expected.nodes[i].id == actual.nodes[i].id, "node id changed during route tree round-trip");
        require(expected.nodes[i].coord.x == actual.nodes[i].coord.x
                && expected.nodes[i].coord.y == actual.nodes[i].coord.y,
            "node coord changed during route tree round-trip");
        require(expected.nodes[i].kind == actual.nodes[i].kind, "node kind changed during route tree round-trip");
        require(expected.nodes[i].node == actual.nodes[i].node, "node number changed during route tree round-trip");
        require(expected.nodes[i].name == actual.nodes[i].name, "node name changed during route tree round-trip");
    }
    for (size_t i = 0; i < expected.edges.size(); ++i) {
        require(expected.edges[i].from == actual.edges[i].from && expected.edges[i].to == actual.edges[i].to,
            "edge endpoint changed during route tree round-trip");
        require(expected.edges[i].wire["label"].asString() == actual.edges[i].wire["label"].asString(),
            "edge wire payload changed during route tree round-trip");
    }
}

void runPnrDbRoundTrip()
{
    std::mt19937 rng(0x5eed1234);
    std::vector<db::PnrDbRouteTree> trees;
    trees.push_back(singleSinkTree(rng));
    trees.push_back(forkedTree(rng));
    trees.push_back(deeperHierarchyTree(rng));
    std::vector<db::PnrDbRouteTree> distributed = distributedRootTrees(rng);
    trees.insert(trees.end(), std::make_move_iterator(distributed.begin()),
                 std::make_move_iterator(distributed.end()));

    Json::Value root(Json::objectValue);
    root["format"] = "scalepnr-design-state";
    root["version"] = 2;

    Json::Value insts(Json::arrayValue);
    insts.append(placedInst("top.unit0.src", "cell_type_alpha", 1, 1, 3));
    insts.append(placedInst("top.unit1.dst", "cell_type_beta", 2, 1, 7));
    insts.append(placedInst("top.block.producer", "cell_type_gamma", 4, 2, 19));
    insts.append(placedInst("top.block.consumer0", "cell_type_delta", 6, 1, 31));
    insts.append(placedInst("top.block.consumer1", "cell_type_delta", 6, 3, 37));
    root["insts"] = insts;

    Json::Value route_trees(Json::arrayValue);
    for (const db::PnrDbRouteTree& tree : trees) {
        assertNoThirdPartyNames(tree);
        require(db::routeTreeHasConsistentIds(tree), "test route tree has inconsistent ids before write");
        route_trees.append(db::routeTreeToJson(tree));
    }
    root["route_trees"] = route_trees;

    std::filesystem::path path = std::filesystem::temp_directory_path() / "scalepnr_pnr_db_test.db";
    {
        std::ofstream out(path);
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(root, &out);
    }

    Json::Value loaded;
    {
        std::ifstream in(path);
        Json::CharReaderBuilder builder;
        std::string errors;
        require(Json::parseFromStream(builder, in, &loaded, &errors), "failed to parse round-tripped DB: " + errors);
    }
    std::filesystem::remove(path);

    require(loaded["insts"].size() == insts.size(), "placed instance hierarchy changed during DB round-trip");
    require(loaded["route_trees"].size() == trees.size(), "route tree count changed during DB round-trip");
    for (Json::ArrayIndex i = 0; i < loaded["route_trees"].size(); ++i) {
        compareTrees(trees[i], db::routeTreeFromJson(loaded["route_trees"][i]));
    }
}

}

int main()
{
    try {
        runPnrDbRoundTrip();
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "pnr_db_test failed: %s\n", e.what());
        return 1;
    }
    return 0;
}
