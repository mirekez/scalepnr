#include "Crossbar.h"

#include <cstdio>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
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

std::string token(const char* relation, int id)
{
    return "|" + std::string(relation) + ":" + std::to_string(id) + "|";
}

std::string nodeName(const char* kind, int id, const std::vector<std::string>& relation_tokens)
{
    std::string out = std::string(kind) + "_ID:" + std::to_string(id) + "|";
    for (const std::string& item : relation_tokens) {
        out += item;
    }
    return out;
}

bool hasToken(const std::string& text, const char* relation, int id)
{
    return text.find(token(relation, id)) != std::string::npos;
}

int encodeSigned4(int value)
{
    return value & 0xf;
}

int encodeJump(int dx, int dy, int num)
{
    return (encodeSigned4(dx) << 8) | (encodeSigned4(dy) << 4) | (num & 0xf);
}

void addBit(NodeMask& mask, int bit)
{
    mask |= NodeMask{0, 1} << bit;
}

bool hasBit(const NodeMask& mask, int bit)
{
    return (mask & (NodeMask{0, 1} << bit)) != NodeMask{};
}

fpga::TechMap oneNumberJumpMap()
{
    fpga::TechMap map;
    map.push_back({
        {{{"BEG"}}, {{"SRC"}}},
        {{{"END"}}, {{"DST"}}},
        {{{"_N3"}}, {{"_ND"}}},
    });
    map.push_back({
        {{{"E"}}, {{"2"}, {"1", "2", "4", "6"}}},
    });
    return map;
}

void checkNodeName(const fpga::CBType& cb, fpga::CBNodeNameType type, int id, const char* kind)
{
    const std::string* name = cb.nodeName(type, id);
    require(name != nullptr, std::string("missing node name for ") + kind + " id " + std::to_string(id));
    std::string expected_prefix = std::string(kind) + "_ID:" + std::to_string(id) + "|";
    require(name->rfind(expected_prefix, 0) == 0,
        std::string("wrong namespace for ") + kind + " id " + std::to_string(id)
            + ": got '" + *name + "', expected prefix '" + expected_prefix + "'");
}

void checkMaskRelation(const fpga::CBType& cb, const NodeMask& mask,
                       fpga::CBNodeNameType from_type, int from_id, const char* from_kind,
                       fpga::CBNodeNameType to_type, const char* to_kind,
                       const char* relation)
{
    const std::string* from_name = cb.nodeName(from_type, from_id);
    require(from_name != nullptr, std::string("missing from node name for relation ") + relation);
    mask.for_each_set_bit([&](int to_id) {
        checkNodeName(cb, to_type, to_id, to_kind);
        require(hasToken(*from_name, relation, to_id),
            std::string("relation ") + relation + " from " + from_kind + " id " + std::to_string(from_id)
                + " reaches " + to_kind + " id " + std::to_string(to_id)
                + ", but from name has no matching token: '" + *from_name + "'");
        return false;
    });
}

std::vector<int> randomTargets(std::mt19937& rng, const std::vector<int>& ids, int count)
{
    std::vector<int> targets;
    std::uniform_int_distribution<size_t> pick(0, ids.size() - 1);
    while (static_cast<int>(targets.size()) < count) {
        int id = ids[pick(rng)];
        bool exists = false;
        for (int current : targets) {
            if (current == id) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            targets.push_back(id);
        }
    }
    return targets;
}

void runNamespaceRegression(unsigned seed)
{
    auto cb_storage = std::make_unique<fpga::CBType>();
    fpga::CBType& cb = *cb_storage;
    cb.name = "CB_NAMES_TEST";

    std::mt19937 rng(seed);
    std::vector<int> ids = {3, 7, 11, 19, 29, 37, 53, 71, 89, 113, 131, 157};

    std::vector<std::vector<std::string>> local_tokens(CB_MAX_NODES);
    std::vector<std::vector<std::string>> dst_tokens(CB_MAX_NODES);
    std::vector<std::vector<std::string>> joint_tokens(CB_MAX_NODES);

    for (int local : ids) {
        for (int src : randomTargets(rng, ids, 3)) {
            addBit(cb.local_src[local].jump, src);
            local_tokens[local].push_back(token("LS", src));
        }
        for (int joint : randomTargets(rng, ids, 2)) {
            addBit(cb.local_joint[local].joint, joint);
            local_tokens[local].push_back(token("LJ", joint));
        }
    }

    for (int dst : ids) {
        for (int src : randomTargets(rng, ids, 3)) {
            addBit(cb.dst_src[dst].jump, src);
            dst_tokens[dst].push_back(token("DS", src));
        }
        for (int local : randomTargets(rng, ids, 2)) {
            addBit(cb.dst_local[dst].local, local);
            dst_tokens[dst].push_back(token("DL", local));
        }
        for (int joint : randomTargets(rng, ids, 2)) {
            addBit(cb.dst_joint[dst].joint, joint);
            dst_tokens[dst].push_back(token("DJ", joint));
        }
    }

    for (int joint : ids) {
        for (int src : randomTargets(rng, ids, 3)) {
            addBit(cb.joint_src[joint].jump, src);
            joint_tokens[joint].push_back(token("JS", src));
        }
    }

    for (int id : ids) {
        cb.rememberNodeName(fpga::CB_NODE_LOCAL, id, nodeName("LOCAL", id, local_tokens[id]));
        cb.rememberNodeName(fpga::CB_NODE_DST, id, nodeName("DST", id, dst_tokens[id]));
        cb.rememberNodeName(fpga::CB_NODE_JOINT, id, nodeName("JOINT", id, joint_tokens[id]));
        cb.rememberNodeName(fpga::CB_NODE_SRC, id, nodeName("SRC", id, {}));
        cb.rememberNodeName(fpga::CB_NODE_JUMP, id, nodeName("JUMP", id, {}));
    }

    for (int id : ids) {
        checkNodeName(cb, fpga::CB_NODE_LOCAL, id, "LOCAL");
        checkNodeName(cb, fpga::CB_NODE_DST, id, "DST");
        checkNodeName(cb, fpga::CB_NODE_JOINT, id, "JOINT");
        checkNodeName(cb, fpga::CB_NODE_SRC, id, "SRC");
        checkNodeName(cb, fpga::CB_NODE_JUMP, id, "JUMP");
    }

    for (int local : ids) {
        checkMaskRelation(cb, cb.local_src[local].jump,
            fpga::CB_NODE_LOCAL, local, "LOCAL",
            fpga::CB_NODE_SRC, "SRC", "LS");
        checkMaskRelation(cb, cb.local_joint[local].joint,
            fpga::CB_NODE_LOCAL, local, "LOCAL",
            fpga::CB_NODE_JOINT, "JOINT", "LJ");
    }

    for (int dst : ids) {
        checkMaskRelation(cb, cb.dst_src[dst].jump,
            fpga::CB_NODE_DST, dst, "DST",
            fpga::CB_NODE_SRC, "SRC", "DS");
        checkMaskRelation(cb, cb.dst_local[dst].local,
            fpga::CB_NODE_DST, dst, "DST",
            fpga::CB_NODE_LOCAL, "LOCAL", "DL");
        checkMaskRelation(cb, cb.dst_joint[dst].joint,
            fpga::CB_NODE_DST, dst, "DST",
            fpga::CB_NODE_JOINT, "JOINT", "DJ");
    }

    for (int joint : ids) {
        checkMaskRelation(cb, cb.joint_src[joint].jump,
            fpga::CB_NODE_JOINT, joint, "JOINT",
            fpga::CB_NODE_SRC, "SRC", "JS");
    }
}

void runEndpointDstIsNotJumpRegression()
{
    auto cb_storage = std::make_unique<fpga::CBType>();
    fpga::CBType& cb = *cb_storage;
    cb.name = "CB_ENDPOINT_DST_TEST";

    int shared_id = 128;
    int local_id = 17;
    int src_id = 129;

    cb.rememberNodeName(fpga::CB_NODE_DST, shared_id, "DST_ENDPOINT_PORTLIKE_ID:128|DL:17|DS:129|");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, local_id, "LOCAL_PIN_ID:17|");
    cb.rememberNodeName(fpga::CB_NODE_SRC, src_id, "SRC_ID:129|");
    cb.rememberNodeName(fpga::CB_NODE_JUMP, src_id, "JUMP_ID:129|");
    cb.dst_local[shared_id].local |= NodeMask{0, 1} << local_id;

    const std::string* dst_name = cb.nodeName(fpga::CB_NODE_DST, shared_id);
    require(dst_name != nullptr && dst_name->find("PORTLIKE") != std::string::npos,
        "endpoint DST fixture did not store endpoint name");

    const std::string* jump_name = cb.nodeName(fpga::CB_NODE_JUMP, shared_id);
    require(jump_name == nullptr,
        std::string("endpoint-only DST leaked into jump namespace: '")
            + (jump_name ? *jump_name : std::string{}) + "'");

    cb.dst_src[shared_id].jump |= NodeMask{0, 1} << src_id;
    checkMaskRelation(cb, cb.dst_src[shared_id].jump,
        fpga::CB_NODE_DST, shared_id, "DST",
        fpga::CB_NODE_SRC, "SRC", "DS");

    jump_name = cb.nodeName(fpga::CB_NODE_JUMP, shared_id);
    require(jump_name == nullptr,
        std::string("route-through DST leaked endpoint name into jump namespace: '")
            + (jump_name ? *jump_name : std::string{}) + "'");

    int mixed_id = 130;
    cb.rememberNodeName(fpga::CB_NODE_DST, mixed_id, "DST_ENDPOINT_PORTLIKE_ID:130|DL:17|");
    cb.rememberNodeName(fpga::CB_NODE_SRC, mixed_id, "SRC_ID:130|");
    cb.rememberNodeName(fpga::CB_NODE_JUMP, mixed_id, "JUMP_ID:130|");

    dst_name = cb.nodeName(fpga::CB_NODE_DST, mixed_id);
    const std::string* src_name = cb.nodeName(fpga::CB_NODE_SRC, mixed_id);
    jump_name = cb.nodeName(fpga::CB_NODE_JUMP, mixed_id);
    require(dst_name != nullptr && dst_name->rfind("DST_ENDPOINT_PORTLIKE_ID:130|", 0) == 0,
        "mixed-id DST endpoint name was not preserved");
    require(src_name != nullptr && src_name->rfind("SRC_ID:130|", 0) == 0,
        "mixed-id SRC name was not preserved");
    require(jump_name != nullptr && jump_name->rfind("JUMP_ID:130|", 0) == 0,
        std::string("mixed-id JUMP lookup returned wrong namespace: '")
            + (jump_name ? *jump_name : std::string{}) + "'");
}

void runOneNumberNormalizedJumpRegression()
{
    fpga::CBType cb;
    cb.name = "ABC_ROUTE_BOX";
    CBTypeSpec spec;
    spec.nodes.emplace("ABC_OUTPUT0", "EL1BEG_N3");
    spec.nodes.emplace("ABC_OUTPUT0", "USRCCLKO");

    fpga::TechMap map = oneNumberJumpMap();
    cb.loadFromSpec(spec, map);

    int output = cb.nodeNum(fpga::CB_NODE_LOCAL, "ABC_OUTPUT0");
    int boundary_src = cb.nodeNum(fpga::CB_NODE_SRC, "EL1BEG_N3");
    int ordinary_local = cb.nodeNum(fpga::CB_NODE_LOCAL, "USRCCLKO");
    require(output >= 0, "one-number fixture output local was not loaded");
    require(boundary_src >= 0,
        "normalized one-number boundary wire was not classified as SRC");
    require(cb.nodeNum(fpga::CB_NODE_LOCAL, "EL1BEG_N3") < 0,
        "normalized one-number boundary wire leaked into LOCAL nodes");
    require(hasBit(cb.local_src[output].jump, boundary_src),
        "normalized one-number boundary SRC was not connected to its local");
    require(ordinary_local >= 0,
        "ordinary local containing the letters SRC was misclassified as a jump");
}

void runIntraCbWireContinuityRegression()
{
    fpga::CBType cb;
    cb.name = "QBOX";
    cb.type_id = 0;
    CBTypeSpec spec;
    // Two PIPs meeting on one physical wire, with deliberately different
    // SRC/DST lane allocation. Names have no technology-specific meaning.
    spec.nodes.emplace("PORT0", "A1SRC");
    spec.nodes.emplace("PORT0", "Q1SRC");
    spec.nodes.emplace("Q1SRC", "PORT1");
    fpga::TechMap map;
    map.push_back({});
    map.push_back({{{{"A"}}, {{"2"}, {"1", "1", "1", "1"}}},
                   {{{"Q"}}, {{"2"}, {"1", "1", "1", "1"}}}});
    cb.loadFromSpec(spec, map);
    const int src = cb.nodeNum(fpga::CB_NODE_SRC, "Q1SRC");
    const int dst = cb.nodeNum(fpga::CB_NODE_DST, "Q1SRC");
    require(src >= 0 && dst >= 0 && src != dst,
            "continuity fixture did not separate the numeric roles");
    const auto& links = std::as_const(cb).dst_by_src[src];
    require(links.size() == 1 && links[0].delta.x == 0 && links[0].delta.y == 0 &&
                links[0].dsts.jump == (NodeMask{0, 1} << dst),
            "same physical wire lost its zero-distance SRC-to-DST continuity");
}

void addResolvedStep(fpga::CBType& from, int src, fpga::CBType& to, int dst, fpga::Coord delta)
{

    fpga::CBJumpState dsts{};
    addBit(dsts.jump, dst);
    from.dst_by_src[src].push_back(fpga::CBType::ResolvedJump{
        delta,
        to.type_id,
        dsts,
        {},
        true,
    });
}

void checkResolvedStep(const fpga::CBType& from, int src, const fpga::CBType& to, int dst,
                       fpga::Coord delta, const char* label)
{
    require(hasBit(from.dstMaskForSrc(src), dst),
        std::string(label) + ": src dst mapping is missing src " + std::to_string(src)
            + " -> dst " + std::to_string(dst));

    bool found = false;
    for (const fpga::CBType::ResolvedJump& entry : from.dst_by_src[src]) {
        if (entry.target_cb_type_id != to.type_id) {
            continue;
        }
        if (entry.delta.x != delta.x || entry.delta.y != delta.y) {
            continue;
        }
        if (!hasBit(entry.dsts.jump, dst)) {
            continue;
        }
        found = true;
        break;
    }
    require(found,
        std::string(label) + ": dst_by_src is missing matching one-step target for src "
            + std::to_string(src) + " -> dst " + std::to_string(dst));
}

void checkDstSrcStep(const fpga::CBType& cb, int dst, int src, const char* label)
{
    require(hasBit(cb.dst_src[dst].jump, src),
        std::string(label) + ": dst_src is missing dst " + std::to_string(dst)
            + " -> src " + std::to_string(src));
}

void runStepwisePassThroughRegression()
{
    auto alpha_storage = std::make_unique<fpga::CBType>();
    auto bravo_storage = std::make_unique<fpga::CBType>();
    auto charlie_storage = std::make_unique<fpga::CBType>();
    auto delta_storage = std::make_unique<fpga::CBType>();
    fpga::CBType& alpha = *alpha_storage;
    fpga::CBType& bravo = *bravo_storage;
    fpga::CBType& charlie = *charlie_storage;
    fpga::CBType& delta_cb = *delta_storage;

    alpha.name = "STEP_ALPHA";
    bravo.name = "STEP_BRAVO";
    charlie.name = "STEP_CHARLIE";
    delta_cb.name = "STEP_DELTA";
    alpha.type_id = 1;
    bravo.type_id = 2;
    charlie.type_id = 3;
    delta_cb.type_id = 4;

    fpga::Coord east{1, 0};
    int alpha_src = encodeJump(1, 0, 0);
    int bravo_dst = encodeJump(1, 0, 0);
    int bravo_src = encodeJump(1, 0, 1);
    int charlie_dst = encodeJump(1, 0, 1);
    int charlie_src = encodeJump(1, 0, 2);
    int delta_dst = encodeJump(1, 0, 2);
    int endpoint_dst = encodeJump(1, 0, 7);
    int endpoint_local = 42;

    alpha.rememberNodeName(fpga::CB_NODE_SRC, alpha_src, "SRC_ID:" + std::to_string(alpha_src) + "|STEP0|");
    alpha.rememberNodeName(fpga::CB_NODE_JUMP, alpha_src, "JUMP_ID:" + std::to_string(alpha_src) + "|STEP0|");
    bravo.rememberNodeName(fpga::CB_NODE_DST, bravo_dst, "DST_ID:" + std::to_string(bravo_dst) + "|STEP0_DST|");
    bravo.rememberNodeName(fpga::CB_NODE_SRC, bravo_src, "SRC_ID:" + std::to_string(bravo_src) + "|STEP1_SRC|");
    bravo.rememberNodeName(fpga::CB_NODE_JUMP, bravo_src, "JUMP_ID:" + std::to_string(bravo_src) + "|STEP1_SRC|");
    charlie.rememberNodeName(fpga::CB_NODE_DST, charlie_dst, "DST_ID:" + std::to_string(charlie_dst) + "|STEP1_DST|");
    charlie.rememberNodeName(fpga::CB_NODE_SRC, charlie_src, "SRC_ID:" + std::to_string(charlie_src) + "|STEP2_SRC|");
    charlie.rememberNodeName(fpga::CB_NODE_JUMP, charlie_src, "JUMP_ID:" + std::to_string(charlie_src) + "|STEP2_SRC|");
    delta_cb.rememberNodeName(fpga::CB_NODE_DST, delta_dst, "DST_ID:" + std::to_string(delta_dst) + "|STEP2_DST|");

    // Endpoint-only DST shares the numeric jump range but must not participate in pass-through checks.
    charlie.rememberNodeName(fpga::CB_NODE_DST, endpoint_dst, "DST_ENDPOINT_PORTLIKE_ID:" + std::to_string(endpoint_dst) + "|");
    charlie.rememberNodeName(fpga::CB_NODE_LOCAL, endpoint_local, "LOCAL_PIN_ID:" + std::to_string(endpoint_local) + "|");
    addBit(charlie.dst_local[endpoint_dst].local, endpoint_local);

    addResolvedStep(alpha, alpha_src, bravo, bravo_dst, east);
    addBit(bravo.dst_src[bravo_dst].jump, bravo_src);
    addResolvedStep(bravo, bravo_src, charlie, charlie_dst, east);
    addBit(charlie.dst_src[charlie_dst].jump, charlie_src);
    addResolvedStep(charlie, charlie_src, delta_cb, delta_dst, east);

    checkResolvedStep(alpha, alpha_src, bravo, bravo_dst, east, "step0 ALPHA->BRAVO");
    checkDstSrcStep(bravo, bravo_dst, bravo_src, "step0 BRAVO internal pass-through");
    checkResolvedStep(bravo, bravo_src, charlie, charlie_dst, east, "step1 BRAVO->CHARLIE");
    checkDstSrcStep(charlie, charlie_dst, charlie_src, "step1 CHARLIE internal pass-through");
    checkResolvedStep(charlie, charlie_src, delta_cb, delta_dst, east, "step2 CHARLIE->DELTA");

    require(!hasBit(charlie.dst_src[endpoint_dst].jump, charlie_src),
        "endpoint-only DST was incorrectly usable as route-through dst_src");
    require(charlie.nodeName(fpga::CB_NODE_JUMP, endpoint_dst) == nullptr,
        "endpoint-only DST leaked into jump namespace in stepwise pass-through test");
}

void runSharedSubtypeNamesRegression()
{
    static_assert(sizeof(fpga::InternedString) == sizeof(const std::string*));
    static_assert(sizeof(fpga::CBConnName) == 2 * sizeof(const std::string*));
    const std::string source_name = "ABC_SHARED_SOURCE_NAME_LONGER_THAN_INLINE_STORAGE";
    const std::string target_name = "XYZ_SHARED_TARGET_NAME_LONGER_THAN_INLINE_STORAGE";
    const fpga::CBNodeNameKey key{fpga::CB_NODE_SRC, 7};
    fpga::CBType base;
    base.rememberNodeName(fpga::CB_NODE_SRC, 7, source_name);
    base.rememberNodeName(fpga::CB_NODE_DST, 9, target_name);
    base.rememberConnName(fpga::CB_NODE_DST, 9, fpga::CB_NODE_SRC, 7, target_name, source_name);
    base.dst_src[9].jump = NodeMask{0, 1} << 7;
    fpga::CBType::ResolvedJump jump;
    jump.delta = {1, 0};
    jump.dsts.jump = NodeMask{0, 1} << 9;
    jump.dst_wires[9] = target_name;
    base.dst_by_src[7].push_back(jump);

    const std::string* source = base.nodeName(fpga::CB_NODE_SRC, 7);
    const std::string* target = base.nodeName(fpga::CB_NODE_DST, 9);
    std::vector<fpga::CBType> subtypes(32, base);
    for (const auto& subtype : subtypes) {
        // Every copy, role and connection endpoint must reference the same pooled string object.
        require(subtype.nodeName(fpga::CB_NODE_SRC, 7) == source, "subtype copied source text");
        require(subtype.nodeName(fpga::CB_NODE_DST, 9) == target, "subtype copied target text");
        const auto* conn = subtype.connName(fpga::CB_NODE_DST, 9, fpga::CB_NODE_SRC, 7);
        require(conn && &conn->from.str() == target && &conn->to.str() == source,
            "connection names do not share node text");
        require(&subtype.dst_by_src[7][0].dst_wires.at(9).str() == target,
            "resolved jump copied destination text");
        require(subtype.sameRoutingSubtype(base), "interning changed subtype equality");
    }

    // A subtype can override its names without modifying the base or its siblings.
    auto& changed = subtypes.front();
    changed.node_names[key] = "ABC_SUBTYPE_ONLY_SOURCE";
    changed.dst_by_src[7][0].dst_wires[9] = "XYZ_SUBTYPE_ONLY_TARGET";
    changed.conn_names[{fpga::CB_NODE_DST, 9, fpga::CB_NODE_SRC, 7}][0].to = "ABC_SUBTYPE_ONLY_SOURCE";
    require(*base.nodeName(fpga::CB_NODE_SRC, 7) == source_name, "subtype changed base source name");
    require(subtypes[1].nodeName(fpga::CB_NODE_SRC, 7) == source, "subtype changed sibling name");
    require(base.dst_by_src[7][0].dst_wires.at(9) == target_name, "subtype changed base destination name");
    require(base.connName(fpga::CB_NODE_DST, 9, fpga::CB_NODE_SRC, 7)->to == source_name,
        "subtype changed base connection name");
    require(!changed.sameRoutingSubtype(base), "subtype name override was ignored");
    require(changed.dst_src[9].jump == base.dst_src[9].jump, "name override changed routing mask");

    // Base destruction, pool growth and subtype-vector relocation cannot invalidate published names.
    base = fpga::CBType{};
    for (int i = 0; i < 4096; ++i) {
        fpga::InternedString extra("UNRELATED_POOL_ENTRY_" + std::to_string(i));
    }
    subtypes.reserve(128);
    require(subtypes[1].nodeName(fpga::CB_NODE_SRC, 7) == source && *source == source_name,
        "source reference did not survive storage growth");
    require(&subtypes[1].dst_by_src[7][0].dst_wires.at(9).str() == target && *target == target_name,
        "destination reference did not survive storage growth");
    require(subtypes[1].nodeName(fpga::CB_NODE_LOCAL, 7) == nullptr,
        "interned source name leaked into another node namespace");
}

}

int main()
{
    try {
        for (unsigned seed = 1; seed <= 25; ++seed) {
            runNamespaceRegression(seed);
        }
        runEndpointDstIsNotJumpRegression();
        runOneNumberNormalizedJumpRegression();
        runIntraCbWireContinuityRegression();
        runStepwisePassThroughRegression();
        runSharedSubtypeNamesRegression();
    }
    catch (const Failure& failure) {
        std::fprintf(stderr, "cb_names_test failed: %s\n", failure.what());
        return 1;
    }
    catch (const std::exception& ex) {
        std::fprintf(stderr, "cb_names_test exception: %s\n", ex.what());
        return 1;
    }
    return 0;
}
