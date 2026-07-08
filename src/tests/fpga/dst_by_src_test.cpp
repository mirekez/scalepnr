#include "Device.h"

#include <memory>
#include <stdexcept>
#include <string>

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

NodeMask bit(int index)
{
    return NodeMask{0, 1} << index;
}

int jumpIndex(int dx, int dy, int lane)
{
    auto encode = [](int value) {
        return value & 0xf;
    };
    return (encode(dx) << 8) | (encode(dy) << 4) | (lane & 0xf);
}

void runSegmentWireDoesNotResolveBeforeSwitchableEndpointRegression()
{
    auto cb_storage = std::make_unique<fpga::CBType>();
    fpga::CBType& cb = *cb_storage;
    cb.name = "ABC_ROUTE_BOX";

    constexpr int src_beg = 1906;
    constexpr int dst_end = 1907;

    cb.rememberNodeName(fpga::CB_NODE_SRC, src_beg, "ABC2BEG2");
    cb.rememberNodeName(fpga::CB_NODE_JUMP, src_beg, "ABC2BEG2");
    cb.rememberNodeName(fpga::CB_NODE_DST, dst_end, "ABC2END2");
    cb.dst_src[dst_end].jump |= bit(src_beg);

    // Intermediate segment names must be followed through tileconn, not
    // aliased to the endpoint before the switchbox is reached.
    cb.rememberNodeName(fpga::CB_NODE_DST, src_beg, "ABC2BEG2");

    int resolved_dst = fpga::testRouteDstNodeByPhysicalWireName(cb, "ABC2A2");
    require(resolved_dst == -1,
        "segment wire ABC2A2 must not resolve before switchable endpoint, got node "
            + std::to_string(resolved_dst));

    int endpoint_dst = fpga::testRouteDstNodeByPhysicalWireName(cb, "ABC2END2");
    require(endpoint_dst == dst_end,
        "switchable endpoint ABC2END2 must resolve to endpoint node, got node "
            + std::to_string(endpoint_dst));

    int resolved_src = fpga::testRouteSrcNodeByPhysicalWireName(cb, "ABC2A2");
    require(resolved_src == -1,
        "segment wire ABC2A2 must not be accepted as a source, got node "
            + std::to_string(resolved_src));

    int exact_src = fpga::testRouteSrcNodeByPhysicalWireName(cb, "ABC2BEG2");
    require(exact_src == src_beg,
        "exact source endpoint ABC2BEG2 must still resolve to its source node, got node "
            + std::to_string(exact_src));
}

void runLocalEndpointIsNotTransitDstRegression()
{
    auto cb_storage = std::make_unique<fpga::CBType>();
    fpga::CBType& cb = *cb_storage;
    cb.name = "ABC_ROUTE_BOX";

    constexpr int shared_id = 114;
    constexpr int src_id = 512;
    constexpr int local_neighbor = 33;

    cb.rememberNodeName(fpga::CB_NODE_DST, shared_id, "ABC_REAL_END0");
    cb.rememberNodeName(fpga::CB_NODE_SRC, src_id, "ABC_REAL_BEG0");
    cb.rememberNodeName(fpga::CB_NODE_JUMP, src_id, "ABC_REAL_BEG0");
    cb.dst_src[shared_id].jump |= NodeMask{0, 1} << src_id;

    cb.rememberNodeName(fpga::CB_NODE_LOCAL, shared_id, "ABC_INPUT_MUX32");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, local_neighbor, "ABC_INPUT_PIN33");
    cb.local_src[shared_id].jump |= NodeMask{0, 1} << src_id;
    cb.local_local[shared_id].local |= NodeMask{0, 1} << local_neighbor;

    int resolved_dst = fpga::testRouteDstNodeByPhysicalWireName(cb, "ABC_INPUT_MUX32");
    require(resolved_dst == -1,
        "local-only endpoint ABC_INPUT_MUX32 must not resolve as a route DST, got node "
            + std::to_string(resolved_dst));

    const std::string* dst_name = cb.nodeName(fpga::CB_NODE_DST, shared_id);
    require(dst_name != nullptr && *dst_name == "ABC_REAL_END0",
        std::string("local endpoint overwrote existing DST name, got '")
            + (dst_name ? *dst_name : std::string{}) + "'");

    require((cb.dst_src[shared_id].jump & (NodeMask{0, 1} << src_id)) != NodeMask{},
        "existing real dst_src route was unexpectedly removed");
    require((cb.dst_local[shared_id].local & (NodeMask{0, 1} << local_neighbor)) == NodeMask{},
        "local endpoint leaked local_local into dst_local");
}

void runWideResolvedDeltaDoesNotDropSourceRegression()
{
    fpga::CBType cb;
    cb.name = "ABC_ROUTE_BOX";
    cb.type_id = 3;

    constexpr int src = 527; // encoded source bucket is east, lane 15.
    constexpr int dst = 512;

    cb.rememberNodeName(fpga::CB_NODE_SRC, src, "ABC_EAST_BEGIN0");
    cb.rememberNodeName(fpga::CB_NODE_JUMP, src, "ABC_EAST_BEGIN0");
    cb.rememberNodeName(fpga::CB_NODE_DST, dst, "ABC_EAST_ARRIVE0");
    cb.dst_src[dst].jump |= bit(src);

    fpga::CBJumpState dsts{};
    dsts.jump = bit(dst);
    cb.dst_by_src[src].push_back(fpga::CBType::ResolvedJump{{2, 0}, cb.type_id, dsts, {}, false});
    cb.rebuildOutgoingSrcs();
    cb.ensureDerivedMasks();

    require((cb.dstMaskForSrc(src) & bit(dst)) != NodeMask{},
        "lane-15 resolved delta was dropped from dst_by_src");
    require((cb.priority_srcs_by_delta[jumpIndex(2, 0, 0)].jump & bit(src)) != NodeMask{},
        "lane-15 resolved delta did not keep the source in its encoded priority bucket");
}

void runPrefixedWireFamilyRegression()
{
    require(fpga::testRouteWireFamilyKey("QQ_N6BEG3") == "N6|3",
        "plain begin wire did not produce the expected route family");
    require(fpga::testRouteWireFamilyKey("QQ_CLOCK_N6A3") == "N6|3",
        "prefixed intermediate segment did not keep the plain route family");
    require(fpga::testRouteWireFamilyKey("QQ_TERM_ALIAS_N6END3") == "N6|3",
        "prefixed endpoint segment did not keep the plain route family");
    require(fpga::testRouteWireFamilyKey("QQ_OTHER5A3") != "N6|3",
        "unrelated route family matched the source family");
}

}

int main()
{
    runSegmentWireDoesNotResolveBeforeSwitchableEndpointRegression();
    runLocalEndpointIsNotTransitDstRegression();
    runWideResolvedDeltaDoesNotDropSourceRegression();
    runPrefixedWireFamilyRegression();
    return 0;
}
