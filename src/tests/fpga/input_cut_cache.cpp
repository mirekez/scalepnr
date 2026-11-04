#include "SourceInputFixture.h"
#include "route/MovingInputCutCache.h"

// Exercise proof identity/invalidation separately from the production moving
// fixtures, which verify real topology, ownership and successful coexistence.
void input_cut_cache_regression() {
    using namespace routing_tests;
    SourceInputFixture f(false);
    pnr::MovingInputCutCache cache;
    pnr::MovingInputCutCache::Key key;
    key.target=&f.tile({2,0});key.pins.setBit(63);
    key.source_hint=f.control->coord;key.source_pos=f.control->pos;
    key.radius=12;key.depth=200;key.anchor=&f.tile({5,1});key.dst=711;
    const std::vector<fpga::RouteCutNode> releases{
        {{6,0},fpga::CB_NODE_SRC,910},{{6,1},fpga::CB_NODE_DST,710}};
    bool cancelled=false;
    size_t calls=0;
    auto negative=[&]{++calls;return false;};
    auto cancel=[&]{return cancelled;};
    auto first=cache.prove(key,releases,negative,cancel);
    require(!first.possible && !first.reused && calls==1,"first negative cut proof was not executed");
    auto duplicate=std::vector<fpga::RouteCutNode>{releases[1],releases[0],releases[1]};
    // Reordered/duplicated release records identify exactly the same set.
    auto repeated=cache.prove(key,duplicate,negative,cancel);
    require(!repeated.possible && repeated.reused && calls==1,"equivalent cut repeated its failed proof");
    auto miss=[&](auto changed,auto cuts) {
        auto before=calls;
        auto result=cache.prove(changed,cuts,negative,cancel);
        require(!result.possible && !result.reused && calls==before+1,"distinct input/cut identity was aliased");
    };
    // Keep physical pins, source placement, anchor, bounds and typed release
    // nodes separate; sharing an integer node ID alone is never sufficient.
    auto changed=key;changed.pins.setBit(64);miss(changed,releases);
    changed=key;changed.source_pos++;miss(changed,releases);
    changed=key;changed.source_hint.x++;miss(changed,releases);
    changed=key;changed.anchor=&f.tile({4,1});miss(changed,releases);
    changed=key;changed.dst++;miss(changed,releases);
    changed=key;changed.radius++;miss(changed,releases);
    changed=key;changed.depth++;miss(changed,releases);
    auto other_release=releases;other_release[0].type=fpga::CB_NODE_JOINT;
    miss(key,other_release);
    // An earlier input's proven cut adds resources to the later input's
    // private view. Include that union, rather than reusing a negative result
    // which assumed the earlier owner was still present.
    auto combined=releases;combined.push_back({{5,1},fpga::CB_NODE_DST,711});
    auto combination=cache.prove(key,combined,[]{return true;},cancel);
    require(combination.possible && !combination.reused,"earlier input release was omitted from proof identity");

    changed=key;changed.radius=30;
    auto interrupted=cache.prove(changed,releases,[&]{cancelled=true;return false;},cancel);
    require(!interrupted.reused,"cancelled proof unexpectedly came from cache");
    cancelled=false;
    // A cancelled negative search cannot suppress a later successful proof.
    auto retry=cache.prove(changed,releases,[]{return true;},cancel);
    require(retry.possible && !retry.reused,"cancelled cut proof was remembered as failure");
    auto positive=cache.prove(changed,releases,negative,cancel);
    require(positive.possible && positive.reused,"independent positive condition was not reusable");

    // Invalidate with a real unroute, not a simulated generation variable.
    fpga::unrouteNetRoute(*f.enable,0);
    auto revised=cache.prove(key,releases,[]{return true;},cancel);
    require(revised.possible && !revised.reused,"live route mutation retained a stale negative proof");
    f.auditAll();

    // Each input/driver owns a separate cache, and bounded eviction must
    // cause recomputation rather than rejecting the discarded candidates.
    pnr::MovingInputCutCache other_driver;
    require(other_driver.prove(key,releases,[]{return true;},cancel).possible,
        "failed proof leaked between driver-scoped caches");
    pnr::MovingInputCutCache bounded(1,2);
    bounded.prove(key,releases,negative,cancel);
    bounded.prove(changed,releases,negative,cancel);
    auto evicted=bounded.prove(key,releases,[]{return true;},cancel);
    require(evicted.possible && !evicted.reused,"evicted proof suppressed a search");
}
