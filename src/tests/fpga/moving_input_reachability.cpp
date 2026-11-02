#include "SourceInputFixture.h"
#include "route/MovingInputReachability.h"

// Exercise the same read-only, existential predicate used by Source Moving.
// In particular, reaching a source CB and exhausting a budget are UNKNOWN:
// neither may falsely blacklist a placement or commit any physical route.
void moving_input_reachability_regression() {
    routing_tests::SourceInputFixture f(false);
    auto& router=f.tech.route;
    auto index=router.backwardDockingIndex({2,0},16);
    NodeMask pins; pins.setBit(63);
    const std::vector<fpga::Tile*> goals{&f.tile({6,0})};
    pnr::MovingInputReachability region(f.tile({2,0}));
    routing_tests::require(region.mayReach(pins,goals,index,nullptr,[]{return true;}) &&
                          region.expanded()==0,
                          "cancelled input region became a negative proof");
    routing_tests::require(!region.mayReach(pins,goals,index),
                          "disconnected CE fan-in reached the fixed source CB");
    const size_t examined=region.expanded();
    routing_tests::require(examined>0 && !region.mayReach(pins,goals,index) &&
                          region.expanded()==examined,
                          "unchanged negative input region was searched again");
    routing_tests::require(!region.released({&f.tile({7,2})}) &&
                          !region.mayReach(pins,goals,index) && region.expanded()==examined,
                          "unrelated release restarted the input region");
    // A newly provided retained-prefix CB may already have been reached.
    routing_tests::require(region.mayReach(pins,{&f.tile({3,0})},index),
                          "new reachable prefix CB retained an old negative result");
    pnr::MovingInputReachability bounded(f.tile({2,0}),1);
    routing_tests::require(bounded.mayReach(pins,goals,index) && bounded.uncertain(),
                          "bounded input region became an impossibility proof");
    f.auditAll();
}
