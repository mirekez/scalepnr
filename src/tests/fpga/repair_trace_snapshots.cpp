#include "SourceInputFixture.h"

// A converging numeric search repeatedly visits unchanged crossbars. Native
// stagnation logs emitted millions of duplicate snapshot references, taking
// most sampled CPU time even though every edge already identified its tiles.
void repair_trace_snapshots_regression() {
  using namespace routing_tests;
  SourceInputFixture f(false);
  for(int a=0;a<64;++a) {
    f.edge({7,0},{3,2},141,1000+a,2000+a,true);
    for(int b=0;b<64;++b) f.edge({3,2},{2,2},2000+a,3000+b,4000+b);
  }
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  rtl::NetRouteBinding binding;
  binding.from=f.source;binding.to=f.sink;binding.from_port="Q";binding.to_port="I0";
  binding.route_name="converging_signal";
  auto run=[&] {
    return fpga::routeUnboundNetFromSource(*f.signal,binding,
        f.pin({7,0},141,true),f.pin({0,0},87,false),{},32768,200);
  };
  // Enabling diagnostics must not change exhaustion or allocate any fragments.
  require(!run(),"disconnected reference unexpectedly completed");
  f.auditAll();
  auto count=[](const std::string& text,const std::string& token) {
    size_t n=0,pos=0;
    while((pos=text.find(token,pos))!=std::string::npos){++n;pos+=token.size();}
    return n;
  };
  std::ostringstream out;
  size_t decisions=0,snapshots=0,refs=0;
  {
    pnr::RouteCongestionTrace trace("converging_signal","test",{f.signal,f.enable},&out);
    require(!run(),"traced disconnected reference unexpectedly completed");
    const auto text=out.str();
    decisions=count(text,"REPAIR_DECISION");snapshots=count(text,"SNAPSHOT_BEGIN");
    refs=count(text,"SNAPSHOT_REF");
    // Publishing identical masks can still replace their owner. A cached
    // snapshot must be refreshed on revision changes, not just mask changes.
    const auto revision=fpga::routeMutationRevision();
    fpga::registerNetRouteTilesFrom(*f.enable,f.source->wires[0],0,0);
    require(fpga::routeMutationRevision()!=revision,"fixture failed to publish ownership change");
    trace.snapshot(f.tile({7,0}));
    require(count(out.str(),"SNAPSHOT_BEGIN")==snapshots+1,"ownership revision retained stale snapshot");
    // Persistent marks also belong in the snapshot, even without lease edits.
    f.tile({7,0}).cb.src_deadend.jump.setBit(1000);
    trace.snapshot(f.tile({7,0}));
    require(count(out.str(),"SNAPSHOT_BEGIN")==snapshots+2,"changed deadend mask was hidden");
    f.tile({7,0}).cb.src_deadend.jump.clearBit(1000);
  }
  f.auditAll();
  std::cout<<"numeric trace decisions="<<decisions<<" snapshots="<<snapshots<<" duplicate_refs="<<refs<<'\n';
  // Keep the complete decision trace, with one state record per touched tile.
  require(decisions>=4096,"fixture did not exercise converging numeric edges");
  require(snapshots<=4 && refs==0,"unchanged numeric snapshots flood the decision trace");
  // Deadline traces defer the expensive owner audit. They still must notice
  // publication changes while the live masks happen to remain identical.
  std::ostringstream deferred;
  {
    pnr::RouteCongestionTrace trace("deferred_signal","test",{f.signal,f.enable},&deferred,{},true);
    trace.snapshot(f.tile({7,0}));
    trace.snapshot(f.tile({7,0}));
    require(count(deferred.str(),"SNAPSHOT_BEGIN")==1 &&
        count(deferred.str(),"SNAPSHOT_REF")==0,"deferred trace repeats unchanged state");
    fpga::registerNetRouteTilesFrom(*f.enable,f.source->wires[0],0,0);
    trace.snapshot(f.tile({7,0}));
    require(count(deferred.str(),"SNAPSHOT_BEGIN")==2,"deferred trace missed ownership publication");
  }
  f.auditAll();
}
