#include "OutlineDesign.h"
#include "Tech.h"

#include <algorithm>
#include <cmath>
#include <array>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "outline_attractor_test: " << message << '\n';
        std::exit(1);
    }
}

bool near(float left, float right)
{
    return std::abs(left - right) < 0.00001F;
}

struct Fixture
{
    technology::Tech tech;
    pnr::OutlineDesign outline;
    Referable<rtl::Module> primitives;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;

    Fixture()
    {
        technology::Tech::clocked_ports.clear();
        technology::Tech::buffers_ports.clear();
        technology::Tech::clocked_ports.emplace("REG", "C");
        technology::Tech::buffers_ports.emplace("IBUF", "O");
        technology::Tech::buffers_ports.emplace("OBUF", "O");

        primitives.name = "outline_attractor_primitives";
        primitives.is_blackbox = true;
        outline.tech = &tech;
        outline.fpga_width = 100;
        outline.fpga_height = 100;
        outline.aspect_x = 10;
        outline.aspect_y = 10;
        outline.step_x = 0.05F;
        outline.step_y = 0.05F;
        outline.boxes1.resize(static_cast<size_t>(
            outline.fpga_width*outline.fpga_height));
    }

    rtl::Inst* makeInst(const std::string& name, const std::string& type,
                        Referable<pnr::RegBunch>& bunch, float x)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name;
        cell->type = type;
        cell->module_ref.set(&primitives);

        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(cell.get());
        inst->cnt_inputs = 0;
        inst->cnt_outputs = 0;
        inst->pos = -1;
        inst->outline.x = x;
        inst->outline.y = bunch.y;
        inst->bunch_ref.set(&bunch);

        rtl::Inst* result = inst.get();
        cells.push_back(std::move(cell));
        insts.push_back(std::move(inst));
        return result;
    }

    void connect(rtl::Inst& left, rtl::Inst& right)
    {
        if (!outline.optimization_peers.contains(&left)) {
            outline.optimization_order.push_back(&left);
        }
        if (!outline.optimization_peers.contains(&right)) {
            outline.optimization_order.push_back(&right);
        }
        outline.optimization_peers[&left].push_back(&right);
        outline.optimization_peers[&right].push_back(&left);
        outline.optimization_sinks[&left].push_back(&right);
        outline.optimization_drivers[&right].push_back(&left);
    }

    void run(rtl::Inst& root, Referable<pnr::RegBunch>& bunch)
    {
        (void)root;
        (void)bunch;
        std::fill(outline.boxes1.begin(), outline.boxes1.end(), 0);
        outline.moveTimingAttractorsSimultaneously();
    }
};

void checkRegisterGravity(Fixture& fixture)
{
    Referable<pnr::RegBunch> bunch;
    bunch.x = 5;
    bunch.y = 5;
    rtl::Inst* reg = fixture.makeInst("register_star", "REG", bunch, 5.40F);
    rtl::Inst* target = fixture.makeInst(
        "register_attraction_target", "LUT6", bunch, 5.00F);
    rtl::Inst* lut = fixture.makeInst(
        "register_follower_lut", "LUT6", bunch, 5.30F);
    rtl::Inst* carry0 = fixture.makeInst(
        "register_carry_0", "CARRY", bunch, 5.20F);
    rtl::Inst* carry1 = fixture.makeInst(
        "register_carry_1", "CARRY", bunch, 5.10F);
    bunch.reg = reg;
    fixture.connect(*reg, *target);
    fixture.connect(*reg, *lut);
    fixture.connect(*lut, *carry0);
    fixture.connect(*carry0, *carry1);

    float register_before = reg->outline.x;
    float lut_before = lut->outline.x;
    float carry0_before = carry0->outline.x;
    float carry1_before = carry1->outline.x;
    fixture.run(*reg, bunch);

    require(fixture.outline.isGravityAttractor(*reg),
            "a clocked register was not classified as an attractor");
    require(!fixture.outline.isGravityAttractor(*target)
                && !fixture.outline.isGravityAttractor(*lut),
            "a LUT was incorrectly classified as an attractor");
    require(!fixture.outline.isGravityAttractor(*carry0),
            "a carry cell was incorrectly classified as an attractor");
    require(reg->outline.x < register_before,
            "the clocked register did not respond to gravity");
    require(lut->outline.x > lut_before
                && carry0->outline.x > carry0_before
                && carry1->outline.x > carry1_before,
            "followers copied a leftward translation instead of correcting toward their register");
    // Different initial offsets require different distances. It is the
    // fraction of each offset corrected that must fade along the chain.
    float lut_fraction = (lut->outline.x-lut_before)/(reg->outline.x-lut_before);
    float carry0_fraction = (carry0->outline.x-carry0_before)/(reg->outline.x-carry0_before);
    float carry1_fraction = (carry1->outline.x-carry1_before)/(reg->outline.x-carry1_before);
    require(near(lut_fraction, 0.05F) && near(carry0_fraction, 0.025F)
                && near(carry1_fraction, 0.0125F),
            "relative-position correction did not fade along the combinational chain");
}

void checkIoGravity(Fixture& fixture)
{
    Referable<pnr::RegBunch> bunch;
    bunch.x = 5;
    bunch.y = 5;
    rtl::Inst* iob = fixture.makeInst("input_star", "IBUF", bunch, 4.60F);
    rtl::Inst* lut = fixture.makeInst("input_lut", "LUT6", bunch, 5.00F);
    rtl::Inst* carry0 = fixture.makeInst(
        "input_carry_0", "CARRY", bunch, 5.10F);
    rtl::Inst* carry1 = fixture.makeInst(
        "input_carry_1", "CARRY", bunch, 5.20F);
    bunch.reg = iob;
    require(fixture.outline.isGravityAttractor(*iob),
            "a configured I/O boundary was not classified as an attractor");
    bunch.fixed = true;
    iob->outline.fixed = true;
    fixture.connect(*iob, *lut);
    fixture.connect(*lut, *carry0);
    fixture.connect(*carry0, *carry1);
    float iob_before = iob->outline.x;
    float lut_before = lut->outline.x;
    float carry0_before = carry0->outline.x;
    float carry1_before = carry1->outline.x;
    fixture.run(*iob, bunch);

    require(near(iob->outline.x, iob_before),
            "fixed I/O gravity moved the package anchor");
    require(lut->outline.x < lut_before,
            "the LUT did not move toward its I/O star");
    require(carry0->outline.x < carry0_before
                && carry1->outline.x < carry1_before,
            "I/O displacement did not propagate into the carry chain");
    require(lut_before - lut->outline.x
                > carry0_before - carry0->outline.x
                && carry0_before - carry0->outline.x
                    > carry1_before - carry1->outline.x,
            "I/O displacement did not decay along the carry chain");
}

void checkCombinationalCellsHaveNoGravity(Fixture& fixture)
{
    Referable<pnr::RegBunch> bunch;
    bunch.x = 5;
    bunch.y = 5;
    rtl::Inst* lut = fixture.makeInst("free_lut", "LUT6", bunch, 4.80F);
    rtl::Inst* carry = fixture.makeInst("free_carry", "CARRY", bunch, 5.20F);
    bunch.reg = lut;
    fixture.connect(*lut, *carry);

    float lut_before = lut->outline.x;
    float carry_before = carry->outline.x;
    fixture.run(*lut, bunch);

    require(!fixture.outline.isGravityAttractor(*lut)
                && !fixture.outline.isGravityAttractor(*carry),
            "a combinational cell received gravity classification");
    require(near(lut->outline.x, lut_before)
                && near(carry->outline.x, carry_before),
            "an unanchored combinational chain started its own attraction");
}

void checkTimingDeficitControlsAcceleration(Fixture& fixture)
{
    Referable<pnr::RegBunch> baseline_bunch;
    baseline_bunch.x = 5;
    baseline_bunch.y = 3;
    rtl::Inst* baseline_reg = fixture.makeInst(
        "baseline_register", "REG", baseline_bunch, 5.40F);
    rtl::Inst* baseline_target = fixture.makeInst(
        "baseline_target", "LUT6", baseline_bunch, 5.00F);
    baseline_bunch.reg = baseline_reg;
    fixture.connect(*baseline_reg, *baseline_target);

    Referable<pnr::RegBunch> critical_bunch;
    critical_bunch.x = 5;
    critical_bunch.y = 7;
    rtl::Inst* critical_reg = fixture.makeInst(
        "critical_register", "REG", critical_bunch, 5.40F);
    rtl::Inst* critical_target = fixture.makeInst(
        "critical_target", "LUT6", critical_bunch, 5.00F);
    critical_bunch.reg = critical_reg;
    fixture.connect(*critical_reg, *critical_target);
    fixture.tech.place.place_timing
        .placement_net_weights[critical_reg][critical_target] = 4.0;

    float baseline_before = baseline_reg->outline.x;
    float critical_before = critical_reg->outline.x;
    fixture.run(*baseline_reg, baseline_bunch);
    float baseline_distance = baseline_before - baseline_reg->outline.x;
    float critical_distance = critical_before - critical_reg->outline.x;
    require(critical_distance > baseline_distance*1.5F,
            "timing deficit did not increase register acceleration");
}

void checkUnequalDistancesDoNotCancel(Fixture& fixture)
{
    Referable<pnr::RegBunch> bunch;
    bunch.x = 5;
    bunch.y = 1;
    rtl::Inst* reg = fixture.makeInst(
        "distance_sensitive_register", "REG", bunch, 5.00F);
    rtl::Inst* near_left = fixture.makeInst(
        "distance_near_left", "LUT6", bunch, 4.90F);
    rtl::Inst* far_right = fixture.makeInst(
        "distance_far_right", "LUT6", bunch, 6.00F);
    bunch.reg = reg;
    fixture.connect(*reg, *near_left);
    fixture.connect(*reg, *far_right);

    float before = reg->outline.x;
    fixture.run(*reg, bunch);
    require(reg->outline.x > before,
            "equal opposite signs hid the longer timing connection");
}

void checkPropagationIsBounded(Fixture& fixture)
{
    Referable<pnr::RegBunch> bunch;
    bunch.x = 5;
    bunch.y = 9;
    rtl::Inst* iob = fixture.makeInst(
        "bounded_input", "IBUF", bunch, 4.50F);
    iob->outline.fixed = true;
    bunch.fixed = true;
    bunch.reg = iob;
    std::vector<rtl::Inst*> chain;
    for (int index = 0; index < 6; ++index) {
        chain.push_back(fixture.makeInst(
            "bounded_follower_" + std::to_string(index), "LUT6", bunch,
            5.00F + 0.05F*index));
    }
    fixture.connect(*iob, *chain.front());
    for (size_t index = 1; index < chain.size(); ++index) {
        fixture.connect(*chain[index - 1], *chain[index]);
    }
    float final_before = chain.back()->outline.x;
    fixture.run(*iob, bunch);
    require(chain[4]->outline.x < 5.00F + 0.05F*4,
            "four-hop follower did not receive faded movement");
    require(near(chain.back()->outline.x, final_before),
            "movement escaped beyond the bounded secondary neighborhood");
}

void checkTensionCrossesRegisterTiers(Fixture& fixture)
{
    Referable<pnr::RegBunch> bunch;
    bunch.x = 5;
    bunch.y = 5;
    rtl::Inst* iob = fixture.makeInst(
        "tiered_input", "IBUF", bunch, 4.00F);
    rtl::Inst* lut0 = fixture.makeInst(
        "tiered_lut_0", "LUT6", bunch, 5.00F);
    rtl::Inst* reg0 = fixture.makeInst(
        "tiered_reg_0", "REG", bunch, 6.00F);
    rtl::Inst* lut1 = fixture.makeInst(
        "tiered_lut_1", "LUT6", bunch, 7.00F);
    rtl::Inst* reg1 = fixture.makeInst(
        "tiered_reg_1", "REG", bunch, 8.00F);
    rtl::Inst* lut2 = fixture.makeInst(
        "tiered_lut_2", "LUT6", bunch, 9.00F);
    iob->outline.fixed = true;
    bunch.fixed = true;
    bunch.reg = iob;
    fixture.connect(*iob, *lut0);
    fixture.connect(*lut0, *reg0);
    fixture.connect(*reg0, *lut1);
    fixture.connect(*lut1, *reg1);
    fixture.connect(*reg1, *lut2);

    float reg0_before = reg0->outline.x;
    float reg1_before = reg1->outline.x;
    fixture.run(*iob, bunch);
    require(near(reg0->outline.x,reg0_before) && near(reg1->outline.x,reg1_before),
            "I/O force dragged a register through its combinational neighbor in the same pass");
    // Fixed-anchor motion reaches only its combinational constellation in one
    // pass.  Subsequent registers then originate their own force from the
    // changed neighbor positions, relaying tension without being dragged.
    for (int iteration = 1; iteration < 30; ++iteration) {
        fixture.run(*iob, bunch);
    }
    float first_tier = reg0_before - reg0->outline.x;
    float second_tier = reg1_before - reg1->outline.x;
    // Relative relaxation also corrects the terminal LUT's initial offset;
    // absolute displacement need not decrease with tier number.
    require(first_tier > 0 && second_tier > 0,
            "fixed tension stopped at the first register tier");
}

void checkChainRelaxesBetweenOppositeAnchors(Fixture& fixture)
{
    Referable<pnr::RegBunch> bunch;
    bunch.x = 5;
    bunch.y = 5;
    rtl::Inst* left = fixture.makeInst(
        "taut_left_input", "IBUF", bunch, 0.05F);
    rtl::Inst* lut0 = fixture.makeInst(
        "taut_lut_0", "LUT6", bunch, 5.00F);
    rtl::Inst* reg0 = fixture.makeInst(
        "taut_reg_0", "REG", bunch, 5.00F);
    rtl::Inst* lut1 = fixture.makeInst(
        "taut_lut_1", "LUT6", bunch, 5.00F);
    rtl::Inst* reg1 = fixture.makeInst(
        "taut_reg_1", "REG", bunch, 5.00F);
    rtl::Inst* lut2 = fixture.makeInst(
        "taut_lut_2", "LUT6", bunch, 5.00F);
    rtl::Inst* right = fixture.makeInst(
        "taut_right_output", "OBUF", bunch, 9.95F);
    left->outline.fixed = true;
    right->outline.fixed = true;
    bunch.fixed = true;
    bunch.reg = left;
    fixture.connect(*left, *lut0);
    fixture.connect(*lut0, *reg0);
    fixture.connect(*reg0, *lut1);
    fixture.connect(*lut1, *reg1);
    fixture.connect(*reg1, *lut2);
    fixture.connect(*lut2, *right);

    for (int iteration = 0; iteration < 100; ++iteration) {
        fixture.run(*left, bunch);
    }
    require(reg0->outline.x > 1.0F && reg0->outline.x < 5.0F
                && reg1->outline.x > 5.0F && reg1->outline.x < 9.0F,
            "register chain did not relax through the space between anchors");
}

void checkFixedAnchorsSeedTautRadialAllocation(Fixture& fixture)
{
    Referable<pnr::RegBunch> root;
    root.x = 9.95F;
    root.y = 7.50F;
    root.fixed = true;
    root.reg = fixture.makeInst("fixed_output_root", "OBUF", root, root.x);
    auto& child = root.sub_bunches.emplace_back();
    child.parent = &root;
    child.reg = fixture.makeInst("fixed_output_child", "REG", child, 0.0F);
    auto& input = child.sub_bunches.emplace_back();
    input.x = 0.05F;
    input.y = 7.50F;
    input.fixed = true;
    input.parent = &child;
    input.reg = fixture.makeInst("fixed_input_leaf", "IBUF", input, input.x);

    fixture.outline.uniform_unanchored_allocation = false;
    fixture.outline.radial_anchor_guides.clear();
    fixture.outline.prepareRadialAnchorGuides(root);
    fixture.outline.recurseRadialAllocation(root, 0, 0);

    require(near(child.x, 5.00F) && near(child.y, 7.50F),
            "branch was not stretched between its fixed I/O anchors");
}

void checkBlockedRootStillCorrectsFollowers(bool partial, bool vertical)
{
    Fixture fixture;
    Referable<pnr::RegBunch> root_bunch, follower_bunch;
    root_bunch.x=root_bunch.y=5;
    follower_bunch.x=follower_bunch.y=5;
    follower_bunch.fixed=true; // A movable LUT in a fixed-I/O bunch, as in the puzzle.
    float start=partial ? 5.49F : 5.50F;
    auto* reg=fixture.makeInst("bounded_register","REG",root_bunch,start);
    auto* lut=fixture.makeInst("bounded_lut","LUT6",follower_bunch,7);
    auto* carry=fixture.makeInst("bounded_carry","CARRY",follower_bunch,7.1F);
    root_bunch.reg=reg; follower_bunch.reg=lut;
    fixture.connect(*reg,*lut); fixture.connect(*lut,*carry);
    if (vertical) for(auto* inst : {reg,lut,carry}) std::swap(inst->outline.x,inst->outline.y);
    auto coordinate=[&](rtl::Inst* inst){return vertical ? inst->outline.y : inst->outline.x;};
    float before_lut=coordinate(lut),before_carry=coordinate(carry);
    size_t moved=fixture.outline.moveTimingAttractorsSimultaneously();
    float follower=coordinate(lut)-before_lut;
    require(near(follower,(5.5F-before_lut)*0.05F)
        && near(coordinate(carry)-before_carry,(5.5F-before_carry)*0.025F),
        "blocked root did not correct followers toward its feasible position");
    require(moved==(partial ? 3U : 2U),"moved counter included unchanged coordinates");
    require(near(coordinate(reg),5.5F),"register escaped its bunch window");
}

void checkBlockedRegistersCannotOverpowerFixedAnchor()
{
    Fixture fixture;
    Referable<pnr::RegBunch> reg_bunch, io_bunch;
    reg_bunch.x=5; reg_bunch.y=1.5F;
    io_bunch.x=5; io_bunch.y=1.9F; io_bunch.fixed=true;
    auto* io=fixture.makeInst("runaway_output","OBUF",io_bunch,5);
    auto* lut=fixture.makeInst("runaway_lut","LUT6",io_bunch,5);
    io->outline.fixed=true; lut->outline.y=8;
    io_bunch.reg=io;
    fixture.connect(*io,*lut);
    std::vector<rtl::Inst*> registers;
    for(int i=0;i<5;++i) {
        auto* reg=fixture.makeInst("runaway_reg_"+std::to_string(i),"REG",reg_bunch,5);
        reg->outline.y=2; registers.push_back(reg); fixture.connect(*reg,*lut);
    }
    reg_bunch.reg=registers.front();
    float before=lut->outline.y;
    fixture.outline.moveTimingAttractorsSimultaneously();
    for(auto* reg:registers) require(near(reg->outline.y,2),"blocked register moved beyond its window");
    require(lut->outline.y<before,
        "COMB detour survived despite all its roots lying on the other side");
    require(near(io->outline.y,1.9F),"repair displaced a fixed I/O anchor");
}

void checkPartiallyBlockedDiagonalMotion()
{
    Fixture fixture;
    Referable<pnr::RegBunch> root_bunch, follower_bunch;
    root_bunch.x = root_bunch.y = 5;
    follower_bunch.x = follower_bunch.y = 7;
    follower_bunch.fixed = true;
    auto* reg = fixture.makeInst("diagonal_register", "REG", root_bunch, 5.5F);
    auto* lut = fixture.makeInst("diagonal_lut", "LUT6", follower_bunch, 7);
    root_bunch.reg = reg;
    follower_bunch.reg = lut;
    fixture.connect(*reg, *lut);
    fixture.outline.moveTimingAttractorsSimultaneously();
    require(near(reg->outline.x, 5.5F) && lut->outline.x < 7,
            "blocked root axis prevented correcting the follower's offset");
    require(reg->outline.y > 5 && lut->outline.y < 7,
            "diagonal root and follower did not approach each other");
}

void checkRegisterLutRegisterDetour(bool vertical, bool reverse_order)
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch;
    bunch.x=bunch.y=4;
    bunch.fixed=true; // No window clipping: isolate the translation-only bug.
    auto* a=fixture.makeInst("detour_launch","REG",bunch,4);
    auto* lut=fixture.makeInst("detour_logic","LUT6",bunch,5);
    auto* b=fixture.makeInst("detour_capture","REG",bunch,4);
    fixture.connect(*a,*lut); fixture.connect(*lut,*b);
    if(vertical) for(auto* inst : {a,lut,b}) std::swap(inst->outline.x,inst->outline.y);
    if(reverse_order) std::reverse(fixture.outline.optimization_order.begin(),
                                  fixture.outline.optimization_order.end());
    auto coordinate=[&](rtl::Inst* inst){return vertical ? inst->outline.y : inst->outline.x;};
    const float initial_length=2;
    for(int iteration=0;iteration<20;++iteration) {
        const float before=std::abs(coordinate(a)-coordinate(lut))
                          +std::abs(coordinate(b)-coordinate(lut));
        fixture.outline.moveTimingAttractorsSimultaneously();
        const float after=std::abs(coordinate(a)-coordinate(lut))
                         +std::abs(coordinate(b)-coordinate(lut));
        if (iteration == 0)
            require(near(coordinate(lut), 4.95375F),
                    "direct REG-COMB-REG correction changed without a remote root");
        require(after<before,"equal register translations preserved the LUT detour");
        require(near(coordinate(a),coordinate(b)),"simultaneous roots saw different snapshots");
    }
    require(2*std::abs(coordinate(lut)-coordinate(a))<initial_length*0.1F,
            "register-LUT-register path failed to contract by 90 percent");
}

void checkBalancedStationaryRoot()
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch;
    bunch.x=bunch.y=5; bunch.fixed=true;
    auto* reg=fixture.makeInst("balanced_reg","REG",bunch,5);
    auto* left=fixture.makeInst("balanced_left","LUT6",bunch,4);
    auto* right=fixture.makeInst("balanced_right","LUT6",bunch,6);
    auto* carry=fixture.makeInst("balanced_carry","CARRY",bunch,7);
    fixture.connect(*reg,*left); fixture.connect(*reg,*right); fixture.connect(*right,*carry);
    fixture.outline.moveTimingAttractorsSimultaneously();
    require(near(reg->outline.x,5),"balanced register force did not cancel");
    require(left->outline.x>4 && right->outline.x<6 && carry->outline.x<7,
            "zero root translation suppressed relative COMB correction");
}

void checkEdgeLutReturnsTowardBlockedRegisters()
{
    Fixture fixture;
    Referable<pnr::RegBunch> reg_bunch, io_bunch;
    reg_bunch.x=5; reg_bunch.y=6;
    io_bunch.x=5; io_bunch.y=9.95F; io_bunch.fixed=true;
    auto* io=fixture.makeInst("edge_output","OBUF",io_bunch,5);
    auto* lut=fixture.makeInst("edge_detour","LUT6",io_bunch,5);
    io->outline.fixed=true;
    fixture.connect(*io,*lut);
    std::vector<rtl::Inst*> roots;
    for(int i=0;i<5;++i) {
        auto* reg=fixture.makeInst("edge_reg_"+std::to_string(i),"REG",reg_bunch,5);
        reg->outline.y=6.5F; // At the window boundary, unable to follow the LUT outward.
        roots.push_back(reg);
        fixture.connect(*reg,*lut);
        fixture.tech.place.place_timing.placement_net_weights[reg][lut]=1;
    }
    const float before=lut->outline.y;
    for(int iteration=0;iteration<60;++iteration)
        fixture.outline.moveTimingAttractorsSimultaneously();
    require(lut->outline.y<7.2F && before-lut->outline.y>2.7F,
            "fixed output kept a shared LUT stranded at the edge despite five inland registers");
    require(near(io->outline.y,9.95F),"relative correction displaced the fixed output");
    for(auto* reg:roots) require(near(reg->outline.y,6.5F),"register escaped its bunch window");
}

void checkSharedCombinationalBunchLinks()
{
    Fixture fixture;
    std::list<Referable<pnr::RegBunch>> bunches;
    auto& io_bunch = bunches.emplace_back();
    auto& reg_bunch = bunches.emplace_back();
    auto& source_bunch = bunches.emplace_back();
    io_bunch.x = reg_bunch.x = source_bunch.x = 5;
    io_bunch.y = 9;
    reg_bunch.y = source_bunch.y = 1;
    io_bunch.fixed = true;
    auto* io = fixture.makeInst("shared_output", "OBUF", io_bunch, 5);
    auto* lut = fixture.makeInst("shared_logic", "LUT6", io_bunch, 5);
    auto* reg = fixture.makeInst("shared_consumer", "REG", reg_bunch, 5);
    auto* source = fixture.makeInst("shared_source", "REG", source_bunch, 5);
    io->outline.fixed = true;
    io_bunch.reg = io; reg_bunch.reg = reg; source_bunch.reg = source;
    for (auto* inst : {io, lut, reg, source}) {
        inst->cell_ref->ports.reserve(4);
        inst->conns.reserve(4);
    }
    auto port = [](rtl::Inst* inst, const char* name, bool output) {
        auto& definition = inst->cell_ref->ports.emplace_back();
        definition.name = name;
        definition.type = output ? rtl::Port::PORT_OUT : rtl::Port::PORT_IN;
        auto& connection = inst->conns.emplace_back();
        connection.port_ref.set(&definition);
        connection.inst_ref.set(static_cast<Referable<rtl::Inst>*>(inst));
        return &connection;
    };
    auto* source_out = port(source, "Q", true);
    auto* lut_out = port(lut, "O", true);
    port(lut, "I", false)->set(source_out);
    port(io, "I", false)->set(lut_out);
    port(reg, "D", false)->set(lut_out);
    // Another pin on the same consumer must not double-count the bunch pair.
    port(reg, "E", false)->set(lut_out);
    port(reg, "C", false)->set(source_out);
    // Even a combinationally driven clock pin must not become a data spring.
    port(source, "C", false)->set(lut_out);
    // Estimate owns this shared LUT in the I/O bunch. Its existing upstream
    // register link is retained; the consumer-to-LUT owner link is missing.
    io_bunch.uplinks.push_back(pnr::BunchLink{.conn=source_out});
    require(fixture.outline.prepareSharedCombLinks(bunches) == 1,
            "missing shared-COMB connection not found or counted twice");
    require(fixture.outline.shared_comb_links.at(&reg_bunch).front() == &io_bunch,
            "shared-COMB connection points to its upstream register, not its owner");
    float before = reg_bunch.y;
    fixture.outline.recurseSecondaryLinks(reg_bunch);
    require(reg_bunch.y > before && near(io_bunch.y, 9),
            "shared-LUT consumer did not move toward its fixed owning bunch");
    require(reg->bunch_ref.peer == &reg_bunch && lut->bunch_ref.peer == &io_bunch,
            "Outline changed shared logic ownership");
    require(fixture.outline.prepareSharedCombLinks(bunches) == 1,
            "repeated preparation retained stale shared-COMB links");
    reg_bunch.uplinks.push_back(pnr::BunchLink{.conn=lut_out});
    require(fixture.outline.prepareSharedCombLinks(bunches) == 0,
            "an already represented bunch connection was added again");
}

void checkFixedOutputTraversal()
{
    Fixture fixture;
    Referable<pnr::RegBunch> output_bunch;
    auto& register_bunch = output_bunch.sub_bunches.emplace_back();
    auto& input_bunch = register_bunch.sub_bunches.emplace_back();
    for (auto* bunch : {&output_bunch, &register_bunch, &input_bunch}) {
        bunch->x = 5;
        bunch->y = 5;
    }
    auto* output = fixture.makeInst("fixed_output", "OBUF", output_bunch, 5);
    auto* reg = fixture.makeInst("output_register", "REG", register_bunch, 5);
    auto* input = fixture.makeInst("upstream_input", "IBUF", input_bunch, 5);
    output_bunch.reg = output;
    register_bunch.reg = reg;
    input_bunch.reg = input;
    output->outline.fixed = true;
    input->outline.fixed = true;
    auto port = [](rtl::Inst* inst, const char* name, bool out) {
        auto& definition = inst->cell_ref->ports.emplace_back();
        definition.name = name;
        definition.type = out ? rtl::Port::PORT_OUT : rtl::Port::PORT_IN;
        auto& connection = inst->conns.emplace_back();
        connection.port_ref.set(&definition);
        connection.inst_ref.set(static_cast<Referable<rtl::Inst>*>(inst));
        return &connection;
    };
    for (auto* inst : {output, reg, input}) {
        inst->cell_ref->ports.reserve(2);
        inst->conns.reserve(2);
    }
    auto* input_out = port(input, "O", true);
    auto* reg_out = port(reg, "Q", true);
    port(reg, "D", false)->set(input_out);
    port(output, "I", false)->set(reg_out);
    fixture.outline.travers_mark = rtl::Inst::genMark();
    fixture.outline.recurseInstPrepare(*output, &output_bunch);
    require(fixture.outline.optimization_edges.size() == 2,
            "fixed output traversal lost the register's upstream bunch");
    require(input->mark == fixture.outline.travers_mark,
            "fixed output traversal did not visit the input child bunch");
    // Force density pressure in each of the eight smear directions. Fixed
    // cells still participate in traversal and occupancy, but never move.
    for (int iteration = 0; iteration < 16; ++iteration) {
        std::fill(fixture.outline.boxes1.begin(), fixture.outline.boxes1.end(), 0);
        fixture.outline.boxes1[50*100+50] = 3;
        fixture.outline.travers_mark = rtl::Inst::genMark();
        fixture.outline.recurseOptimizeInsts(*output, &output_bunch, iteration);
        require(near(output->outline.x, 5) && near(output->outline.y, 5)
                    && near(input->outline.x, 5) && near(input->outline.y, 5),
                "density optimization displaced a fixed I/O");
        require(input->mark == fixture.outline.travers_mark,
                "fixed-cell movement guard skipped descendant traversal");
    }
}

void checkBunchWindowFollowsRegister()
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch, anchor_bunch;
    bunch.x = 2; bunch.y = 5;
    anchor_bunch.x = 8; anchor_bunch.y = 5;
    anchor_bunch.fixed = true;
    auto* reg = fixture.makeInst("traveling_register", "REG", bunch, 2);
    auto* output = fixture.makeInst("distant_output", "OBUF", anchor_bunch, 8);
    bunch.reg = reg; anchor_bunch.reg = output;
    output->outline.fixed = true;
    fixture.connect(*reg, *output);
    for (int i = 0; i < 100; ++i) {
        float before = reg->outline.x;
        fixture.outline.moveTimingAttractorsSimultaneously();
        require(reg->outline.x >= before && reg->outline.x-before <= 0.07501F,
                "following the bunch root broke the per-step movement bound");
        fixture.outline.followBunchRoots();
    }
    require(reg->outline.x > 7.9F && near(bunch.x, reg->outline.x),
            "register remained trapped against its original bunch window");
    require(near(output->outline.x, 8) && near(anchor_bunch.x, 8),
            "following bunch roots moved a fixed anchor");
}

void checkBranchFollowersBalanceSignalSides()
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch;
    bunch.x = bunch.y = 5;
    bunch.fixed = true; // no movable-bunch window in this force-only test
    auto directed = [&](rtl::Inst* driver, rtl::Inst* sink) {
        fixture.connect(*driver, *sink);
    };
    auto* merge = fixture.makeInst("merge", "LUT6", bunch, 5);
    auto* output = fixture.makeInst("merge_output", "OBUF", bunch, 9);
    output->outline.fixed = true;
    bunch.reg = output;
    directed(merge, output);
    for (int branch = 0; branch < 3; ++branch) {
        auto* leaf = fixture.makeInst("branch_"+std::to_string(branch), "LUT6", bunch, 3);
        directed(leaf, merge);
        for (int input = 0; input < 3; ++input) {
            auto* anchor = fixture.makeInst("input_"+std::to_string(branch*3+input), "IBUF", bunch, 1);
            anchor->outline.fixed = true;
            directed(anchor, leaf);
        }
    }
    fixture.outline.moveTimingAttractorsSimultaneously();
    require(merge->outline.x > 5,
            "remote input multiplicity drowned the nearer output-side pull");
    require(near(output->outline.x, 9), "branch following moved fixed output");
    require(!fixture.outline.isGravityAttractor(*merge),
            "balancing branch followers turned COMBs into attraction sources");
}

void checkCombinationalConeCanSpanItsAnchors()
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch, io_bunch;
    bunch.x = 2; bunch.y = 5;
    io_bunch.x = 9; io_bunch.y = 5; io_bunch.fixed = true;
    auto* lut = fixture.makeInst("wide_cone_follower", "LUT6", bunch, 2);
    auto* input = fixture.makeInst("wide_cone_input", "IBUF", io_bunch, 9);
    input->outline.fixed = true;
    fixture.connect(*input, *lut);
    for (int iteration = 0; iteration < 100; ++iteration) {
        float before = lut->outline.x;
        fixture.outline.moveTimingAttractorsSimultaneously();
        require(lut->outline.x >= before && lut->outline.x-before <= 0.07501F,
                "wide COMB cone exceeded the bounded follower step");
    }
    require(lut->outline.x > 8.8F && near(bunch.x, 2),
            "COMB follower was confined to its owner's old bunch window");
    require(near(input->outline.x, 9), "wide cone moved its fixed input");
}

void checkFollowerKeepsContinuationBeyondRootHorizon()
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch;
    bunch.x = bunch.y = 5;
    bunch.fixed = true;
    auto* input = fixture.makeInst("long_input", "IBUF", bunch, 0);
    auto* output = fixture.makeInst("long_output", "OBUF", bunch, 7);
    input->outline.fixed = output->outline.fixed = true;
    auto directed = [&](rtl::Inst* driver, rtl::Inst* sink) {
        fixture.connect(*driver, *sink);
    };
    std::vector<rtl::Inst*> chain;
    rtl::Inst* previous = input;
    for (int index = 0; index < 6; ++index) {
        auto* lut = fixture.makeInst("long_logic_"+std::to_string(index), "LUT6", bunch, index+1);
        directed(previous, lut);
        chain.push_back(lut);
        previous = lut;
    }
    directed(previous, output);
    fixture.outline.moveTimingAttractorsSimultaneously();
    require(near(chain.front()->outline.x, 1) && near(chain.back()->outline.x, 6),
            "bounded root following collapsed the chain's end cells onto I/Os");
    require(near(input->outline.x, 0) && near(output->outline.x, 7),
            "continuation following displaced fixed I/Os");
}

void checkTwoSidedFollowersUseConnectedGeometry(bool vertical, bool reverse_order)
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch;
    bunch.x = bunch.y = 5;
    bunch.fixed = true;
    auto* input = fixture.makeInst("local_input", "IBUF", bunch, 0);
    auto* output = fixture.makeInst("local_output", "OBUF", bunch, 8);
    input->outline.y = output->outline.y = 9;
    input->outline.fixed = output->outline.fixed = true;
    auto* left = fixture.makeInst("local_left", "LUT6", bunch, 4);
    auto* middle = fixture.makeInst("local_middle", "CARRY", bunch, 5);
    auto* right = fixture.makeInst("local_right", "LUT6", bunch, 6);
    fixture.connect(*input, *left);
    fixture.connect(*left, *middle);
    fixture.connect(*middle, *right);
    fixture.connect(*right, *output);
    if (vertical)
        for (auto* inst : {input, left, middle, right, output})
            std::swap(inst->outline.x, inst->outline.y);
    if (reverse_order)
        std::reverse(fixture.outline.optimization_order.begin(),
                     fixture.outline.optimization_order.end());
    const auto transverse = [&](rtl::Inst* inst) {
        return vertical ? inst->outline.x : inst->outline.y;
    };

    // The middle is already straight between its actual peers. Remote I/O
    // positions must not invent a bend there before those peers have moved.
    fixture.outline.moveTimingAttractorsSimultaneously();
    require(near(transverse(middle), 5),
            "remote roots bent a locally straight COMB connection");
    // Retain most of the longitudinal root force (-0.05 before blending),
    // rather than replacing it with the zero local-midpoint displacement.
    const float along = vertical ? middle->outline.y : middle->outline.x;
    require(along < 4.975F && along >= 4.95F,
            "straightening lost the roots' longitudinal tension");
    require(transverse(left) > 5 && transverse(right) > 5,
            "local following suppressed the boundary-root correction");
    fixture.outline.moveTimingAttractorsSimultaneously();
    require(transverse(middle) > 5,
            "COMB failed to follow its neighbors on the next snapshot");
    require(near(transverse(input), 9) && near(transverse(output), 9),
            "local following moved a fixed boundary");
}

void checkReconvergentPropagation(bool reverse)
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch;
    bunch.x = bunch.y = 5;
    auto* root = fixture.makeInst("root", "REG", bunch, 5.4F);
    auto* left = fixture.makeInst("left", "LUT6", bunch, 5.2F);
    auto* right = fixture.makeInst("right", "CARRY", bunch, 4.9F);
    auto* join = fixture.makeInst("join", "LUT6", bunch, 4.85F);
    auto* peer = fixture.makeInst("peer", "REG", bunch, 4.7F);
    root->outline.y = 5.1F;
    right->outline.y = 5.2F;
    join->outline.y = 4.7F;
    fixture.connect(*root, *left);
    fixture.connect(*root, *right);
    fixture.connect(*left, *join);
    fixture.connect(*right, *join);
    fixture.connect(*join, *peer);
    if (reverse) std::reverse(fixture.outline.optimization_order.begin(),
                              fixture.outline.optimization_order.end());
    for (int pass = 0; pass < 6; ++pass)
        fixture.outline.moveTimingAttractorsSimultaneously();
    // Reference from the original per-root visited-set/deque traversal.
    // A shared follower must be counted once per root, while REG peers
    // continue to originate their own movement rather than being followers.
    const std::array<std::pair<float, float>, 5> expected{{
        {5.24783230F, 5.08382177F}, {5.18233395F, 4.99456120F},
        {4.99785280F, 5.08383417F}, {4.84626675F, 4.82985544F},
        {4.76846313F, 4.89373779F}}};
    size_t index = 0;
    for (auto* inst : {root, left, right, join, peer}) {
        require(near(inst->outline.x, expected[index].first) &&
                    near(inst->outline.y, expected[index].second),
                "reconvergent propagation changed the simultaneous movement");
        ++index;
    }
}

// A long two-COMB path is invisible to endpoint springs when each endpoint
// has equally close neighbours on opposite sides. Whole-path pressure must
// move BOTH registers, without giving the COMBs independent gravity.
void checkWholePathRegisterPressure(bool vertical, bool fixed_launch)
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch;
    bunch.x = bunch.y = 5;
    bunch.fixed = true; // No artificial window in this force-only fixture.
    auto* launch = fixture.makeInst("span_launch", "REG", bunch, 1);
    auto* first = fixture.makeInst("span_first", "LUT6", bunch, 1.1F);
    auto* last = fixture.makeInst("span_last", "CARRY", bunch, 7.9F);
    auto* capture = fixture.makeInst("span_capture", "REG", bunch, 8);
    auto* left = fixture.makeInst("span_left_peer", "LUT6", bunch, 0.9F);
    auto* right = fixture.makeInst("span_right_peer", "LUT6", bunch, 8.1F);
    bunch.reg = launch;
    launch->outline.fixed = fixed_launch;
    fixture.connect(*launch, *first);
    fixture.connect(*first, *last);
    fixture.connect(*last, *capture);
    fixture.connect(*left, *launch);
    fixture.connect(*capture, *right);
    if (vertical) for (auto& inst : fixture.insts)
        std::swap(inst->outline.x,inst->outline.y);
    rtl::Conn input;
    input.inst_ref.set(static_cast<Referable<rtl::Inst>*>(capture));
    pnr::PlaceTimingAnalysis analysis;
    auto& endpoint = analysis.endpoint_details.emplace_back();
    endpoint.data_in = &input;
    endpoint.required_ns = 1;
    endpoint.arrival_ns = 2.6;
    endpoint.critical_edges = {{.sink=capture,.driver=last,.wire_delay_ns=0.1},
        {.sink=last,.driver=first,.wire_delay_ns=2.2},
        {.sink=first,.driver=launch,.wire_delay_ns=0.1}};
    fixture.outline.prepareTimingSpans(analysis,100,100);
    require(fixture.outline.timing_spans.size()==1
        && std::abs(fixture.outline.timing_spans.front().wire_budget_ns-0.77)<1e-8,
        "whole-path guide lost intrinsic delay or selected a COMB as launch");
    auto position = [&](rtl::Inst* inst) { return vertical ? inst->outline.y : inst->outline.x; };
    std::vector<std::pair<float,float>> original;
    for (auto& inst:fixture.insts) original.emplace_back(inst->outline.x,inst->outline.y);
    fixture.outline.moveTimingAttractorsSimultaneously();
    require(fixed_launch ? near(position(launch),1) : position(launch)>1.01F,
        "whole-path pressure failed to move the launch or moved a fixed launch");
    require(position(capture)<7.99F, "whole-path pressure did not move the capture");
    require(!fixture.outline.isGravityAttractor(*first) && !fixture.outline.isGravityAttractor(*last),
        "whole-path guide turned COMBs into gravity sources");
    require(std::abs(position(launch)-1)<=0.07501F && std::abs(position(capture)-8)<=0.07501F,
        "whole-path pressure bypassed the movement cap");
    // A second clock/setup limit uses its own requirement. Refresh must replace
    // old paths, not keep applying the former tight-clock attraction.
    endpoint.required_ns = 5;
    fixture.outline.prepareTimingSpans(analysis,100,100);
    require(std::abs(fixture.outline.timing_spans.front().wire_budget_ns-4.77)<1e-8,
        "path guide ignored the endpoint's clock/setup requirement");
    for (size_t i=0;i<original.size();++i) {
        fixture.insts[i]->outline.x=original[i].first;
        fixture.insts[i]->outline.y=original[i].second;
    }
    fixture.outline.moveTimingAttractorsSimultaneously();
    require(near(position(launch),1) && near(position(capture),8),
        "a timing-safe span still applied register pressure");
    // Keep the tight guide but shrink the geometry, without refreshing timing.
    endpoint.required_ns=1;
    fixture.outline.prepareTimingSpans(analysis,100,100);
    for(size_t i=0;i<original.size();++i) {
        fixture.insts[i]->outline.x=original[i].first/10;
        fixture.insts[i]->outline.y=original[i].second/10;
    }
    fixture.outline.moveTimingAttractorsSimultaneously();
    require(near(position(launch),0.1F) && near(position(capture),0.8F),
        "whole-path attraction used stale excess after live geometry became safe");
    launch->cell_ref->type="IBUF";
    fixture.outline.prepareTimingSpans(analysis,100,100);
    require(fixture.outline.timing_spans.empty(),
        "register span guide bypassed the fixed-I/O faded propagation");
    launch->cell_ref->type="REG";
    endpoint.arrival_ns=10; // Intrinsic delay alone now exceeds the period.
    fixture.outline.prepareTimingSpans(analysis,100,100);
    require(fixture.outline.timing_spans.empty(),
        "an intrinsically impossible path became a zero-distance span constraint");
    analysis.endpoint_details.clear();
    fixture.outline.prepareTimingSpans(analysis,100,100);
    require(fixture.outline.timing_spans.empty(), "refresh retained stale critical spans");
}

// Abstract replay of the two-clock puzzle's competing root pulls. The local
// LUT/CARRY correction points right/up; the old 7/8 blend reversed both axes.
void checkCombCorrectionIsNotReversed(bool vertical, bool reverse, bool boundary=false)
{
    Fixture fixture;
    Referable<pnr::RegBunch> bunch;
    bunch.x=bunch.y=5; bunch.fixed=true;
    auto make = [&](const char* name,const char* type,float x,float y) {
        auto* inst=fixture.makeInst(name,type,bunch,x/10);
        inst->outline.y=y/10;
        return inst;
    };
    auto* source=make("correction_source","REG",36.2095F,54.9702F);
    auto* a=make("correction_a","LUT6",39.1023F,56.2332F);
    auto* b=make("correction_b","CARRY",54.4903F,63.2753F);
    auto* nearby=make("correction_near","REG",34.2F,49.3F);
    auto* out0=make("correction_out0","REG",56.85F,63.45F);
    auto* out1=make("correction_out1","REG",56.91F,64.52F);
    bunch.reg=source;
    auto link=[&](rtl::Inst* driver,rtl::Inst* sink,double weight) {
        fixture.connect(*driver,*sink);
        fixture.tech.place.place_timing.placement_net_weights[driver][sink]=weight-1;
        fixture.tech.place.place_timing.placement_net_weights[sink][driver]=weight-1;
    };
    link(source,a,6.488); link(a,nearby,1.447178); link(a,b,4.866);
    link(b,out0,3.064); link(b,out1,3.244);
    // Preserve the competing launch-register forces from the recorded case.
    // Without these connections the source itself moves right, hiding the
    // reversal in the propagated COMB correction.
    auto* upstream=make("correction_upstream","LUT6",31.6307F,52.0950F);
    auto* side=make("correction_side","LUT6",28.5689F,51.8418F);
    link(upstream,source,1.046025); link(source,side,1.932990);
    auto* near_balance=make("correction_near_balance","LUT6",29.2977F,42.3668F);
    link(near_balance,nearby,1.447178);
    if(boundary) {
        // A fixed output reaches A through B: preserving only *direct* I/O
        // neighbours is insufficient. Its faded pull must retain precedence
        // over a local centroid, and the package anchor must not move.
        out0->cell_ref->type="OBUF";
        out0->outline.fixed=true;
        out0->outline.x=1; out0->outline.y=5.6F;
    }
    if(vertical) for(auto& inst:fixture.insts) std::swap(inst->outline.x,inst->outline.y);
    if(reverse) std::reverse(fixture.outline.optimization_order.begin(),fixture.outline.optimization_order.end());
    float before_x=a->outline.x, before_y=a->outline.y;
    float fixed_x=out0->outline.x, fixed_y=out0->outline.y;
    fixture.outline.moveTimingAttractorsSimultaneously();
    // Dot product with the local correction, not an arbitrary preferred axis.
    const double local_x=vertical ? 0.128 : 0.391;
    const double local_y=vertical ? 0.391 : 0.128;
    const double projection=(a->outline.x-before_x)*local_x+(a->outline.y-before_y)*local_y;
    require(boundary ? projection<0 : projection>0,
        boundary ? "local centroid overruled a propagated fixed-boundary correction"
                 : "propagated root tension reversed the COMB's useful local correction");
    if(boundary) require(near(out0->outline.x,fixed_x) && near(out0->outline.y,fixed_y),
                        "COMB direction correction moved a fixed boundary");
}

}

int main(int argc, char** argv)
{
    if(argc==2) {
        const std::string mode=argv[1];
        require(mode=="span" || mode=="comb", "unknown focused regression");
        for(bool vertical:{false,true}) for(bool option:{false,true}) {
            if(mode=="span") checkWholePathRegisterPressure(vertical,option);
            else for(bool boundary:{false,true}) checkCombCorrectionIsNotReversed(vertical,option,boundary);
        }
        std::cout << "outline_attractor_test " << mode << " passed\n";
        return 0;
    }
    for(bool vertical:{false,true}) {
        for(bool fixed:{false,true}) checkWholePathRegisterPressure(vertical,fixed);
        for(bool reverse:{false,true}) for(bool boundary:{false,true})
            checkCombCorrectionIsNotReversed(vertical,reverse,boundary);
    }
    checkReconvergentPropagation(false);
    checkReconvergentPropagation(true);
    // Each fixture owns pointers to the test's stack-local bunches. Do not
    // reuse its optimization graph after those bunches leave scope.
    for(auto check : {checkRegisterGravity,checkIoGravity,checkCombinationalCellsHaveNoGravity,
            checkTimingDeficitControlsAcceleration,checkUnequalDistancesDoNotCancel,
            checkPropagationIsBounded,checkTensionCrossesRegisterTiers,
            checkChainRelaxesBetweenOppositeAnchors,checkFixedAnchorsSeedTautRadialAllocation}) {
        Fixture fixture;
        check(fixture);
    }
    for(bool partial : {false,true}) for(bool vertical : {false,true})
        checkBlockedRootStillCorrectsFollowers(partial,vertical);
    checkBlockedRegistersCannotOverpowerFixedAnchor();
    for(bool vertical : {false,true}) for(bool reverse : {false,true})
        checkRegisterLutRegisterDetour(vertical,reverse);
    checkBalancedStationaryRoot();
    checkEdgeLutReturnsTowardBlockedRegisters();
    checkPartiallyBlockedDiagonalMotion();
    checkSharedCombinationalBunchLinks();
    checkFixedOutputTraversal();
    checkBunchWindowFollowsRegister();
    checkBranchFollowersBalanceSignalSides();
    checkCombinationalConeCanSpanItsAnchors();
    checkFollowerKeepsContinuationBeyondRootHorizon();
    for (bool vertical : {false, true}) for (bool reverse : {false, true})
        checkTwoSidedFollowersUseConnectedGeometry(vertical, reverse);
    std::cout << "OUTLINE_ATTRACTOR_TEST register=anchor iob=fixed_anchor "
                 "lut=follower carry=propagated no_comb_gravity=1 "
                 "timing_weighted_acceleration=1 bounded_hops=4 "
                 "distance_sensitive_force=1 multi_register_tension=1 "
                 "opposite_anchor_relaxation=1 fixed_anchor_tension_seed=1 "
                 "relative_comb_following=1 stationary_root_correction=1 "
                 "detour_contraction=1 blocked_register_io_balance=1 "
                 "fixed_output_traversal=1 fixed_density_guard=1 "
                 "moving_bunch_window=1 directed_branch_balance=1 "
                 "wide_comb_cone=1 bounded_root_continuation=1 "
                 "connected_comb_geometry=1 whole_path_register_pressure=1 "
                 "nonreversing_comb_correction=1\n";
    return 0;
}
