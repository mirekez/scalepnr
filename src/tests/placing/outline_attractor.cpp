#include "OutlineDesign.h"
#include "Tech.h"

#include <algorithm>
#include <cmath>
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

}

int main()
{
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
    std::cout << "OUTLINE_ATTRACTOR_TEST register=anchor iob=fixed_anchor "
                 "lut=follower carry=propagated no_comb_gravity=1 "
                 "timing_weighted_acceleration=1 bounded_hops=4 "
                 "distance_sensitive_force=1 multi_register_tension=1 "
                 "opposite_anchor_relaxation=1 fixed_anchor_tension_seed=1 "
                 "relative_comb_following=1 stationary_root_correction=1 "
                 "detour_contraction=1 blocked_register_io_balance=1\n";
    return 0;
}
