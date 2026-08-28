#include "OutlineDesign.h"
#include "Tech.h"

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
    std::vector<int> occupancy;

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
        occupancy.resize(static_cast<size_t>(
            outline.fpga_width*outline.fpga_height));
        outline.boxes1 = occupancy.data();
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
        std::fill(occupancy.begin(), occupancy.end(), 0);
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
    require(lut->outline.x < lut_before
                && carry0->outline.x < carry0_before
                && carry1->outline.x < carry1_before,
            "register displacement did not propagate through its LUT/carry constellation");
    require(lut_before - lut->outline.x
                > carry0_before - carry0->outline.x
                && carry0_before - carry0->outline.x
                    > carry1_before - carry1->outline.x,
            "register displacement did not decay along its combinational chain");
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
    // Fixed-anchor motion reaches only its combinational constellation in one
    // pass.  Subsequent registers then originate their own force from the
    // changed neighbor positions, relaying tension without being dragged.
    for (int iteration = 1; iteration < 30; ++iteration) {
        fixture.run(*iob, bunch);
    }
    float first_tier = reg0_before - reg0->outline.x;
    float second_tier = reg1_before - reg1->outline.x;
    require(first_tier > second_tier && second_tier > 0,
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

}

int main()
{
    Fixture fixture;
    checkRegisterGravity(fixture);
    checkIoGravity(fixture);
    checkCombinationalCellsHaveNoGravity(fixture);
    checkTimingDeficitControlsAcceleration(fixture);
    checkUnequalDistancesDoNotCancel(fixture);
    checkPropagationIsBounded(fixture);
    checkTensionCrossesRegisterTiers(fixture);
    checkChainRelaxesBetweenOppositeAnchors(fixture);
    checkFixedAnchorsSeedTautRadialAllocation(fixture);
    std::cout << "OUTLINE_ATTRACTOR_TEST register=anchor iob=fixed_anchor "
                 "lut=follower carry=propagated no_comb_gravity=1 "
                 "timing_weighted_acceleration=1 bounded_hops=4 "
                 "distance_sensitive_force=1 multi_register_tension=1 "
                 "opposite_anchor_relaxation=1 fixed_anchor_tension_seed=1\n";
    return 0;
}
