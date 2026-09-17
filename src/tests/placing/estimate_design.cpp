#include "EstimateDesign.h"
#include "Tech.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// This focused test links no Tech.cpp, so provide the three technology tables
// used by EstimateDesign here.
technology::CombDelays technology::Tech::comb_delays;
std::multimap<std::string, std::string> technology::Tech::clocked_ports;
std::multimap<std::string, std::string> technology::Tech::buffers_ports;

namespace
{

struct TestFailure
{
    std::string message;
};

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw TestFailure{message};
    }
}

bool near(double left, double right, double epsilon = 1e-9)
{
    return std::abs(left - right) <= epsilon;
}

struct TreeSpec
{
    int id = 0;
    double path_delay_ns = 0;
    std::vector<double> cell_delays_ns;
    int direct_registers = 0;
};

std::vector<TreeSpec> makeSpecs()
{
    constexpr int tree_count = 24;
    std::mt19937 random(0x5ca1e123u);
    std::uniform_int_distribution<int> depth_distribution(2, 6);
    std::uniform_real_distribution<double> weight_distribution(0.25, 2.5);
    std::uniform_int_distribution<int> register_distribution(7, 10);

    std::vector<TreeSpec> specs;
    specs.reserve(tree_count);
    for (int id = 0; id < tree_count; ++id) {
        // Strictly increasing numerical deficit: delay - common clock period.
        // Random depths and partitions keep the trees structurally diverse.
        double path_delay = 0.55 + 0.085*id;
        int depth = depth_distribution(random);
        std::vector<double> weights(static_cast<size_t>(depth));
        for (double& weight : weights) {
            weight = weight_distribution(random);
        }
        double total_weight = std::accumulate(weights.begin(), weights.end(), 0.0);
        for (double& weight : weights) {
            weight = path_delay*weight/total_weight;
        }
        specs.push_back(TreeSpec{
            .id = id,
            .path_delay_ns = path_delay,
            .cell_delays_ns = std::move(weights),
            .direct_registers = register_distribution(random),
        });
    }
    return specs;
}

struct Fixture
{
    static constexpr double clock_period_ns = 1.0;

    rtl::Design design;
    Referable<rtl::Module> top_module;
    Referable<rtl::Module> primitive_module;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    clk::Clocks clocks;
    technology::Tech tech;
    int next_designator = 1;

    explicit Fixture(const std::vector<TreeSpec>& specs,
                     const std::vector<int>& output_order)
    {
        top_module.name = "estimate_test_top";
        top_module.is_blackbox = false;
        primitive_module.name = "estimate_test_primitives";
        primitive_module.is_blackbox = true;
        primitive_module.parent_ref.set(&top_module);

        design.top_cell.name = "top";
        design.top_cell.type = "top";
        design.top_cell.module_ref.set(&top_module);
        design.top.cell_ref.set(&design.top_cell);
        design.top.depth = 0;
        design.top.pos = -1;
        design.top_cell.ports.reserve(specs.size());
        design.top.conns.reserve(specs.size());
        cells.reserve(specs.size()*16);
        insts.reserve(specs.size()*16 + 1);

        technology::Tech::clocked_ports.clear();
        technology::Tech::buffers_ports.clear();
        technology::Tech::comb_delays.map.clear();
        technology::Tech::clocked_ports.emplace("REG", "C");
        technology::Tech::buffers_ports.emplace("OUT_BUFFER", "I");

        Referable<rtl::Inst>* clock_source = makeInst(
            "clock_source", "CLOCK_SOURCE", {{"O", rtl::Port::PORT_OUT}});
        Referable<rtl::Conn>* clock_output = conn(clock_source, "O");
        require(clock_output, "clock source has no output");
        clocks.clocks_list.emplace_back(rtl::Clock{
            .name = "test_clock",
            .conn_ptr = clock_output,
            .conn_name = "clock_source.O",
            .period_ns = clock_period_ns,
            .duty = 50,
        });

        std::vector<Referable<rtl::Inst>*> output_buffers(specs.size(), nullptr);
        for (const TreeSpec& spec : specs) {
            output_buffers[static_cast<size_t>(spec.id)] = buildTree(spec, clock_source);
        }
        for (int id : output_order) {
            addTopOutput(id, output_buffers.at(static_cast<size_t>(id)));
        }
    }

    Referable<rtl::Inst>* makeInst(
        const std::string& name, const std::string& type,
        const std::vector<std::pair<std::string, int>>& ports)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name;
        cell->type = type;
        cell->module_ref.set(&primitive_module);
        cell->ports.reserve(ports.size());
        int input_index = 0;
        int output_index = 0;
        for (const auto& [port_name, port_type] : ports) {
            rtl::Port port;
            port.name = port_name;
            port.type = static_cast<decltype(port.type)>(port_type);
            if (port.type == rtl::Port::PORT_IN) {
                port.index = input_index++;
            }
            else if (port.type == rtl::Port::PORT_OUT) {
                port.index = output_index++;
            }
            cell->ports.emplace_back(std::move(port));
        }

        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(cell.get());
        inst->parent_ref.set(&design.top);
        inst->cnt_inputs = input_index;
        inst->cnt_outputs = output_index;
        inst->pos = -1;
        inst->conns.reserve(cell->ports.size());
        for (auto& port : cell->ports) {
            auto& connection = inst->conns.emplace_back();
            connection.port_ref.set(&port);
            connection.inst_ref.set(inst.get());
        }

        Referable<rtl::Inst>* result = inst.get();
        cells.push_back(std::move(cell));
        insts.push_back(std::move(inst));
        return result;
    }

    Referable<rtl::Conn>* conn(Referable<rtl::Inst>* inst,
                              const std::string& port_name)
    {
        for (auto& candidate : inst->conns) {
            if (candidate.port_ref.peer && candidate.port_ref->name == port_name) {
                return &candidate;
            }
        }
        return nullptr;
    }

    void connect(Referable<rtl::Inst>* driver, const std::string& output_name,
                 Referable<rtl::Inst>* sink, const std::string& input_name)
    {
        Referable<rtl::Conn>* output = conn(driver, output_name);
        Referable<rtl::Conn>* input = conn(sink, input_name);
        require(output && input, "synthetic tree references a missing port");
        int designator = next_designator++;
        output->port_ref->designator = designator;
        input->port_ref->designator = designator;
        input->set(output);
        auto& net = top_module.nets.emplace_back();
        net.name = "net_" + std::to_string(designator);
        net.designators.push_back(designator);
    }

    Referable<rtl::Inst>* makeRegister(const std::string& name,
                                       Referable<rtl::Inst>* clock_source)
    {
        Referable<rtl::Inst>* reg = makeInst(name, "REG", {
            {"D", rtl::Port::PORT_IN},
            {"C", rtl::Port::PORT_IN},
            {"Q", rtl::Port::PORT_OUT},
        });
        connect(clock_source, "O", reg, "C");
        return reg;
    }

    Referable<rtl::Inst>* buildTree(const TreeSpec& spec,
                                    Referable<rtl::Inst>* clock_source)
    {
        std::string prefix = "tree_" + std::to_string(spec.id) + "_";
        Referable<rtl::Inst>* driver = makeRegister(prefix + "source", clock_source);
        std::string driver_port = "Q";

        for (size_t index = 0; index < spec.cell_delays_ns.size(); ++index) {
            std::string type = "COMB_" + std::to_string(spec.id) + "_"
                + std::to_string(index);
            technology::Tech::comb_delays.map[type] = {
                1, {spec.cell_delays_ns[index]}
            };
            Referable<rtl::Inst>* logic = makeInst(
                prefix + "logic_" + std::to_string(index), type,
                {{"I", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
            connect(driver, driver_port, logic, "I");
            driver = logic;
            driver_port = "O";
        }

        Referable<rtl::Inst>* path_register = makeRegister(
            prefix + "path_reg", clock_source);
        connect(driver, driver_port, path_register, "D");
        driver = path_register;
        for (int index = 0; index < spec.direct_registers; ++index) {
            Referable<rtl::Inst>* next = makeRegister(
                prefix + "direct_reg_" + std::to_string(index), clock_source);
            connect(driver, "Q", next, "D");
            driver = next;
        }

        Referable<rtl::Inst>* output_buffer = makeInst(
            prefix + "output", "OUT_BUFFER",
            {{"I", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
        connect(driver, "Q", output_buffer, "I");
        return output_buffer;
    }

    void addTopOutput(int id, Referable<rtl::Inst>* output_buffer)
    {
        rtl::Port port;
        port.name = "out_" + std::to_string(id);
        port.type = rtl::Port::PORT_OUT;
        port.index = id;
        port.designator = next_designator++;
        design.top_cell.ports.emplace_back(std::move(port));

        auto& top_connection = design.top.conns.emplace_back();
        top_connection.port_ref.set(&design.top_cell.ports.back());
        top_connection.inst_ref.set(&design.top);
        Referable<rtl::Conn>* output = conn(output_buffer, "O");
        require(output, "output buffer has no output");
        output->port_ref->designator = top_connection.port_ref->designator;
        top_connection.set(output);
    }

    pnr::EstimateDesign run()
    {
        pnr::EstimateDesign estimate;
        estimate.tech = &tech;
        estimate.clocks = &clocks;
        estimate.estimateDesign(design);
        return estimate;
    }
};

std::string bunchSignature(const std::list<Referable<pnr::RegBunch>>& bunches)
{
    std::ostringstream output;
    for (const auto& bunch : bunches) {
        output << '(' << bunch.reg->cell_ref->name
               << ':' << bunch.size
               << ':' << bunch.size_regs
               << ':' << bunch.size_regs_own
               << ':' << bunch.size_comb
               << ':' << bunch.size_comb_own
               << ':' << bunch.sub_bunches.size()
               << ':' << bunch.uplinks.size()
               << bunchSignature(bunch.sub_bunches) << ')';
    }
    return output.str();
}

bool hasAggregatedRegisters(const std::list<Referable<pnr::RegBunch>>& bunches)
{
    for (const auto& bunch : bunches) {
        if (bunch.size_regs_own > 1 || hasAggregatedRegisters(bunch.sub_bunches)) {
            return true;
        }
    }
    return false;
}

void randomized_trees_are_sorted_and_aggregated_deterministically()
{
    std::vector<TreeSpec> specs = makeSpecs();
    std::vector<int> ascending(specs.size());
    std::iota(ascending.begin(), ascending.end(), 0);
    std::vector<int> shuffled = ascending;
    std::mt19937 shuffle_random(0x51f17e5u);
    std::shuffle(shuffled.begin(), shuffled.end(), shuffle_random);
    require(shuffled != ascending, "fixed shuffle unexpectedly preserved tree order");

    Fixture canonical_fixture(specs, ascending);
    pnr::EstimateDesign canonical = canonical_fixture.run();
    Fixture shuffled_fixture(specs, shuffled);
    pnr::EstimateDesign estimated = shuffled_fixture.run();

    require(canonical.data_outs.size() == specs.size(),
            "canonical estimation lost output trees");
    require(estimated.data_outs.size() == specs.size(),
            "shuffled estimation lost output trees");
    require(bunchSignature(estimated.data_outs) == bunchSignature(canonical.data_outs),
            "shuffling RTL outputs changed sorted or aggregated bunches");
    require(hasAggregatedRegisters(estimated.data_outs),
            "long zero-combinational register chains were not aggregated");

    auto bunch = estimated.data_outs.begin();
    for (auto spec = specs.rbegin(); spec != specs.rend(); ++spec, ++bunch) {
        std::string expected_name = "tree_" + std::to_string(spec->id) + "_output";
        require(bunch != estimated.data_outs.end(), "sorted bunch list ended early");
        require(bunch->reg->cell_ref->name == expected_name,
                "EstimateDesign did not sort the greatest timing deficit first");
        if (!near(bunch->reg->stats.top_max_delay, spec->path_delay_ns)) {
            std::ostringstream message;
            message << expected_name << " path delay is "
                    << bunch->reg->stats.top_max_delay << " ns, expected "
                    << spec->path_delay_ns << " ns; arcs=";
            for (double delay : spec->cell_delays_ns) {
                message << delay << ',';
            }
            throw TestFailure{message.str()};
        }
        require(near(bunch->reg->stats.max_deficit,
                     spec->path_delay_ns - Fixture::clock_period_ns),
                "timing deficit is not path delay minus clock period");
    }

    std::cout << "ESTIMATE_DESIGN_TEST trees=" << specs.size()
              << " shuffled=yes aggregation=yes deficit_range_ns="
              << specs.front().path_delay_ns - Fixture::clock_period_ns << ".."
              << specs.back().path_delay_ns - Fixture::clock_period_ns << '\n';
}

}

int main()
{
    try {
        randomized_trees_are_sorted_and_aggregated_deterministically();
    }
    catch (const TestFailure& failure) {
        std::cerr << "estimate_design_test: " << failure.message << '\n';
        return 1;
    }
    std::cout << "estimate_design_test passed\n";
    return 0;
}
