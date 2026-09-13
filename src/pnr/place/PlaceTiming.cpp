#include "PlaceTiming.h"

#include "Tech.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

using namespace pnr;

namespace
{

int signum(int value)
{
    return (value > 0) - (value < 0);
}

rtl::Conn* followedDriver(rtl::Conn* sink_input)
{
    return sink_input ? sink_input->follow() : nullptr;
}

struct OutputResult
{
    double arrival_ns = 0;
    clk::TimingPath* critical_input = nullptr;
};

struct ForceAccumulator
{
    PlaceTimingForce force;
    double strongest_weight = -1;
};

struct AnalysisContext
{
    PlaceTiming& owner;
    PlaceTimingAnalysis analysis;
    std::unordered_map<const clk::TimingPath*, OutputResult> output_cache;
    std::unordered_set<const clk::TimingPath*> active;
    std::unordered_map<rtl::Inst*, ForceAccumulator> force_by_inst;
    std::unordered_map<rtl::Conn*, size_t> fanout_cache;

    double wireDelay(rtl::Conn& sink_input, rtl::Conn& driver_output)
    {
        auto found = fanout_cache.find(&driver_output);
        if (found == fanout_cache.end()) {
            size_t fanout = rtl::Conn::fromBase(driver_output).getPeers().size();
            found = fanout_cache.emplace(&driver_output, fanout).first;
        }
        return owner.estimateWireDelay(
            sink_input, driver_output, found->second);
    }

    double intrinsicDelay(const clk::TimingPath& output_path,
                          const clk::TimingPath& input_path) const
    {
        rtl::Conn* output = output_path.data_output;
        if (!output) {
            output = followedDriver(output_path.data_in);
        }
        if (!owner.tech || !output || !output->inst_ref.peer
            || !output->inst_ref->cell_ref.peer || !input_path.data_in
            || !input_path.data_in->port_ref.peer || !output->port_ref.peer) {
            return 0;
        }
        return owner.tech->comb_delays.getDelay(
            output->inst_ref->cell_ref->type,
            input_path.data_in->port_ref->index,
            output->port_ref->index);
    }

    clk::TimingPath* canonical(clk::TimingPath& path) const
    {
        return path.precalculated ? path.precalculated : &path;
    }

    OutputResult evaluateOutput(clk::TimingPath& original)
    {
        clk::TimingPath* path = canonical(original);
        auto cached = output_cache.find(path);
        if (cached != output_cache.end()) {
            return cached->second;
        }
        if (!active.insert(path).second) {
            return {};
        }

        OutputResult result;
        if (path->data_output && !path->sub_paths.empty()) {
            result.arrival_ns = -std::numeric_limits<double>::infinity();
            for (auto& input_path : path->sub_paths) {
                if (!input_path.data_in) {
                    continue;
                }
                double candidate = evaluateInput(input_path)
                    + intrinsicDelay(*path, input_path);
                if (candidate > result.arrival_ns) {
                    result.arrival_ns = candidate;
                    result.critical_input = &input_path;
                }
            }
            if (!std::isfinite(result.arrival_ns)) {
                result.arrival_ns = 0;
            }
        }

        active.erase(path);
        output_cache.emplace(path, result);
        ++analysis.evaluated_nodes;
        return result;
    }

    double evaluateInput(clk::TimingPath& path)
    {
        rtl::Conn* driver = followedDriver(path.data_in);
        if (!driver) {
            return 0;
        }
        double wire_delay = wireDelay(*path.data_in, *driver);
        ++analysis.evaluated_edges;
        analysis.total_wire_delay_ns += wire_delay;
        if (!path.data_in->inst_ref.peer || !driver->inst_ref.peer
            || !path.data_in->inst_ref->tile.peer || !driver->inst_ref->tile.peer) {
            ++analysis.unplaced_edges;
        }
        return wire_delay + evaluateOutput(path).arrival_ns;
    }

    void appendCriticalEdges(clk::TimingPath& original,
                             std::vector<PlaceTimingEdge>& edges)
    {
        rtl::Conn* driver = followedDriver(original.data_in);
        if (!driver || !original.data_in) {
            return;
        }
        edges.push_back(PlaceTimingEdge{
            .sink_input = original.data_in,
            .driver_output = driver,
            .sink = original.data_in->inst_ref.peer,
            .driver = driver->inst_ref.peer,
            .wire_delay_ns = wireDelay(*original.data_in, *driver),
        });

        clk::TimingPath* path = canonical(original);
        OutputResult output = evaluateOutput(*path);
        if (output.critical_input) {
            appendCriticalEdges(*output.critical_input, edges);
        }
    }

    void addForce(rtl::Inst* inst, rtl::Inst* peer, double x, double y,
                  double weight)
    {
        if (!inst || !peer || weight <= 0) {
            return;
        }
        ForceAccumulator& accumulator = force_by_inst[inst];
        accumulator.force.inst = inst;
        accumulator.force.x += x*weight;
        accumulator.force.y += y*weight;
        accumulator.force.weight += weight;
        ++accumulator.force.violated_paths;
        if (weight > accumulator.strongest_weight) {
            accumulator.strongest_weight = weight;
            accumulator.force.strongest_peer = peer;
        }
    }

    void addEndpointForces(const PlaceTimingEndpoint& endpoint)
    {
        double violation = std::max(0.0, -endpoint.slack_ns);
        if (violation <= 0) {
            return;
        }
        for (const PlaceTimingEdge& edge : endpoint.critical_edges) {
            if (!edge.driver || !edge.sink || !edge.driver->tile.peer
                || !edge.sink->tile.peer) {
                continue;
            }
            int dx = edge.sink->coord.x - edge.driver->coord.x;
            int dy = edge.sink->coord.y - edge.driver->coord.y;
            if (dx == 0 && dy == 0) {
                continue;
            }
            double weight = violation*std::max(
                edge.wire_delay_ns, owner.calibration.local_wire_ns);
            addForce(edge.driver, edge.sink, signum(dx), signum(dy), weight);
            addForce(edge.sink, edge.driver, -signum(dx), -signum(dy), weight);
        }
    }

    void addEndpoint(rtl::Clock* clock, clk::Timings::TimingInfo& info)
    {
        if (!clock || !info.path.data_in) {
            return;
        }
        PlaceTimingEndpoint endpoint;
        endpoint.clock = clock;
        endpoint.timing_path = &info.path;
        endpoint.data_in = info.data_in ? info.data_in : info.path.data_in;
        endpoint.required_ns = info.setup_limit > 0
            ? info.setup_limit : clock->period_ns;
        endpoint.arrival_ns = evaluateInput(info.path);
        endpoint.slack_ns = endpoint.required_ns - endpoint.arrival_ns;
        if (analysis.endpoints == 0) {
            analysis.worst_slack_ns = endpoint.slack_ns;
        }
        else {
            analysis.worst_slack_ns = std::min(
                analysis.worst_slack_ns, endpoint.slack_ns);
        }
        ++analysis.endpoints;
        appendCriticalEdges(info.path, endpoint.critical_edges);
        if (endpoint.slack_ns < 0) {
            ++analysis.violated_endpoints;
            analysis.total_negative_slack_ns -= endpoint.slack_ns;
            addEndpointForces(endpoint);
        }
        analysis.endpoint_details.push_back(std::move(endpoint));
    }

    PlaceTimingAnalysis finish(
        std::chrono::steady_clock::time_point started)
    {
        analysis.forces.reserve(force_by_inst.size());
        for (auto& [inst, accumulator] : force_by_inst) {
            (void) inst;
            analysis.forces.push_back(accumulator.force);
        }
        std::ranges::sort(analysis.forces,
            [](const PlaceTimingForce& left, const PlaceTimingForce& right) {
                if (left.weight != right.weight) {
                    return left.weight > right.weight;
                }
                if (left.inst && right.inst
                    && left.inst->coord.y != right.inst->coord.y) {
                    return left.inst->coord.y < right.inst->coord.y;
                }
                if (left.inst && right.inst
                    && left.inst->coord.x != right.inst->coord.x) {
                    return left.inst->coord.x < right.inst->coord.x;
                }
                if (left.inst && right.inst) {
                    return left.inst->makeName() < right.inst->makeName();
                }
                return left.inst != nullptr;
            });
        analysis.elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        return std::move(analysis);
    }
};

}

void PlaceTiming::preparePlacementGuide(clk::Timings& timings)
{
    placement_net_weights.clear();
    for (auto& [clock, infos] : timings.clocked_inputs) {
        if (!clock) continue;
        for (auto& info : infos) {
            if (!info.path.data_in) continue;
            double required_ns = info.setup_limit > 0
                ? info.setup_limit : clock->period_ns;
            // Every routed data net has a baseline physical cost. Nets in a
            // tighter clock domain receive greater pressure while capacity is
            // being smeared, before exact placed slack is available.
            double safe_required_ns = std::max(required_ns, 0.05);
            double setup_deficit_ns = std::max(
                0.0, info.path.max_setup_time - required_ns);
            double timing_pressure = (1.0
                + setup_deficit_ns/safe_required_ns)/safe_required_ns;
            std::unordered_set<const clk::TimingPath*> visited;
            auto visit = [&](auto&& self, clk::TimingPath& original) -> void {
                rtl::Conn* driver = followedDriver(original.data_in);
                rtl::Inst* sink_inst = original.data_in
                    ? original.data_in->inst_ref.peer : nullptr;
                rtl::Inst* driver_inst = driver
                    ? driver->inst_ref.peer : nullptr;
                if (sink_inst && driver_inst && sink_inst != driver_inst) {
                    placement_net_weights[sink_inst][driver_inst]
                        += timing_pressure;
                    placement_net_weights[driver_inst][sink_inst]
                        += timing_pressure;
                }

                clk::TimingPath* path = original.precalculated
                    ? original.precalculated : &original;
                if (!visited.insert(path).second) return;
                for (clk::TimingPath& sub_path : path->sub_paths) {
                    self(self, sub_path);
                }
            };
            visit(visit, info.path);
        }
    }
}

double PlaceTiming::placementNetWeight(const rtl::Inst& inst,
                                       const rtl::Inst& peer) const
{
    double weight = 1.0;
    auto inst_weights = placement_net_weights.find(&inst);
    if (inst_weights == placement_net_weights.end()) return weight;
    auto peer_weight = inst_weights->second.find(&peer);
    return peer_weight == inst_weights->second.end()
        ? weight : weight + peer_weight->second;
}

double PlaceTiming::estimateWireDelay(const rtl::Conn& sink_input,
                                      const rtl::Conn& driver_output) const
{
    size_t fanout = rtl::Conn::fromBase(
        const_cast<rtl::Conn&>(driver_output)).getPeers().size();
    return estimateWireDelay(sink_input, driver_output, fanout);
}

double PlaceTiming::estimateWireDelay(const rtl::Conn& sink_input,
                                      const rtl::Conn& driver_output,
                                      size_t fanout) const
{
    rtl::Inst* sink = sink_input.inst_ref.peer;
    rtl::Inst* driver = driver_output.inst_ref.peer;
    if (!sink || !driver || !sink->tile.peer || !driver->tile.peer) {
        return 0;
    }

    int dx = std::abs(sink->coord.x - driver->coord.x);
    int dy = std::abs(sink->coord.y - driver->coord.y);
    double delay = calibration.local_wire_ns
        + dx*calibration.horizontal_ns_per_tile
        + dy*calibration.vertical_ns_per_tile;
    if (dx != 0 && dy != 0) {
        delay += calibration.bend_ns;
    }

    if (fanout > 1) {
        delay += std::log2(static_cast<double>(fanout))
            * calibration.extra_fanout_ns;
    }
    return delay;
}

PlaceTimingAnalysis PlaceTiming::analyze(clk::Timings& timings)
{
    auto started = std::chrono::steady_clock::now();
    AnalysisContext context{*this};

    for (auto& [clock, infos] : timings.clocked_inputs) {
        for (auto& info : infos) {
            context.addEndpoint(clock, info);
        }
    }

    return context.finish(started);
}

void PlaceTiming::correctSetupTiming(PlaceTimingEndpoint& endpoint) const
{
    double arrival_ns = endpoint.arrival_ns;
    for (PlaceTimingEdge& edge : endpoint.critical_edges) {
        if (!edge.sink_input || !edge.driver_output) continue;
        double corrected_wire_delay = estimateWireDelay(
            *edge.sink_input, *edge.driver_output);
        arrival_ns += corrected_wire_delay - edge.wire_delay_ns;
        edge.wire_delay_ns = corrected_wire_delay;
    }
    endpoint.arrival_ns = arrival_ns;
    endpoint.slack_ns = endpoint.required_ns - arrival_ns;
}

void PlaceTiming::evaluateSetupTiming(
    const std::vector<PlaceTimingEndpoint*>& endpoints)
{
    AnalysisContext context{*this};
    for (PlaceTimingEndpoint* endpoint : endpoints) {
        if (!endpoint->timing_path) {
            correctSetupTiming(*endpoint);
            continue;
        }
        endpoint->arrival_ns = context.evaluateInput(*endpoint->timing_path);
        endpoint->slack_ns = endpoint->required_ns - endpoint->arrival_ns;
        endpoint->critical_edges.clear();
        context.appendCriticalEdges(*endpoint->timing_path, endpoint->critical_edges);
    }
}

struct PlaceTimingPrepared::Impl
{
    static constexpr size_t none = std::numeric_limits<size_t>::max();
    struct Input {
        rtl::Conn* sink = nullptr;
        rtl::Conn* driver = nullptr;
        size_t output = none;
        double fanout_log2 = 0;
    };
    struct Output {
        std::vector<std::pair<size_t, double>> inputs;
        size_t evaluated = 0;
        size_t active = 0;
        double arrival = 0;
        size_t critical = none;
    };
    PlaceTiming& owner;
    std::vector<Input> inputs;
    std::vector<Output> outputs;
    std::unordered_map<clk::TimingPath*, size_t> input_ids;
    std::unordered_map<clk::TimingPath*, size_t> output_ids;
    size_t epoch = 0;

    explicit Impl(PlaceTiming& owner) : owner(owner) {}

    size_t prepareInput(clk::TimingPath& path)
    {
        if (auto found = input_ids.find(&path); found != input_ids.end())
            return found->second;
        const size_t id = inputs.size();
        input_ids.emplace(&path, id);
        inputs.emplace_back();
        Input value;
        value.sink = path.data_in;
        value.driver = followedDriver(path.data_in);
        if (value.driver) {
            size_t fanout = rtl::Conn::fromBase(*value.driver).getPeers().size();
            if (fanout > 1) value.fanout_log2 = std::log2(static_cast<double>(fanout));
            value.output = prepareOutput(path);
        }
        inputs[id] = value;
        return id;
    }

    size_t prepareOutput(clk::TimingPath& original)
    {
        auto* path = original.precalculated ? original.precalculated : &original;
        if (auto found = output_ids.find(path); found != output_ids.end())
            return found->second;
        const size_t id = outputs.size();
        output_ids.emplace(path, id);
        outputs.emplace_back();
        if (path->data_output) {
            AnalysisContext reference{owner};
            for (auto& branch : path->sub_paths) {
                if (!branch.data_in) continue;
                const size_t input = prepareInput(branch);
                // Recursion may grow outputs: do not retain a vector reference.
                outputs[id].inputs.emplace_back(input, reference.intrinsicDelay(*path, branch));
            }
        }
        return id;
    }

    double wireDelay(const Input& input) const
    {
        if (!input.sink || !input.driver || !input.sink->inst_ref.peer ||
            !input.driver->inst_ref.peer || !input.sink->inst_ref->tile.peer ||
            !input.driver->inst_ref->tile.peer) return 0;
        return owner.estimateWireDelay(*input.sink, *input.driver, 1) +
            input.fanout_log2 * owner.calibration.extra_fanout_ns;
    }

    double evaluateInput(size_t id)
    {
        const Input& input = inputs[id];
        if (!input.driver) return 0;
        return wireDelay(input) + evaluateOutput(input.output);
    }

    double evaluateOutput(size_t id)
    {
        Output& output = outputs[id];
        if (output.evaluated == epoch) return output.arrival;
        if (output.active == epoch) return 0;
        output.active = epoch;
        output.arrival = output.inputs.empty() ? 0 : -std::numeric_limits<double>::infinity();
        output.critical = none;
        for (auto [input, intrinsic] : output.inputs) {
            const double arrival = evaluateInput(input) + intrinsic;
            if (arrival > output.arrival) {
                output.arrival = arrival;
                output.critical = input;
            }
        }
        if (!std::isfinite(output.arrival)) output.arrival = 0;
        output.active = 0;
        output.evaluated = epoch;
        return output.arrival;
    }

    void evaluate(const std::vector<PlaceTimingEndpoint*>& endpoints)
    {
        // Compile every requested root before evaluation so the indexed graph
        // cannot reallocate during recursion. Cache contains topology only.
        for (auto* endpoint : endpoints)
            if (endpoint->timing_path) prepareInput(*endpoint->timing_path);
        if (++epoch == 0) {
            for (auto& output : outputs) output.evaluated = output.active = 0;
            ++epoch;
        }
        for (auto* endpoint : endpoints) {
            if (!endpoint->timing_path) {
                owner.correctSetupTiming(*endpoint);
                continue;
            }
            size_t id = input_ids.at(endpoint->timing_path);
            endpoint->arrival_ns = evaluateInput(id);
            endpoint->slack_ns = endpoint->required_ns - endpoint->arrival_ns;
            endpoint->critical_edges.clear();
            while (id != none) {
                const Input& input = inputs[id];
                if (!input.driver) break;
                endpoint->critical_edges.push_back({input.sink, input.driver,
                    input.sink->inst_ref.peer, input.driver->inst_ref.peer, wireDelay(input)});
                id = outputs[input.output].critical;
            }
        }
    }
};

PlaceTimingPrepared::PlaceTimingPrepared(PlaceTiming& owner)
    : impl(std::make_unique<Impl>(owner)) {}
PlaceTimingPrepared::~PlaceTimingPrepared() = default;
void PlaceTimingPrepared::evaluate(const std::vector<PlaceTimingEndpoint*>& endpoints)
{
    impl->evaluate(endpoints);
}

std::vector<rtl::Inst*> PlaceTiming::setupDependencies(
    const PlaceTimingEndpoint& endpoint) const
{
    std::unordered_set<rtl::Inst*> cells;
    std::unordered_set<const clk::TimingPath*> seen;
    auto account = [&](rtl::Conn* conn) {
        if (conn && conn->inst_ref.peer) cells.insert(conn->inst_ref.peer);
    };
    auto visit = [&](auto&& self, clk::TimingPath& path) -> void {
        account(path.data_in);
        account(followedDriver(path.data_in));
        clk::TimingPath* output = path.precalculated ? path.precalculated : &path;
        if (!seen.insert(output).second) return;
        account(output->data_output);
        if (output->data_output) {
            for (auto& input : output->sub_paths) self(self, input);
        }
    };
    account(endpoint.data_in);
    if (endpoint.timing_path) visit(visit, *endpoint.timing_path);
    else {
        for (const auto& edge : endpoint.critical_edges) {
            if (edge.driver) cells.insert(edge.driver);
            if (edge.sink) cells.insert(edge.sink);
        }
    }
    return {cells.begin(), cells.end()};
}

PlaceTimingIncremental::PlaceTimingIncremental(
    PlaceTiming& timing, PlaceTimingAnalysis& state) : owner(timing), analysis(state)
{
    for (size_t i = 0; i < analysis.endpoint_details.size(); ++i) {
        const auto& endpoint = analysis.endpoint_details[i];
        slacks.insert(endpoint.slack_ns);
        for (rtl::Inst* cell : owner.setupDependencies(endpoint))
            endpoints_by_cell[cell].push_back(i);
    }
}

void PlaceTimingIncremental::removeSlack(double slack)
{
    slacks.erase(slacks.find(slack));
    if (slack < 0) {
        --analysis.violated_endpoints;
        analysis.total_negative_slack_ns += slack;
    }
}

void PlaceTimingIncremental::addSlack(double slack)
{
    slacks.insert(slack);
    if (slack < 0) {
        ++analysis.violated_endpoints;
        analysis.total_negative_slack_ns -= slack;
    }
    analysis.worst_slack_ns = *slacks.begin();
}

PlaceTimingIncremental::Transaction PlaceTimingIncremental::update(
    const std::vector<rtl::Inst*>& changed)
{
    Transaction transaction;
    refresh(changed, &transaction);
    return transaction;
}

void PlaceTimingIncremental::updateForward(const std::vector<rtl::Inst*>& changed)
{
    refresh(changed, nullptr);
}

void PlaceTimingIncremental::refresh(
    const std::vector<rtl::Inst*>& changed, Transaction* transaction)
{
    std::vector<size_t> indices;
    for (rtl::Inst* cell : changed) {
        auto found = endpoints_by_cell.find(cell);
        if (found != endpoints_by_cell.end())
            indices.insert(indices.end(), found->second.begin(), found->second.end());
    }
    std::ranges::sort(indices);
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    std::vector<PlaceTimingEndpoint*> endpoints;
    if (transaction) transaction->reserve(indices.size());
    endpoints.reserve(indices.size());
    for (size_t index : indices) {
        auto& endpoint = analysis.endpoint_details[index];
        if (transaction) transaction->push_back({index, endpoint});
        removeSlack(endpoint.slack_ns);
        endpoints.push_back(&endpoint);
    }
    owner.evaluateSetupTiming(endpoints);
    for (auto* endpoint : endpoints) addSlack(endpoint->slack_ns);
}

void PlaceTimingIncremental::restore(Transaction&& transaction)
{
    for (auto& snapshot : transaction) {
        auto& endpoint = analysis.endpoint_details[snapshot.index];
        removeSlack(endpoint.slack_ns);
        endpoint = std::move(snapshot.endpoint);
        addSlack(endpoint.slack_ns);
    }
}

double PlaceTimingIncremental::minimumSlack(
    const std::vector<rtl::Inst*>& cells) const
{
    double slack = std::numeric_limits<double>::infinity();
    for (rtl::Inst* cell : cells) {
        auto found = endpoints_by_cell.find(cell);
        if (found != endpoints_by_cell.end()) {
            for (size_t index : found->second)
                slack = std::min(slack, analysis.endpoint_details[index].slack_ns);
        }
    }
    return slack;
}
