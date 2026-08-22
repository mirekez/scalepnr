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
};

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
        if (!clock) {
            continue;
        }
        for (auto& info : infos) {
            if (!info.path.data_in) {
                continue;
            }
            PlaceTimingEndpoint endpoint;
            endpoint.clock = clock;
            endpoint.data_in = info.data_in ? info.data_in : info.path.data_in;
            endpoint.required_ns = info.setup_limit > 0
                ? info.setup_limit : clock->period_ns;
            endpoint.arrival_ns = context.evaluateInput(info.path);
            endpoint.slack_ns = endpoint.required_ns - endpoint.arrival_ns;
            if (context.analysis.endpoints == 0) {
                context.analysis.worst_slack_ns = endpoint.slack_ns;
            }
            else {
                context.analysis.worst_slack_ns = std::min(
                    context.analysis.worst_slack_ns, endpoint.slack_ns);
            }
            ++context.analysis.endpoints;
            if (endpoint.slack_ns < 0) {
                ++context.analysis.violated_endpoints;
                context.analysis.total_negative_slack_ns -= endpoint.slack_ns;
                context.appendCriticalEdges(info.path, endpoint.critical_edges);
                context.addEndpointForces(endpoint);
            }
            context.analysis.endpoint_details.push_back(std::move(endpoint));
        }
    }

    context.analysis.forces.reserve(context.force_by_inst.size());
    for (auto& [inst, accumulator] : context.force_by_inst) {
        (void) inst;
        context.analysis.forces.push_back(accumulator.force);
    }
    std::ranges::sort(context.analysis.forces,
        [](const PlaceTimingForce& left, const PlaceTimingForce& right) {
            if (left.weight != right.weight) {
                return left.weight > right.weight;
            }
            if (left.inst && right.inst && left.inst->coord.y != right.inst->coord.y) {
                return left.inst->coord.y < right.inst->coord.y;
            }
            if (left.inst && right.inst && left.inst->coord.x != right.inst->coord.x) {
                return left.inst->coord.x < right.inst->coord.x;
            }
            if (left.inst && right.inst) {
                return left.inst->makeName() < right.inst->makeName();
            }
            return left.inst != nullptr;
        });

    context.analysis.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    return std::move(context.analysis);
}
