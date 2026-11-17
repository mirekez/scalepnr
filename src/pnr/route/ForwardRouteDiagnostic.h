#pragma once

#include "Tile.h"

#include <functional>
#include <initializer_list>
#include <ostream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fpga
{
struct TileJumpTarget;
struct TileLocalTarget;
} // namespace fpga

namespace pnr
{

struct RoutingAnchor;
struct TraceRouteForward;

// Optional observation and logging for one forward route attempt.
// Routing decisions never read its audit history.
struct ForwardRouteDiagnostic
{
    using Saved = std::vector<std::pair<fpga::Tile *, fpga::CBState>>;
    using SavedIndex = std::unordered_map<fpga::Tile *, size_t>;
    using Observer = std::function<void(const Saved &)>;

    // Describe one angle-ranked exit in the opt-in audit.
    struct Source
    {
        int src = -1;
        int joint = -1, joint2 = -1;
        bool preexisting_deadend = false;
        bool missing_topology = false;
        bool source_busy = false, destination_busy = false, joint_busy = false;
        bool free = false;
        size_t targets = 0, blocked_targets = 0, descents = 0;
    };

    // Identify one typed node in repeated-visit diagnostics.
    struct Key
    {
        fpga::Tile *tile;
        fpga::CBNodeNameType type;
        int node;

        bool operator==(const Key &other) const
        {
            return tile == other.tile && type == other.type &&
                   node == other.node;
        }

    };

    struct KeyHash
    {
        size_t operator()(const Key &key) const
        {
            size_t value = std::hash<fpga::Tile *>{}(key.tile);
            value ^= static_cast<size_t>(key.type) + 0x9e3779b9U +
                     (value << 6) + (value >> 2);
            value ^= static_cast<size_t>(key.node) + 0x9e3779b9U +
                     (value << 6) + (value >> 2);
            return value;
        }

    };

    struct Seen
    {
        size_t first = 0, visits = 0;
        long long first_parent = -1;
    };

    // Per-recursion-call audit choices, separate from the routing cursor.
    struct Step
    {
        std::vector<Source> sources;
        bool reported = false;
    };

    // Keep the audit path aligned with recursive calls.
    struct PathScope
    {
        ForwardRouteDiagnostic *diagnostic;

        PathScope(ForwardRouteDiagnostic *diagnostic, fpga::Tile &tile,
                  fpga::CBNodeNameType type, int node, size_t cursor)
            : diagnostic(diagnostic && diagnostic->audit ? diagnostic : nullptr)
        {
            if (this->diagnostic)
                this->diagnostic->path.emplace_back(Key{&tile, type, node},
                                                    cursor);
        }

        ~PathScope()
        {
            if (diagnostic)
                diagnostic->path.pop_back();
        }

    };

    // Restore search leases after temporarily showing the original snapshot.
    struct RestoreState
    {
        fpga::Tile &tile;
        const fpga::CBState &state;

        ~RestoreState()
        {
            tile.cb.restoreLeases(state);
        }

    };

    std::ostream *decisions = nullptr;
    std::ostream *audit = nullptr;
    const Observer *observer = nullptr;
    Saved saved;
    SavedIndex saved_index;
    std::vector<std::pair<Key, size_t>> path;
    std::unordered_map<Key, Seen, KeyHash> seen;
    size_t toward = 0, away = 0, level = 0;
    size_t jump_prints = 0, source_reports = 0;
    size_t speculative_deadend_skips = 0;
    size_t max_same_tile_frames = 0;
    bool tile_chain_printed = false;

    // Report one visited node and any resulting terminal decision.
    void attach_deadline_stream();
    void attempt(size_t index);
    void begin_anchor(const RoutingAnchor &anchor);
    void finish_anchor(size_t queued, const TraceRouteForward &trace);
    void reject(size_t cursor);
    void state(size_t cursor, long long parent, fpga::Tile &tile,
               fpga::CBNodeNameType type, int node);
    void grounded(size_t cursor, bool local);
    void source_count(size_t cursor, size_t count);
    void target(size_t cursor, int src, const fpga::TileJumpTarget &target);
    void descend(size_t parent, size_t child);
    void edge(size_t cursor, fpga::Tile &tile, int src, int joint, int joint2,
              bool free);
    void target_count(size_t cursor, int src, size_t count);
    void local_target(size_t cursor, const fpga::TileLocalTarget &target,
                      bool busy);
    void local(size_t cursor, fpga::Tile &tile, int node);
    bool remember(fpga::Tile &tile, TraceRouteForward &trace);
    void forget(fpga::Tile &tile, bool added);
    void before_probe(fpga::CBNodeNameType type, bool has_local_probe);

    // Track opt-in audit observations without changing search state.
    void enter(fpga::Tile &tile, fpga::CBNodeNameType type, int node,
               fpga::Coord aim, size_t cursor, long long parent, size_t depth,
               size_t same_tile_depth, const TraceRouteForward &trace);
    void jump(fpga::Coord aim, fpga::Tile &from, fpga::Tile &to, size_t cursor,
              int src, size_t source_choice, size_t depth);
    void source_start(Step &step, int src);
    void preexisting_deadend(Step &step);
    void blocked_target(Step &step);
    void target_descent(Step &step);
    void source_targets(Step &step, size_t count);
    void source_choice(Step &step, fpga::Tile &tile, fpga::CBNodeNameType type,
                       int dst, int src, int joint, int joint2, bool free,
                       bool root);
    void source_outcomes(Step &step, fpga::Tile &tile,
                         fpga::CBNodeNameType type, int dst, fpga::Coord aim,
                         size_t cursor, size_t depth, size_t source_count);
    void speculative_skip(fpga::Tile &tile, int src, size_t depth,
                          size_t cursor);
    static void audit_candidates(fpga::CBType &type,
                                 fpga::CBNodeNameType from_type, int from_node,
                                 fpga::Coord angle, const NodeMask &allowed,
                                 const std::vector<uint16_t> &active,
                                 const std::vector<uint16_t> &unresolved);
    void
    blocker(fpga::Tile &tile, const char *reason,
            std::initializer_list<std::pair<fpga::CBNodeNameType, int>> nodes,
            TraceRouteForward &trace);

    static int distance(fpga::Coord aim, fpga::Coord point);
    static void describe(std::ostream *out, fpga::Tile &tile,
                         fpga::CBNodeNameType type, int id);
};

} // namespace pnr
