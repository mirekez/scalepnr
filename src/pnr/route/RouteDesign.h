#pragma once

#include "Design.h"
#include "RegBunch.h"
#include "TileSet.h"
#include "Inst.h"
#include "Clocks.h"
#include "png_draw.h"
#include "Device.h"

#include <array>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

namespace technology
{
    struct Tech;
}

namespace pnr
{

struct RouteDesign
{
    technology::Tech* tech = nullptr;
    fpga::Device* fpga = nullptr;

    static constexpr const int mesh_width = 10;
    static constexpr const int mesh_height = 10;

    int fpga_width = 0;
    int fpga_height = 0;

    float aspect_x = 0;
    float aspect_y = 0;

    float image_zoom = 4;

    uint64_t travers_mark = 0;
    int iteration_limit = 1;
    int move_attempt_limit = 16;
    int route_iteration_budget = 0;
    int route_recursion_budget = 0;
    bool route_changed = false;
    bool route_progress = false;
    std::unordered_map<uint64_t, u256> route_dst_deadends;
    std::unordered_map<uint64_t, u256> route_src_deadends;
    struct RouteStats {
        static constexpr size_t max_depth = 8;
        size_t task_attempts = 0;
        size_t new_attempts = 0;
        size_t continuation_attempts = 0;
        size_t already_complete = 0;
        size_t new_completed = 0;
        size_t new_partial = 0;
        size_t new_empty = 0;
        size_t new_failed = 0;
        size_t cont_completed = 0;
        size_t cont_advanced = 0;
        size_t cont_no_advance = 0;
        size_t cont_failed_rip = 0;
        size_t cont_failed_empty = 0;
        size_t completed = 0;
        size_t partial_started = 0;
        size_t partial_advanced = 0;
        size_t rip_backs = 0;
        size_t backstep_attempts = 0;
        size_t backstep_success = 0;
        size_t backstep_fragments = 0;
        size_t commit_rollbacks = 0;
        size_t dst_deadend_marks = 0;
        size_t src_deadend_marks = 0;
        size_t failed = 0;
        size_t route_searches = 0;
        size_t search_pops = 0;
        size_t pops_on_deadend_tile = 0;
        size_t pops_on_dst_deadend_tile = 0;
        size_t pops_on_src_deadend_tile = 0;
        size_t edge_trials = 0;
        size_t edge_accepted = 0;
        size_t edge_rejected_no_name = 0;
        size_t edge_rejected_busy = 0;
        size_t edge_rejected_busy_dst = 0;
        size_t edge_rejected_busy_src = 0;
        size_t edge_rejected_busy_local = 0;
        size_t edge_rejected_no_target = 0;
        size_t edge_rejected_deadend = 0;
        size_t edge_rejected_dst_deadend = 0;
        size_t edge_rejected_src_deadend = 0;
        size_t no_src_nodes = 0;
        size_t no_src_nodes_depth0 = 0;
        size_t no_src_nodes_with_joint_path = 0;
        size_t preempt_attempts = 0;
        size_t preempt_success = 0;
        bool has_last_busy = false;
        fpga::Coord last_busy_coord;
        int last_busy_depth = 0;
        int last_busy_local = -1;
        int last_busy_src = -1;
        u256 last_busy_src_mask{};
        u256 last_busy_dst_mask{};
        u256 last_busy_local_mask{};
        bool has_last_no_src = false;
        fpga::Coord last_no_src_coord;
        int last_no_src_depth = 0;
        int last_no_src_local = -1;
        u256 last_no_src_joint_mask{};
        bool has_last_deadend_mark = false;
        fpga::Coord last_dst_deadend_coord;
        int last_dst_deadend_node = -1;
        fpga::Coord last_src_deadend_coord;
        int last_src_deadend_node = -1;
        std::string last_deadend_net;
        std::array<size_t, max_depth> pops_by_depth{};
        std::array<size_t, max_depth> trials_by_depth{};
        std::array<size_t, max_depth> accepted_by_depth{};
        std::array<size_t, max_depth> backsteps_by_depth{};
        std::array<size_t, max_depth> rollbacks_by_depth{};
        std::array<size_t, max_depth> deadends_by_depth{};
        std::array<size_t, max_depth> partial_by_depth{};
        std::array<size_t, max_depth> completed_by_depth{};

        void clear();
    };
    RouteStats route_stats;
    struct RouteTask {
        rtl::Inst* from = nullptr;
        rtl::Inst* to = nullptr;
        rtl::Net* net = nullptr;
        std::string from_port;
        std::string to_port;
        std::string net_name;
        size_t attempt = 0;
        bool fanout = false;
    };
    std::vector<RouteTask> route_todo;
    std::vector<RouteTask> pending_route_todo;
    std::vector<RouteTask> fanout_route_todo;
    std::vector<RouteTask> moving_deferred_todo;
    bool fanout_stage = false;
    bool moving_stage = false;
    rtl::Inst* moving_focus_inst = nullptr;
    std::unordered_map<uintptr_t, std::vector<uint64_t>> move_tried_placements;
    std::unordered_set<uintptr_t> move_finished_insts;
    uint64_t source_route_mark = 0;
    void collectRouteTasks(rtl::Inst& inst, RegBunch* bunch = nullptr);
    bool routeNetTask(RouteTask& task, int depth = 0);
    bool routeFanoutTask(RouteTask& task, int depth = 0);
    bool routeInstTask(rtl::Inst& inst, int depth = 0);
    void routeDesign(std::list<Referable<RegBunch>>& bunch_list);
    void recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, bool place, int depth = 0);
    bool routeNet(rtl::Inst& from, const std::string& from_port, rtl::Inst& to, const std::string& to_port, std::vector<Wire>& wire, bool& complete, size_t attempt = 0, rtl::Net* net = nullptr);
    bool routeNet(rtl::Inst& from, rtl::Inst& to, const std::string& to_port, std::vector<Wire>& wire);
    bool routeNet(rtl::Inst& from, rtl::Inst& to, std::vector<Wire>& wire);
    bool tryNext(Tile& from, Tile& to, int from_pos, int to_pos, const std::string& to_port, std::vector<Wire>& wire, int depth = 0, rtl::Inst* dst_inst_override = nullptr);
    bool enqueueRouteTask(const RouteTask& task, std::vector<RouteTask>& queue);
    void requeueNet(rtl::Net& net, bool fanout = false);
    bool moveUnfinishedCell(const RouteTask& task, std::vector<RouteTask>* moved_tasks = nullptr, const RouteTask* trigger_task = nullptr);

    png_draw image;
};

}
