#include "Device.h"
#include "RegBunch.h"
#include "TimingPath.h"
#include "Cell.h"
#include "Module.h"
#include "PinLookupCache.h"
#include "route/RouteDiagnostics.h"

#include <algorithm>
#include <chrono>
#include <format>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>

namespace {
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

struct PackingFixture {
    fpga::TileType model{"RESOURCE", 1, 0};
    std::unique_ptr<fpga::CBType> cb = std::make_unique<fpga::CBType>("MATRIX");
    Referable<rtl::Module> parent, primitive;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    Referable<rtl::Inst>* candidate;
    fpga::Tile* tile;

    static int position(int bit) {
        return (bit / 8) * 128 + (bit % 8 >= 4 ? 64 : 0) + (bit % 4) * 4;
    }
    Referable<rtl::Inst>* make(const std::string& name, bool driver) {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name;
        cell->type = driver ? "DRIVER" : "FD_SAMPLE";
        cell->module_ref.set(&primitive);
        rtl::Port port;
        port.name = driver ? "OUT" : "SELECT";
        port.type = driver ? rtl::Port::PORT_OUT : rtl::Port::PORT_IN;
        cell->ports.emplace_back(std::move(port));
        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(cell.get());
        inst->pos = -1;
        inst->cnt_inputs = !driver;
        inst->cnt_outputs = driver;
        auto& conn = inst->conns.emplace_back();
        conn.inst_ref.set(inst.get());
        conn.port_ref.set(&cell->ports.front());
        auto* result = inst.get();
        cells.push_back(std::move(cell));
        insts.push_back(std::move(inst));
        return result;
    }
    PackingFixture() {
        parent.name = "top";
        primitive.name = "primitive";
        primitive.is_blackbox = true;
        primitive.parent_ref.set(&parent);
        for (int site = 0; site < 2; ++site) {
            model.sites.push_back(fpga::SiteModel{.name = "BANK" + std::to_string(site),
                                                 .type = "LOGIC", .pos = site});
            // Realistic endpoint-table size, with only SELECT sharing a resource.
            for (int pin = 0; pin < 96; ++pin) {
                int resource = site * 256 + pin;
                model.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, resource,
                    "PORT_" + std::to_string(pin));
                model.pin_map.input_nodes[resource].setBit(100 + pin);
            }
            int resource = site * 256 + 128;
            model.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, resource, "SELECT");
            model.pin_map.input_nodes[resource].setBit(43 + site);
            model.pin_map.rememberEndpointRouteRef(fpga::TILE_PIN_INPUT, resource,
                                                  43 + site, "MATRIX");
        }
        for (int bit = 0; bit < 16; ++bit) {
            fpga::Element element;
            element.name = "REGISTER_" + std::to_string(bit);
            element.type = fpga::ELEMENT_FD;
            element.bitmap_pos = bit;
            element.elements_to_left = 0;
            model.elements.push_back(element);
        }
        auto& device = fpga::Device::current();
        device.tile_grid.clear();
        device.tile_grid.resize(1);
        device.size_width = device.size_height = 1;
        device.grid_spec.size = {1, 1};
        tile = &device.tile_grid.front();
        tile->coord = tile->cb_coord = {0, 0};
        tile->tile_type = &model;
        tile->cb_type = cb.get();
        tile->cb.type = cb.get();
        for (int i = 0; i < 3; ++i) {
            auto* source = make("driver_" + std::to_string(i), true);
            auto* sink = make("load_" + std::to_string(i), false);
            source->conns[0].port_ref->designator = i + 1;
            sink->conns[0].port_ref->designator = i + 1;
            sink->conns[0].set(&source->conns[0]);
            auto& net = parent.nets.emplace_back();
            net.name = "control_" + std::to_string(i);
            net.designators.push_back(i + 1);
            if (i < 2) require(tile->tryAddAt(sink, position(i * 8), false) >= 0,
                               "failed to place reference owner");
            else candidate = sink;
        }
    }
    ~PackingFixture() { fpga::Device::current().tile_grid.clear(); }

    size_t probe(int iterations, bool cached) {
        std::optional<fpga::PinLookupCache> cache;
        if (cached) cache.emplace();
        auto free_before = tile->elements_free;
        size_t rejects = 0;
        for (int i = 0; i < iterations; ++i) {
            for (int bit = 0; bit < 16; ++bit) {
                require(tile->tryAddAt(candidate, position(bit), false) < 0,
                        "independent drivers packed onto the same input");
                ++rejects;
            }
        }
        require(!candidate->tile.peer && tile->elements_free == free_before,
                "rejected packing changed occupancy");
        if (cached) require(cache->hits > 0, "packing did not use immutable pin cache");
        return rejects;
    }
    void checkLiveOccupancy() {
        fpga::PinLookupCache cache;
        require(tile->tryAddAt(candidate, position(1), false) < 0, "missing input conflict");
        // Keep the lookup cache warm while the actual reservation owner changes.
        auto* owner = insts[1].get();
        tile->unassign(owner);
        require(tile->tryAddAt(candidate, position(1), false) >= 0,
                "cached mapping retained a released occupancy conflict");
        tile->unassign(candidate);
        require(tile->tryAddAt(owner, position(0), false) >= 0, "failed to restore owner");
        require(tile->tryAddAt(candidate, position(1), false) < 0,
                "cached mapping ignored a new occupancy conflict");
    }
};

struct AuditFixture {
    rtl::Inst storage;
    std::vector<std::unique_ptr<Referable<rtl::Net>>> objects;
    std::vector<rtl::Net*> nets;
    fpga::Tile tile;

    explicit AuditFixture(int count) {
        tile.coord = {0, 0};
        tile.full_name = "TEST_TILE";
        storage.wires.resize(count);
        for (int n = 0; n < count; ++n) {
            auto net = std::make_unique<Referable<rtl::Net>>();
            net->name = "signal_" + std::to_string(n);
            for (int f = 0; f < 16; ++f) {
                fpga::Wire w;
                w.from = {n % 64, n / 64 + f};
                w.to = {n % 64, n / 64 + f + 1};
                w.local = f;
                w.pos = 1;
                w.jump = 100 + f;
                w.dst = f + 1;
                w.owns_landing = f == 15;
                storage.wires[n].push_back(w);
            }
            rtl::NetRouteBinding binding;
            binding.owner = &storage;
            binding.route_index = n;
            binding.route_name = "route_" + std::to_string(n);
            net->routes.push_back(binding);
            nets.push_back(net.get());
            objects.push_back(std::move(net));
        }
        tile.cb.src.jump.setBit(100);
        tile.cb.src.jump.setBit(999); // Deliberate orphan, independently detected.
    }
    void check() {
        fpga::CongestionAuditIndex index(nets);
        for (Coord coord : {Coord{0, 0}, Coord{0, 1}, Coord{3, 5}, Coord{-1, -1}}) {
            tile.coord = coord;
            std::ostringstream reference, indexed;
            fpga::auditTileCongestion(tile, reference, nets);
            fpga::auditTileCongestion(tile, indexed, index);
            require(reference.str() == indexed.str(), "indexed audit differs from exhaustive proof");
        }
        tile.coord = {0, 0};
        tile.cb.src.jump = {};
        std::ostringstream reference, indexed;
        fpga::auditTileCongestion(tile, reference, nets);
        fpga::auditTileCongestion(tile, indexed, index);
        require(reference.str() == indexed.str(), "audit cached live lease masks");
        std::ostringstream trace_output;
        pnr::RouteCongestionTrace trace("signal", "test", nets, &trace_output);
        trace.blocked(tile, "before", {});
        Referable<rtl::Net> empty;
        const auto revision = fpga::routeMutationRevision();
        require(!fpga::unrouteNetRoute(empty, 0), "empty unroute unexpectedly succeeded");
        require(fpga::routeMutationRevision() == revision,
                "a failed unroute invalidated an unchanged ownership index");
        trace.blocked(tile, "unchanged-after-failed-unroute", {});
        require(trace_output.str().find("SNAPSHOT_REF") != std::string::npos,
                "failed unroute discarded an unchanged diagnostic snapshot");
        storage.wires[0][0].jump = 123;
        fpga::registerNetRouteTilesFrom(*nets[0], storage.wires[0], 0, 0);
        trace.blocked(tile, "after", {});
        require(trace_output.str().find("id=123") != std::string::npos,
                "registered route retained a stale audit index");
        storage.wires[0][0].from = {-4, -4};
        storage.wires[0][0].to = {-4, -4};
        tile.coord = {-4, -4};
        trace.search(tile, tile, 0, 5);
        trace.blocked(tile, "new-search", {});
        auto output = trace_output.str();
        require(output.substr(output.find("reason=new-search")).find("id=123") != std::string::npos,
                "search boundary missed newly touched coordinates");
    }
    void randomClaims() {
        std::mt19937 random(64219);
        for (int repeat = 0; repeat < 10; ++repeat) {
            for (auto& route : storage.wires) for (auto& wire : route) {
                wire.from = {int(random() % 5) - 2, int(random() % 5) - 2};
                wire.to = {int(random() % 5) - 2, int(random() % 5) - 2};
                wire.type = static_cast<fpga::Wire::Type>(random() % 3);
                wire.jump = random() % 32;
                wire.local = random() % 32;
                wire.dst = random() % 32;
                wire.joint = random() % 32;
                wire.shared = random() % 2;
                wire.owns_dst = random() % 2;
                wire.owns_landing = random() % 2;
                wire.from_node_type = fpga::CB_NODE_SRC;
                wire.from_node = random() % 32;
                wire.to_node_type = fpga::CB_NODE_DST;
                wire.to_node = random() % 32;
            }
            // Mix genuine, missing, and out-of-scope registrations. The index
            // must never use the Tile registry as proof of actual ownership.
            tile.addRoutedBinding(objects[0].get(), objects[0]->routeId(0));
            Referable<rtl::Net> outside;
            tile.addRoutedNet(&outside);
            fpga::CongestionAuditIndex index(nets);
            for (int x = -2; x <= 2; ++x) {
                tile.coord = {x, int(random() % 5) - 2};
                tile.cb.dst.jump.setBit(random() % 32);
                std::ostringstream reference, indexed;
                fpga::auditTileCongestion(tile, reference, nets);
                fpga::auditTileCongestion(tile, indexed, index);
                require(reference.str() == indexed.str(), "random ownership proof differs");
            }
            tile.removeRoutedNet(&outside);
        }
    }
};

class CountingBuffer : public std::streambuf {
 public:
    size_t bytes = 0;
    std::streamsize xsputn(const char*, std::streamsize n) override { bytes += n; return n; }
    int overflow(int c) override { if (c != traits_type::eof()) ++bytes; return traits_type::not_eof(c); }
};

template<class Operation> double measure(Operation&& operation) {
    std::vector<double> samples;
    for (int repeat = 0; repeat < 5; ++repeat) {
        auto start = std::chrono::steady_clock::now();
        operation();
        samples.push_back(std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count());
    }
    std::sort(samples.begin(), samples.end());
    return samples[2];
}

void report(const char* name, double before, double after) {
    std::cout << std::format("{} baseline_ms={:.3f} optimized_ms={:.3f} improvement={:.1f}%\n",
                             name, before, after, 100 * (1 - after / before));
    require(after <= before * 0.80, "optimization did not meet the 20% benchmark threshold");
}

// Keep the sink opaque to the benchmark caller, as it is in the router where
// diagnostic enablement comes from the environment and may change for replay.
#if defined(__GNUC__)
__attribute__((noinline))
#endif
void diagnosticProbe(const pnr::PackingTrace& trace, bool lazy, int i,
                     const std::string& name) {
    auto format = [&] { return std::format(
        "PACK_POSITION tile=({},{}) member={} pos={} depth={}",
        i % 100, i % 91, name, i % 16, 2); };
    if (lazy) trace(format);
    else trace(format());
}

void diagnostics(bool benchmark) {
    std::ofstream replay;
    pnr::PackingTrace disabled(nullptr, replay);
    int formatted = 0;
    disabled([&] { ++formatted; return std::string("must not execute"); });
    require(formatted == 0, "disabled diagnostic formatted its message");
    std::unique_ptr<std::FILE, decltype(&std::fclose)> file(std::tmpfile(), &std::fclose);
    require(bool(file), "cannot create trace capture");
    pnr::PackingTrace enabled(file.get(), replay);
    enabled([&] { ++formatted; return std::string("formatted"); });
    enabled("literal");
    std::rewind(file.get());
    char captured[64] = {};
    std::fread(captured, 1, sizeof(captured) - 1, file.get());
    require(formatted == 1 && std::string(captured) == "formatted\nliteral\n",
            "lazy trace changed enabled output");
    if (!benchmark) return;
    std::string name = "generated_cell_4567";
    double before = measure([&] {
        for (int i = 0; i < 500000; ++i) diagnosticProbe(disabled, false, i, name);
    });
    double after = measure([&] {
        for (int i = 0; i < 500000; ++i) diagnosticProbe(disabled, true, i, name);
    });
    report("disabled_diagnostics", before, after);
}
}

int main(int argc, char** argv) {
    try {
        bool benchmark = argc == 2 && std::string_view(argv[1]) == "--benchmark";
        PackingFixture packing;
        require(packing.probe(2, false) == packing.probe(2, true), "packing decisions differ");
        packing.checkLiveOccupancy();
        diagnostics(benchmark);
        if (benchmark) {
            double before = measure([&] { packing.probe(1000, false); });
            double after = measure([&] { packing.probe(1000, true); });
            report("packing_pin_cache", before, after);
        }
        AuditFixture audit(benchmark ? 12000 : 256);
        audit.check();
        if (!benchmark) audit.randomClaims();
        if (benchmark) {
            CountingBuffer buffer;
            std::ostream output(&buffer);
            auto batch = [&](const fpga::CongestionAuditIndex* index) {
                for (int n = 0; n < 32; ++n) {
                    audit.tile.coord = {n, n};
                    if (index) fpga::auditTileCongestion(audit.tile, output, *index);
                    else fpga::auditTileCongestion(audit.tile, output, audit.nets);
                }
            };
            double before = measure([&] { batch(nullptr); });
            // Include construction cost once per search, not just lookup time.
            double after = measure([&] { fpga::CongestionAuditIndex index(audit.nets); batch(&index); });
            report("ownership_audit_batch", before, after);
            require(buffer.bytes > 0, "audit output was omitted");
        }
        std::cout << "packing_perf correctness checks passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "packing_perf: " << e.what() << '\n';
        return 1;
    }
}
