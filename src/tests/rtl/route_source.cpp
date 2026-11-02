#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "RouteSource.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <stdexcept>
#include <string_view>

namespace {
bool count_allocations = false;
size_t allocations = 0, allocated_bytes = 0;
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
}

// Count actual heap requests made by source identity, excluding fixture setup.
// This executable alone replaces new; production binaries are unaffected.
void* operator new(size_t size) {
    if (count_allocations) { ++allocations; allocated_bytes += size; }
    if (void* memory = std::malloc(size ? size : 1)) return memory;
    throw std::bad_alloc{};
}
[[gnu::noinline]] void operator delete(void* memory) noexcept { std::free(memory); }
[[gnu::noinline]] void operator delete(void* memory, size_t) noexcept { std::free(memory); }
void* operator new[](size_t size) { return ::operator new(size); }
[[gnu::noinline]] void operator delete[](void* memory) noexcept { std::free(memory); }
[[gnu::noinline]] void operator delete[](void* memory, size_t) noexcept { std::free(memory); }

int main(int argc, char** argv) {
    try {
        const bool reproduce = argc > 1 && std::string_view(argv[1]) == "--reproduce";
        std::array<Referable<rtl::Cell>, 4> cells;
        std::array<Referable<rtl::Inst>, 4> instances;
        std::array<Referable<rtl::Port>, 4> outputs;
        std::array<Referable<rtl::Port>, 2> inputs;
        for (size_t i = 0; i < instances.size(); ++i) {
            cells[i].name = "source_" + std::to_string(i);
            instances[i].cell_ref.set(&cells[i]);
            outputs[i].name = "Q";
            outputs[i].type = rtl::Port::PORT_OUT;
            instances[i].conns.resize(i < 2 ? 1 : 2);
            instances[i].conns[0].inst_ref.set(&instances[i]);
            instances[i].conns[0].port_ref.set(&outputs[i]);
            if (i >= 2) {
                cells[i].attributes["scalepnr_passthrough"] = "source";
                inputs[i - 2].name = "I";
                inputs[i - 2].type = rtl::Port::PORT_IN;
                instances[i].conns[1].inst_ref.set(&instances[i]);
                instances[i].conns[1].port_ref.set(&inputs[i - 2]);
                instances[i].conns[1].set(&instances[i == 2 ? 0 : 2].conns[0]);
            }
        }
        const std::string port = "Q", other_port = "D";
        // Initialize any immutable lookup key outside the measured loop.
        require(rtl::sameRouteSource(&instances[0], port, &instances[3], port),
                "two live source helpers did not resolve to their driver");
        constexpr size_t comparisons = 10000;
        for (int mode = 0; mode < 4; ++mode) {
            // Mutations between batches must be visible immediately; no alias
            // identity cache may hide a rewired helper or changed attribute.
            instances[2].conns[1].set(&instances[mode == 1 ? 1 : 0].conns[0]);
            cells[2].attributes["scalepnr_passthrough"] = mode == 2 ? "other" : "source";
            bool correct = true;
            allocations = allocated_bytes = 0;
            count_allocations = true;
            for (size_t i = 0; i < comparisons; ++i) {
                correct &= !rtl::sameRouteSource(&instances[0], port, &instances[1], port);
                correct &= rtl::sameRouteSource(&instances[0], port, &instances[3], port) ==
                           (mode == 0 || mode == 3);
                correct &= !rtl::sameRouteSource(&instances[0], other_port, &instances[3], port);
            }
            count_allocations = false;
            std::printf("SOURCE_IDENTITY mode=%d comparisons=%zu allocations=%zu bytes=%zu\n",
                        mode, comparisons * 3, allocations, allocated_bytes);
            require(correct, "source comparison ignored a live alias, port, or attribute change");
            require(reproduce ? allocations >= comparisons * 2 : allocations == 0,
                    "source comparison allocated a temporary lookup key");
        }
        require(!rtl::sameRouteSource(nullptr, port, &instances[0], port) &&
                !rtl::sameRouteSource(&instances[0], "", &instances[3], port),
                "invalid endpoint was accepted");
        return 0;
    } catch (const std::exception& error) {
        count_allocations = false;
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
