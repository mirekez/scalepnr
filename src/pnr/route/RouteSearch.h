#pragma once

#include "Device.h"
#include "Tile.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace pnr {

constexpr uint8_t COMBINATORIAL_TILE_VISIT_LIMIT = 2;

// Store exact path-local tile visit counts in a persistent numeric tree.
// Alternative search branches share their unchanged history without copying.
class CombinatorialTileVisits {
  struct Entry {
    int left = -1;
    int right = -1;
    uint8_t count = 0;
  };

public:
  CombinatorialTileVisits()
      : width(std::max(1, fpga::Device::current().size_width)),
        tile_count(std::max(1, width *
                                   std::max(1, fpga::Device::current()
                                                   .size_height))) {}

  // Append one tile to a candidate path and reject its third occurrence.
  bool append(int root, const fpga::Tile *tile, int &next_root) {
    int key = tileKey(tile);
    if (key < 0 || value(root, 0, tile_count - 1, key) >=
                       COMBINATORIAL_TILE_VISIT_LIMIT) {
      return false;
    }
    next_root = increment(root, 0, tile_count - 1, key);
    return true;
  }

private:
  int tileKey(const fpga::Tile *tile) const {
    if (!tile || tile->coord.x < 0 || tile->coord.x >= width ||
        tile->coord.y < 0) {
      return -1;
    }
    int key = tile->coord.y * width + tile->coord.x;
    return key < tile_count ? key : -1;
  }

  uint8_t value(int root, int low, int high, int key) const {
    if (root < 0) {
      return 0;
    }
    if (low == high) {
      return entries[static_cast<size_t>(root)].count;
    }
    int middle = low + (high - low) / 2;
    return key <= middle
               ? value(entries[static_cast<size_t>(root)].left, low, middle,
                       key)
               : value(entries[static_cast<size_t>(root)].right, middle + 1,
                       high, key);
  }

  int increment(int root, int low, int high, int key) {
    int copy = static_cast<int>(entries.size());
    entries.push_back(root >= 0 ? entries[static_cast<size_t>(root)] : Entry{});
    if (low == high) {
      ++entries[static_cast<size_t>(copy)].count;
      return copy;
    }
    int middle = low + (high - low) / 2;
    if (key <= middle) {
      int child = increment(entries[static_cast<size_t>(copy)].left, low,
                            middle, key);
      entries[static_cast<size_t>(copy)].left = child;
    } else {
      int child = increment(entries[static_cast<size_t>(copy)].right,
                            middle + 1, high, key);
      entries[static_cast<size_t>(copy)].right = child;
    }
    return copy;
  }

  int width = 1;
  int tile_count = 1;
  std::vector<Entry> entries;
};

} // namespace pnr
