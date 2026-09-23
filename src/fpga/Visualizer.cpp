#include "Visualizer.h"

#include "Device.h"
#include "Net.h"
#include "Tile.h"
#include "Wire.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_set>

namespace {

using fpga::CBNodeNameType;
using fpga::Coord;
using fpga::Visualizer;

constexpr int border_padding = 10;
constexpr int node_step = 8;
constexpr int node_size = 2;
constexpr int edge_title_reserve = 50;
constexpr int vertical_guide_offset = 60;
constexpr int local_first_line_offset = 130;
constexpr int local_first_line_distance = 90;
constexpr int local_second_line_distance = 110;
constexpr int joint_line_offset = 350;
constexpr int glyph_source_height = 5;
constexpr int glyph_width = 5;
constexpr int glyph_height = 7;
constexpr int glyph_advance = 7;
constexpr int node_font_width = glyph_width - 1;
constexpr int node_font_height = glyph_height - 1;
constexpr int node_font_advance = glyph_advance - 1;
constexpr int coordinate_glyph_width = glyph_width * 5;
constexpr int coordinate_glyph_height = glyph_height * 5;
constexpr int coordinate_glyph_advance = glyph_advance * 5;
constexpr int highlighted_glyph_width = 6;
constexpr int highlighted_glyph_height = 8;
constexpr int highlighted_glyph_advance = 8;
constexpr int highlighted_label_padding = 2;

constexpr Visualizer::Color background{247, 248, 250, 255};
constexpr Visualizer::Color border{70, 76, 86, 255};
constexpr Visualizer::Color guide{188, 193, 202, 255};
constexpr Visualizer::Color free_node{25, 105, 235, 255};
constexpr Visualizer::Color occupied_node{25, 185, 80, 255};
constexpr Visualizer::Color title_color{0, 0, 0, 255};
constexpr Visualizer::Color coordinate_color{105, 105, 105, 255};
constexpr Visualizer::Color highlighted_color{255, 0, 0, 255};

enum class Edge : uint8_t
{
    top,
    right,
    bottom,
    left,
};

bool sameCoord(const Coord& left, const Coord& right)
{
    return left.x == right.x && left.y == right.y;
}

int signedNibble(int value)
{
    value &= 0xf;
    return value >= 8 ? value - 16 : value;
}

Coord encodedDirection(int node)
{
    return {signedNibble(node >> 8), signedNibble(node >> 4)};
}

Edge directionEdge(Coord direction)
{
    if (direction.x == 0 && direction.y == 0) {
        return Edge::top;
    }
    int abs_x = std::abs(direction.x);
    int abs_y = std::abs(direction.y);
    if (direction.y < 0 && abs_y >= abs_x) {
        return Edge::top;
    }
    if (direction.x < 0 && abs_x >= abs_y) {
        return Edge::left;
    }
    if (direction.x > 0 && abs_x >= abs_y) {
        return Edge::right;
    }
    return Edge::bottom;
}

std::array<uint8_t, glyph_source_height> glyphRows(char value)
{
    char c = static_cast<char>(std::toupper(static_cast<unsigned char>(value)));
    switch (c) {
    case 'A': return {2, 5, 7, 5, 5};
    case 'B': return {6, 5, 6, 5, 6};
    case 'C': return {3, 4, 4, 4, 3};
    case 'D': return {6, 5, 5, 5, 6};
    case 'E': return {7, 4, 6, 4, 7};
    case 'F': return {7, 4, 6, 4, 4};
    case 'G': return {3, 4, 5, 5, 3};
    case 'H': return {5, 5, 7, 5, 5};
    case 'I': return {7, 2, 2, 2, 7};
    case 'J': return {1, 1, 1, 5, 2};
    case 'K': return {5, 5, 6, 5, 5};
    case 'L': return {4, 4, 4, 4, 7};
    case 'M': return {5, 7, 7, 5, 5};
    case 'N': return {5, 7, 7, 7, 5};
    case 'O': return {2, 5, 5, 5, 2};
    case 'P': return {6, 5, 6, 4, 4};
    case 'Q': return {2, 5, 5, 3, 1};
    case 'R': return {6, 5, 6, 5, 5};
    case 'S': return {3, 4, 2, 1, 6};
    case 'T': return {7, 2, 2, 2, 2};
    case 'U': return {5, 5, 5, 5, 7};
    case 'V': return {5, 5, 5, 5, 2};
    case 'W': return {5, 5, 7, 7, 5};
    case 'X': return {5, 5, 2, 5, 5};
    case 'Y': return {5, 5, 2, 2, 2};
    case 'Z': return {7, 1, 2, 4, 7};
    case '0': return {7, 5, 5, 5, 7};
    case '1': return {2, 6, 2, 2, 7};
    case '2': return {6, 1, 7, 4, 7};
    case '3': return {6, 1, 3, 1, 6};
    case '4': return {5, 5, 7, 1, 1};
    case '5': return {7, 4, 6, 1, 6};
    case '6': return {3, 4, 7, 5, 7};
    case '7': return {7, 1, 2, 2, 2};
    case '8': return {7, 5, 7, 5, 7};
    case '9': return {7, 5, 7, 1, 6};
    case '_': return {0, 0, 0, 0, 7};
    case '-': return {0, 0, 7, 0, 0};
    case '.': return {0, 0, 0, 0, 2};
    case ':': return {0, 2, 0, 2, 0};
    case '/': return {1, 1, 2, 4, 4};
    case '[': return {6, 4, 4, 4, 6};
    case ']': return {3, 1, 1, 1, 3};
    case '(': return {1, 2, 2, 2, 1};
    case ')': return {4, 2, 2, 2, 4};
    case '+': return {0, 2, 7, 2, 0};
    case '=': return {0, 7, 0, 7, 0};
    case '$': return {2, 7, 6, 3, 2};
    case '#': return {5, 7, 5, 7, 5};
    case ',': return {0, 0, 0, 2, 4};
    case ' ': return {0, 0, 0, 0, 0};
    default: return {7, 1, 2, 0, 2};
    }
}

std::array<uint8_t, glyph_height> nodeGlyphRows(char value)
{
    char c = static_cast<char>(std::toupper(static_cast<unsigned char>(value)));
    switch (c) {
    case 'A': return {14, 17, 17, 31, 17, 17, 17};
    case 'B': return {30, 17, 17, 30, 17, 17, 30};
    case 'C': return {15, 16, 16, 16, 16, 16, 15};
    case 'D': return {30, 17, 17, 17, 17, 17, 30};
    case 'E': return {31, 16, 16, 30, 16, 16, 31};
    case 'F': return {31, 16, 16, 30, 16, 16, 16};
    case 'G': return {15, 16, 16, 19, 17, 17, 15};
    case 'H': return {17, 17, 17, 31, 17, 17, 17};
    case 'I': return {31, 4, 4, 4, 4, 4, 31};
    case 'J': return {7, 2, 2, 2, 18, 18, 12};
    case 'K': return {17, 18, 20, 24, 20, 18, 17};
    case 'L': return {16, 16, 16, 16, 16, 16, 31};
    case 'M': return {17, 27, 21, 21, 17, 17, 17};
    case 'N': return {17, 25, 21, 19, 17, 17, 17};
    case 'O': return {14, 17, 17, 17, 17, 17, 14};
    case 'P': return {30, 17, 17, 30, 16, 16, 16};
    case 'Q': return {14, 17, 17, 17, 21, 18, 13};
    case 'R': return {30, 17, 17, 30, 20, 18, 17};
    case 'S': return {15, 16, 16, 14, 1, 1, 30};
    case 'T': return {31, 4, 4, 4, 4, 4, 4};
    case 'U': return {17, 17, 17, 17, 17, 17, 14};
    case 'V': return {17, 17, 17, 17, 17, 10, 4};
    case 'W': return {17, 17, 17, 17, 21, 21, 10};
    case 'X': return {17, 17, 10, 4, 10, 17, 17};
    case 'Y': return {17, 17, 10, 4, 4, 4, 4};
    case 'Z': return {31, 1, 2, 4, 8, 16, 31};
    case '0': return {14, 17, 19, 21, 25, 17, 14};
    case '1': return {4, 12, 4, 4, 4, 4, 14};
    case '2': return {14, 17, 1, 2, 4, 8, 31};
    case '3': return {30, 1, 1, 14, 1, 1, 30};
    case '4': return {2, 6, 10, 18, 31, 2, 2};
    case '5': return {31, 16, 16, 30, 1, 1, 30};
    case '6': return {14, 16, 16, 30, 17, 17, 14};
    case '7': return {31, 1, 2, 4, 8, 8, 8};
    case '8': return {14, 17, 17, 14, 17, 17, 14};
    case '9': return {14, 17, 17, 15, 1, 1, 14};
    case '_': return {0, 0, 0, 0, 0, 0, 31};
    case '-': return {0, 0, 0, 31, 0, 0, 0};
    case '.': return {0, 0, 0, 0, 0, 12, 12};
    case ':': return {0, 12, 12, 0, 12, 12, 0};
    case '/': return {1, 2, 2, 4, 8, 8, 16};
    case '[': return {14, 8, 8, 8, 8, 8, 14};
    case ']': return {14, 2, 2, 2, 2, 2, 14};
    case '(': return {2, 4, 8, 8, 8, 4, 2};
    case ')': return {8, 4, 2, 2, 2, 4, 8};
    case '+': return {0, 4, 4, 31, 4, 4, 0};
    case '=': return {0, 0, 31, 0, 31, 0, 0};
    case '$': return {4, 15, 20, 14, 5, 30, 4};
    case '#': return {10, 31, 10, 10, 31, 10, 0};
    case ',': return {0, 0, 0, 0, 0, 4, 8};
    case ' ': return {0, 0, 0, 0, 0, 0, 0};
    default: return {14, 17, 1, 2, 4, 0, 4};
    }
}

// Hand-fitted small glyphs. Downsampling the 5x7 alphabet to 4x6 skips its
// last column and row, erasing right-hand stems, baselines and underscores.
// Keep small captions binary and crisp instead of resampling those strokes.
std::array<uint8_t, node_font_height> compactNodeGlyphRows(char value)
{
    char c = static_cast<char>(std::toupper(static_cast<unsigned char>(value)));
    switch (c) {
    case 'A': return {6, 9, 9, 15, 9, 9};
    case 'B': return {14, 9, 14, 9, 9, 14};
    case 'C': return {7, 8, 8, 8, 8, 7};
    case 'D': return {14, 9, 9, 9, 9, 14};
    case 'E': return {15, 8, 14, 8, 8, 15};
    case 'F': return {15, 8, 14, 8, 8, 8};
    case 'G': return {7, 8, 8, 11, 9, 7};
    case 'H': return {9, 9, 15, 9, 9, 9};
    case 'I': return {14, 4, 4, 4, 4, 14};
    case 'J': return {7, 2, 2, 2, 10, 4};
    case 'K': return {9, 10, 12, 12, 10, 9};
    case 'L': return {8, 8, 8, 8, 8, 15};
    case 'M': return {9, 15, 15, 9, 9, 9};
    case 'N': return {9, 13, 13, 11, 11, 9};
    case 'O': return {6, 9, 9, 9, 9, 6};
    case 'P': return {14, 9, 9, 14, 8, 8};
    case 'Q': return {6, 9, 9, 9, 11, 7};
    case 'R': return {14, 9, 9, 14, 10, 9};
    case 'S': return {7, 8, 6, 1, 1, 14};
    case 'T': return {15, 4, 4, 4, 4, 4};
    case 'U': return {9, 9, 9, 9, 9, 6};
    case 'V': return {9, 9, 9, 9, 6, 6};
    case 'W': return {9, 9, 9, 15, 15, 9};
    case 'X': return {9, 9, 6, 6, 9, 9};
    case 'Y': return {9, 9, 6, 4, 4, 4};
    case 'Z': return {15, 1, 2, 4, 8, 15};
    case '0': return {6, 9, 9, 9, 9, 6};
    case '1': return {4, 12, 4, 4, 4, 14};
    case '2': return {6, 9, 1, 2, 4, 15};
    case '3': return {14, 1, 6, 1, 1, 14};
    case '4': return {2, 6, 10, 15, 2, 2};
    case '5': return {15, 8, 14, 1, 1, 14};
    case '6': return {7, 8, 14, 9, 9, 6};
    case '7': return {15, 1, 2, 4, 4, 4};
    case '8': return {6, 9, 6, 9, 9, 6};
    case '9': return {6, 9, 9, 7, 1, 14};
    case '_': return {0, 0, 0, 0, 0, 15};
    case '-': return {0, 0, 15, 0, 0, 0};
    case '.': return {0, 0, 0, 0, 0, 4};
    case ':': return {0, 4, 0, 0, 4, 0};
    case '/': return {1, 1, 2, 4, 8, 8};
    case '[': return {6, 4, 4, 4, 4, 6};
    case ']': return {6, 2, 2, 2, 2, 6};
    case '(': return {2, 4, 8, 8, 4, 2};
    case ')': return {4, 2, 1, 1, 2, 4};
    case '+': return {0, 4, 14, 4, 0, 0};
    case '=': return {0, 15, 0, 15, 0, 0};
    case '$': return {4, 7, 12, 6, 14, 4};
    case '#': return {10, 15, 10, 15, 10, 0};
    case ',': return {0, 0, 0, 0, 4, 8};
    case ' ': return {0, 0, 0, 0, 0, 0};
    default: return {6, 9, 1, 2, 0, 2};
    }
}

int textPixels(const std::string& text, int advance = node_font_advance,
               int width = node_font_width)
{
    return text.empty() ? 0
                        : (static_cast<int>(text.size()) - 1) * advance + width;
}

std::string nodeTitle(const std::string* name)
{
    return name ? *name : std::string{};
}

template<typename Table>
void addTableKeys(const Table& table, std::set<int>& nodes)
{
    for (const auto& [node, unused] : table.values) {
        (void)unused;
        nodes.insert(node);
    }
}

void addMask(const NodeMask& mask, std::set<int>& nodes)
{
    mask.for_each_set_bit([&](int node) {
        nodes.insert(node);
        return false;
    });
}

template<typename Table, typename GetMask>
void addTableValues(const Table& table, std::set<int>& nodes, GetMask get_mask)
{
    for (const auto& [unused, state] : table.values) {
        (void)unused;
        addMask(get_mask(state), nodes);
    }
}

bool occupied(const fpga::Tile& tile, CBNodeNameType type, int node)
{
    switch (type) {
    case fpga::CB_NODE_LOCAL:
        return tile.cb.local.local.testBit(node)
            || tile.pin_state.leased_nodes.testBit(node);
    case fpga::CB_NODE_JOINT:
        return tile.cb.joint.jump.testBit(node);
    case fpga::CB_NODE_SRC:
        return tile.cb.src.jump.testBit(node);
    case fpga::CB_NODE_DST:
        return tile.cb.dst.jump.testBit(node);
    case fpga::CB_NODE_JUMP:
        return false;
    }
    return false;
}

Visualizer::Color routeColor(const rtl::Net& net, uint64_t route_id)
{
    size_t hash = std::hash<std::string>{}(net.name);
    hash ^= static_cast<size_t>(route_id) + 0x9e3779b97f4a7c15ULL
        + (hash << 6) + (hash >> 2);
    static constexpr std::array<Visualizer::Color, 8> palette{{
        {220, 45, 45, 255}, {230, 125, 20, 255}, {145, 55, 215, 255},
        {15, 145, 160, 255}, {210, 45, 145, 255}, {110, 95, 220, 255},
        {190, 75, 25, 255}, {25, 145, 105, 255},
    }};
    return palette[hash % palette.size()];
}

} // namespace

size_t fpga::Visualizer::NodeKeyHash::operator()(const NodeKey& key) const
{
    size_t hash = static_cast<size_t>(static_cast<uint32_t>(key.x));
    hash ^= static_cast<size_t>(static_cast<uint32_t>(key.y)) << 17;
    hash ^= static_cast<size_t>(key.type) << 34;
    hash ^= static_cast<size_t>(key.node) << 40;
    return hash;
}

fpga::Visualizer::Visualizer(Device& device) : device(device)
{
}

fpga::Visualizer::Visualizer() : Visualizer(Device::current())
{
}

bool fpga::Visualizer::inRegion(Coord coord) const
{
    return coord.x >= center.x - 2 && coord.x <= center.x + 2
        && coord.y >= center.y - 2 && coord.y <= center.y + 2;
}

fpga::Visualizer::Point fpga::Visualizer::tileOrigin(Coord coord) const
{
    return {(coord.x - (center.x - 2)) * tile_pixels,
            (coord.y - (center.y - 2)) * tile_pixels};
}

// Use the same ownership resolution as routing, without changing any tile state.
fpga::Coord fpga::Visualizer::routingCoord(Coord coord) const
{
    if (Tile* tile = device.getTile(coord.x, coord.y)) {
        if (Tile* owner = device.routeTile(*tile)) return owner->coord;
    }
    return coord;
}

void fpga::Visualizer::buildJumpDirections()
{
    // Resolve only crossbars drawn in this 5x5 view. Scanning every source in
    // a large device makes terminal-failure visualization dominate runtime.
    for (int y = center.y - 2; y <= center.y + 2; ++y) {
        for (int x = center.x - 2; x <= center.x + 2; ++x) {
            Tile* source = device.getTile(x, y);
            if (!source || !source->cb_type
                || !sameCoord(routingCoord(source->coord), source->coord)) {
                continue;
            }
            for (const auto& [src, unused] : source->cb_type->dst_by_src.values) {
                (void)unused;
                std::vector<TileJumpTarget> targets =
                    device.resolveJumpTargets(*source, src);
                for (const TileJumpTarget& target : targets) {
                    if (!target.tile || target.dst_node < 0) {
                        continue;
                    }
                    Coord delta{target.tile->coord.x - source->coord.x,
                                target.tile->coord.y - source->coord.y};
                    if (inRegion(source->coord)) {
                        node_directions[{source->coord.x, source->coord.y,
                                         CB_NODE_SRC, src}] = delta;
                    }
                    if (inRegion(target.tile->coord)) {
                        NodeKey key{target.tile->coord.x, target.tile->coord.y,
                                    CB_NODE_DST,
                                    static_cast<uint16_t>(target.dst_node)};
                        Coord incoming{-delta.x, -delta.y};
                        auto found = node_directions.find(key);
                        if (found == node_directions.end()
                            || std::abs(incoming.x) + std::abs(incoming.y)
                                < std::abs(found->second.x)
                                    + std::abs(found->second.y)) {
                            node_directions[key] = incoming;
                        }
                    }
                }
            }
        }
    }
}

void fpga::Visualizer::drawCB(int x, int y)
{
    center = {x, y};
    crossbars_drawn = true;
    node_positions.clear();
    node_directions.clear();
    drawn_nodes.clear();
    node_titles.clear();
    highlighted_label_drawn = false;
    render_stats = {};
    image.init(image_pixels, image_pixels);
    for (int py = 0; py < image.height; ++py) {
        for (int px = 0; px < image.width; ++px) {
            image.set_pixel(px, py, background.r, background.g, background.b,
                            background.a);
        }
    }

    buildJumpDirections();
    for (int tile_y = y - 2; tile_y <= y + 2; ++tile_y) {
        for (int tile_x = x - 2; tile_x <= x + 2; ++tile_x) {
            if (Tile* tile = device.getTile(tile_x, tile_y)) {
                drawTile(*tile);
            }
        }
    }
    drawNodeDots();
    drawNodeTitles();
}

void fpga::Visualizer::drawTile(Tile& tile)
{
    if (!inRegion(tile.coord)) {
        return;
    }
    ++render_stats.tiles;
    Point origin = tileOrigin(tile.coord);
    int left = origin.x + border_padding;
    int top = origin.y + border_padding;
    int right = origin.x + tile_pixels - border_padding - 1;
    int bottom = origin.y + tile_pixels - border_padding - 1;
    std::string coordinate_text = std::to_string(tile.coord.x) + ","
        + std::to_string(tile.coord.y);
    int coordinate_width = textPixels(coordinate_text,
                                      coordinate_glyph_advance,
                                      coordinate_glyph_width);
    drawCoordinateText(
        coordinate_text,
        {left + (right - left + 1 - coordinate_width) / 2,
         top + (bottom - top + 1 - coordinate_glyph_height) / 2});

    image.draw_line(left, top, right, top, border.r, border.g, border.b, border.a);
    image.draw_line(right, top, right, bottom, border.r, border.g, border.b, border.a);
    image.draw_line(right, bottom, left, bottom, border.r, border.g, border.b, border.a);
    image.draw_line(left, bottom, left, top, border.r, border.g, border.b, border.a);

    // Resource views identify their owner instead of duplicating its nodes and empty lease state.
    Coord owner = routingCoord(tile.coord);
    if (!tile.cb_type || !sameCoord(owner, tile.coord)) {
        std::string label;
        if (!tile.cb_type) {
            ++render_stats.tiles_without_crossbar;
            label = "No crossbar assigned";
        } else {
            ++render_stats.attached_tiles;
            label = "Crossbar at " + std::to_string(owner.x) + ","
                + std::to_string(owner.y);
            if (!inRegion(owner)) label += " (outside view)";
        }
        drawText(label, {left + 12, top + 16}, TextDirection::right,
                 title_color, node_font_width, node_font_height, node_font_advance);
        return;
    }
    ++render_stats.crossbars;

    int local_x = origin.x + local_first_line_offset;
    int local_x2 = local_x + local_first_line_distance;
    int local_x3 = local_x2 + local_second_line_distance;
    int joint_x = origin.x + joint_line_offset;
    int guide_top = top + vertical_guide_offset;
    image.draw_line(local_x, guide_top, local_x, bottom,
                    guide.r, guide.g, guide.b, guide.a);
    image.draw_line(local_x2, guide_top, local_x2, bottom,
                    guide.r, guide.g, guide.b, guide.a);
    image.draw_line(local_x3, guide_top, local_x3, bottom,
                    guide.r, guide.g, guide.b, guide.a);
    image.draw_line(joint_x, guide_top, joint_x, bottom,
                    guide.r, guide.g, guide.b, guide.a);

    std::set<int> locals;
    std::set<int> joints;
    std::set<int> srcs;
    std::set<int> dsts;
    CBType& cb = *tile.cb_type;

    auto add_route_node = [&](Coord coord, CBNodeNameType type, int node) {
        if (node < 0 || !sameCoord(coord, tile.coord)) {
            return;
        }
        if (type == CB_NODE_LOCAL) locals.insert(node);
        if (type == CB_NODE_JOINT) joints.insert(node);
        if (type == CB_NODE_SRC) srcs.insert(node);
        if (type == CB_NODE_DST) dsts.insert(node);
    };
    auto add_route_nodes = [&](const std::vector<Wire>& route) {
        for (const Wire& fragment : route) {
            if (fragment.type == Wire::WIRE_TILE_PIN) {
                add_route_node(fragment.from, CB_NODE_LOCAL, fragment.local);
                continue;
            }
            if (fragment.type == Wire::WIRE_ROUTE_EDGE) {
                add_route_node(fragment.from,
                               static_cast<CBNodeNameType>(fragment.from_node_type),
                               fragment.from_node);
                add_route_node(fragment.to,
                               static_cast<CBNodeNameType>(fragment.to_node_type),
                               fragment.to_node);
                continue;
            }
            if (fragment.type != Wire::WIRE_CROSSBAR) {
                continue;
            }
            add_route_node(fragment.from,
                           fragment.pos == 0 ? CB_NODE_LOCAL : CB_NODE_DST,
                           fragment.local);
            add_route_node(fragment.from, CB_NODE_JOINT, fragment.joint2);
            add_route_node(fragment.from, CB_NODE_JOINT, fragment.joint);
            add_route_node(fragment.from, CB_NODE_SRC, fragment.jump);
            add_route_node(fragment.to, CB_NODE_DST, fragment.dst);
        }
    };
    std::unordered_set<const std::vector<Wire>*> indexed_routes;
    auto add_binding_nodes = [&](rtl::Net* net, size_t route_index) {
        if (!net || route_index >= net->routes.size()) {
            return;
        }
        const rtl::NetRouteBinding& binding = net->routes[route_index];
        if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
            return;
        }
        const std::vector<Wire>* route = &binding.owner->wires[binding.route_index];
        if (indexed_routes.insert(route).second) {
            add_route_nodes(*route);
        }
    };
    for (const Tile::RoutedBinding& binding : tile.routed_bindings) {
        if (!binding.net || binding.route_id == 0) {
            continue;
        }
        add_binding_nodes(binding.net,
                          binding.net->findRouteBindingById(binding.route_id));
    }
    for (const Ref<rtl::Net>& net_ref : tile.routedNets) {
        if (!net_ref.peer) {
            continue;
        }
        for (size_t route_index = 0; route_index < net_ref.peer->routes.size();
             ++route_index) {
            add_binding_nodes(net_ref.peer, route_index);
        }
    }

    // Names annotate numeric resources but do not establish physical presence:
    // specialized CBTypes retain names removed from their live topology.
    addTableKeys(cb.local_src, locals);
    addTableValues(cb.local_src, srcs,
                   [](const CBJumpState& state) -> const NodeMask& { return state.jump; });
    addTableKeys(cb.local_joint, locals);
    addTableValues(cb.local_joint, joints,
                   [](const CBJointState& state) -> const NodeMask& { return state.joint; });
    addTableKeys(cb.local_local, locals);
    addTableValues(cb.local_local, locals,
                   [](const CBLocalState& state) -> const NodeMask& { return state.local; });
    addTableKeys(cb.src_joint, srcs);
    addTableValues(cb.src_joint, joints,
                   [](const CBJointState& state) -> const NodeMask& { return state.joint; });
    addTableKeys(cb.joint_src, joints);
    addTableValues(cb.joint_src, srcs,
                   [](const CBJumpState& state) -> const NodeMask& { return state.jump; });
    addTableKeys(cb.joint_local, joints);
    addTableValues(cb.joint_local, locals,
                   [](const CBLocalState& state) -> const NodeMask& { return state.local; });
    addTableKeys(cb.joint_joint, joints);
    addTableValues(cb.joint_joint, joints,
                   [](const CBJointState& state) -> const NodeMask& { return state.joint; });
    addTableKeys(cb.dst_src, dsts);
    addTableValues(cb.dst_src, srcs,
                   [](const CBJumpState& state) -> const NodeMask& { return state.jump; });
    addTableKeys(cb.dst_local, dsts);
    addTableValues(cb.dst_local, locals,
                   [](const CBLocalState& state) -> const NodeMask& { return state.local; });
    addTableKeys(cb.dst_joint, dsts);
    addTableValues(cb.dst_joint, joints,
                   [](const CBJointState& state) -> const NodeMask& { return state.joint; });
    addTableKeys(cb.joint_reachable_srcs, joints);
    addTableValues(cb.joint_reachable_srcs, srcs,
                   [](const CBJumpState& state) -> const NodeMask& { return state.jump; });
    addTableKeys(cb.src_reachable_joints, srcs);
    addTableValues(cb.src_reachable_joints, joints,
                   [](const CBJointState& state) -> const NodeMask& { return state.joint; });
    addTableKeys(cb.local_reachable_joints, locals);
    addTableValues(cb.local_reachable_joints, joints,
                   [](const CBJointState& state) -> const NodeMask& { return state.joint; });
    addTableValues(cb.dsts_reaching_src, dsts,
                   [](const CBJumpState& state) -> const NodeMask& { return state.jump; });
    addTableValues(cb.dsts_reaching_local, dsts,
                   [](const CBJumpState& state) -> const NodeMask& { return state.jump; });
    for (const auto& [src, targets] : cb.dst_by_src.values) {
        if (!targets.empty()) {
            srcs.insert(src);
        }
    }
    addMask(cb.local_input_nodes, locals);
    addMask(cb.local_output_nodes, locals);
    addMask(cb.constant_zero_nodes, locals);
    addMask(cb.constant_one_nodes, locals);
    addMask(cb.valid_dst_nodes, dsts);
    addMask(tile.incoming_dst_nodes, dsts);
    addMask(tile.cb.local.local, locals);
    addMask(tile.pin_state.leased_nodes, locals);
    addMask(tile.cb.joint.jump, joints);
    addMask(tile.cb.src.jump, srcs);
    addMask(tile.cb.dst.jump, dsts);

    render_stats.local_nodes += locals.size();
    render_stats.joint_nodes += joints.size();
    render_stats.src_nodes += srcs.size();
    render_stats.dst_nodes += dsts.size();

    auto place_vertical = [&](const auto& nodes, CBNodeNameType type, int px) {
        int capacity = (bottom - guide_top - node_step) / node_step + 1;
        int index = 0;
        for (int node : nodes) {
            int lane = index / capacity;
            int row = index % capacity;
            int lane_offset = lane == 0
                ? 0
                : ((lane + 1) / 2) * node_step * (lane & 1 ? 1 : -1);
            Point point{px + lane_offset,
                        guide_top + node_step + row * node_step};
            NodeKey key{tile.coord.x, tile.coord.y, type,
                        static_cast<uint16_t>(node)};
            node_positions[key] = point;
            drawn_nodes.push_back({key, point, occupied(tile, type, node)});
            const std::string* name = cb.nodeName(type, node);
            if (type == CB_NODE_LOCAL) {
                std::string title = nodeTitle(name);
                if (!title.empty()) {
                    node_titles.push_back(
                        {title, {point.x - 4 - textPixels(title), point.y - 3},
                         TextDirection::right});
                }
            } else if (type == CB_NODE_JOINT) {
                std::string title = nodeTitle(name);
                if (!title.empty()) {
                    node_titles.push_back(
                        {title, {point.x + 4, point.y - 3},
                         TextDirection::right});
                }
            }
            ++index;
        }
    };
    std::vector<int> ordered_locals;
    ordered_locals.reserve(locals.size());
    for (int node : locals) {
        if (!cb.constant_zero_nodes.testBit(node)
            && !cb.constant_one_nodes.testBit(node)) {
            ordered_locals.push_back(node);
        }
    }
    cb.constant_zero_nodes.for_each_set_bit([&](int node) {
        if (locals.contains(node)) ordered_locals.push_back(node);
        return false;
    });
    cb.constant_one_nodes.for_each_set_bit([&](int node) {
        if (locals.contains(node) && !cb.constant_zero_nodes.testBit(node)) {
            ordered_locals.push_back(node);
        }
        return false;
    });
    size_t first_local_count = (ordered_locals.size() + 2) / 3;
    size_t second_local_end = (2 * ordered_locals.size() + 2) / 3;
    std::vector<int> first_locals(ordered_locals.begin(),
                                  ordered_locals.begin() + first_local_count);
    std::vector<int> second_locals(ordered_locals.begin() + first_local_count,
                                   ordered_locals.begin() + second_local_end);
    std::vector<int> third_locals(ordered_locals.begin() + second_local_end,
                                  ordered_locals.end());
    place_vertical(first_locals, CB_NODE_LOCAL, local_x);
    place_vertical(second_locals, CB_NODE_LOCAL, local_x2);
    place_vertical(third_locals, CB_NODE_LOCAL, local_x3);
    place_vertical(joints, CB_NODE_JOINT, joint_x);

    std::array<std::vector<int>, 4> dst_edges;
    std::array<std::vector<int>, 4> src_edges;
    auto node_direction = [&](CBNodeNameType type, int node) {
        NodeKey key{tile.coord.x, tile.coord.y, type,
                    static_cast<uint16_t>(node)};
        auto found = node_directions.find(key);
        if (found != node_directions.end()) {
            return found->second;
        }
        if (type == CB_NODE_SRC) {
            auto priorities = cb.src_priority_deltas.find(node);
            if (priorities != cb.src_priority_deltas.end()
                && !priorities->second.empty()) {
                return priorities->second.front();
            }
        }
        Coord fallback = encodedDirection(node);
        // Encoded directions describe wire travel. If an incoming source is
        // outside the viewport, its DST still belongs on the arrival edge,
        // opposite to that travel direction (as in buildJumpDirections()).
        if (type == CB_NODE_DST) {
            fallback = {-fallback.x, -fallback.y};
        }
        if (fallback.x == 0 && fallback.y == 0) {
            switch (node & 3) {
            case 0: fallback = {0, -1}; break;
            case 1: fallback = {1, 0}; break;
            case 2: fallback = {0, 1}; break;
            default: fallback = {-1, 0}; break;
            }
        }
        return fallback;
    };
    for (int node : dsts) {
        dst_edges[static_cast<size_t>(directionEdge(
            node_direction(CB_NODE_DST, node)))].push_back(node);
    }
    for (int node : srcs) {
        src_edges[static_cast<size_t>(directionEdge(
            node_direction(CB_NODE_SRC, node)))].push_back(node);
    }

    for (size_t edge_index = 0; edge_index < dst_edges.size(); ++edge_index) {
        // Only the top edge starts next to its corners. Preserve the previous
        // caption reserve on the other edges. Never wrap into extra lanes.
        int corner_padding = edge_index == static_cast<size_t>(Edge::top)
            ? node_size + 1 : edge_title_reserve + node_step;
        int count = dst_edges[edge_index].size() + src_edges[edge_index].size();
        int intervals = std::max(1, count - 1);
        int span = right - left - 2 * corner_padding;
        auto edge_point = [&](Edge edge, int index, bool source) {
            int offset = corner_padding
                + std::min(index * node_step, index * span / intervals);
            switch (edge) {
            case Edge::top:
                return Point{source ? right - offset : left + offset,
                             top};
            case Edge::right:
                return Point{right,
                             source ? bottom - offset : top + offset};
            case Edge::bottom:
                return Point{source ? left + offset : right - offset,
                             bottom};
            case Edge::left:
                return Point{left,
                             source ? top + offset : bottom - offset};
            }
            return Point{};
        };
        auto add_edge_title = [&](Edge title_edge, const Point& point,
                                  CBNodeNameType type, int node) {
            std::string title = nodeTitle(cb.nodeName(type, node));
            if (title.empty()) {
                return;
            }
            switch (title_edge) {
            case Edge::top:
                node_titles.push_back(
                    {title, {point.x - 2, top + 3}, TextDirection::down});
                break;
            case Edge::right:
                node_titles.push_back(
                    {title, {right - 3 - textPixels(title), point.y - 2},
                     TextDirection::right});
                break;
            case Edge::bottom:
                node_titles.push_back(
                    {title, {point.x + 2, bottom - 3}, TextDirection::up});
                break;
            case Edge::left:
                node_titles.push_back(
                    {title, {left + 3, point.y - 2}, TextDirection::right});
                break;
            }
        };
        Edge edge = static_cast<Edge>(edge_index);
        int index = 0;
        for (int node : dst_edges[edge_index]) {
            Point point = edge_point(edge, index++, false);
            NodeKey key{tile.coord.x, tile.coord.y, CB_NODE_DST,
                        static_cast<uint16_t>(node)};
            node_positions[key] = point;
            drawn_nodes.push_back({key, point,
                                   occupied(tile, CB_NODE_DST, node)});
            add_edge_title(edge, point, CB_NODE_DST, node);
        }
        index = 0;
        for (int node : src_edges[edge_index]) {
            Point point = edge_point(edge, index++, true);
            NodeKey key{tile.coord.x, tile.coord.y, CB_NODE_SRC,
                        static_cast<uint16_t>(node)};
            node_positions[key] = point;
            drawn_nodes.push_back({key, point,
                                   occupied(tile, CB_NODE_SRC, node)});
            add_edge_title(edge, point, CB_NODE_SRC, node);
        }
    }
}

void fpga::Visualizer::drawNodeDots()
{
    render_stats.nodes = drawn_nodes.size();
    render_stats.occupied_nodes = static_cast<size_t>(std::count_if(
        drawn_nodes.begin(), drawn_nodes.end(),
        [](const NodeDraw& node) { return node.occupied; }));
    for (const NodeDraw& node : drawn_nodes) {
        Color color = node.occupied ? occupied_node : free_node;
        for (int dy = 0; dy < node_size; ++dy) {
            for (int dx = 0; dx < node_size; ++dx) {
                image.set_pixel(node.point.x + dx, node.point.y + dy,
                                color.r, color.g, color.b, color.a);
            }
        }
    }
}

void fpga::Visualizer::drawText(const std::string& text, Point point,
                                TextDirection direction, const Color& color,
                                int font_width, int font_height, int advance,
                                int bold_weight)
{
    if (text.empty() || font_width <= 0 || font_height <= 0) {
        return;
    }
    for (size_t char_index = 0; char_index < text.size(); ++char_index) {
        bool compact = font_width == node_font_width && font_height == node_font_height;
        const auto rows = nodeGlyphRows(text[char_index]);
        const auto compact_rows = compactNodeGlyphRows(text[char_index]);
        std::vector<uint8_t> pixels(
            static_cast<size_t>(font_width * font_height), 0);
        auto set_local = [&](int x, int y) {
            if (x >= 0 && x < font_width && y >= 0 && y < font_height) {
                pixels[static_cast<size_t>(y * font_width + x)] = 1;
            }
        };
        for (int y = 0; y < font_height; ++y) {
            int source_y = compact ? y : y * glyph_height / font_height;
            for (int x = 0; x < font_width; ++x) {
                int source_x = compact ? x : x * glyph_width / font_width;
                uint8_t row = compact ? compact_rows[source_y] : rows[source_y];
                int width = compact ? node_font_width : glyph_width;
                if ((row & (1u << (width - 1 - source_x))) != 0) {
                    set_local(x, y);
                }
            }
        }
        if (bold_weight > 0) {
            std::vector<uint8_t> ordinary = pixels;
            for (int y = 0; y < font_height; ++y) {
                for (int x = 0; x < font_width; ++x) {
                    if (ordinary[static_cast<size_t>(y * font_width + x)]) {
                        for (int dy = 0; dy <= bold_weight; ++dy) {
                            for (int dx = 0; dx <= bold_weight; ++dx) {
                                set_local(x + dx, y + dy);
                            }
                        }
                    }
                }
            }
        }
        int char_advance = static_cast<int>(char_index) * advance;
        for (int row = 0; row < font_height; ++row) {
            for (int column = 0; column < font_width; ++column) {
                if (!pixels[static_cast<size_t>(row * font_width + column)]) {
                    continue;
                }
                int x = point.x;
                int y = point.y;
                if (direction == TextDirection::right) {
                    x += char_advance + column;
                    y += row;
                } else if (direction == TextDirection::down) {
                    x += font_height - 1 - row;
                    y += char_advance + column;
                } else {
                    x += row;
                    y -= char_advance + column;
                }
                image.set_pixel(x, y, color.r, color.g, color.b, color.a);
            }
        }
    }
}

void fpga::Visualizer::drawNodeTitles()
{
    std::set<std::pair<int, int>> occupied_anchors;
    render_stats.node_titles = 0;
    render_stats.title_anchor_collisions = 0;
    for (const TextDraw& title : node_titles) {
        auto anchor = std::make_pair(title.point.x, title.point.y);
        if (!occupied_anchors.insert(anchor).second) {
            ++render_stats.title_anchor_collisions;
            continue;
        }
        drawText(title.text, title.point, title.direction, title_color,
                 node_font_width, node_font_height, node_font_advance);
        ++render_stats.node_titles;
    }
}

void fpga::Visualizer::drawCoordinateText(const std::string& text, Point point)
{
    for (size_t char_index = 0; char_index < text.size(); ++char_index) {
        const auto rows = glyphRows(text[char_index]);
        int char_x = point.x
            + static_cast<int>(char_index) * coordinate_glyph_advance;
        for (int y = 0; y < coordinate_glyph_height; ++y) {
            int source_y = y * glyph_source_height / coordinate_glyph_height;
            for (int x = 0; x < coordinate_glyph_width; ++x) {
                int source_x = x * 3 / coordinate_glyph_width;
                if ((rows[source_y] & (1u << (2 - source_x))) == 0) continue;
                image.set_pixel(char_x + x, point.y + y,
                                coordinate_color.r, coordinate_color.g,
                                coordinate_color.b, coordinate_color.a);
            }
        }
    }
}

void fpga::Visualizer::drawFailureLabel(const std::string& name)
{
    if (image.width <= 0 || image.height <= 0) {
        throw std::logic_error("Visualizer::drawCB must precede drawFailureLabel");
    }
    drawHighlightedName(name, {image.width / 2, image.height / 2});
}

void fpga::Visualizer::drawHighlightedName(const std::string& name, Point start)
{
    if (name.empty() || highlighted_label_drawn) return;
    int maximum_width = image.width - 8;
    size_t maximum_chars = static_cast<size_t>(std::max(
        1, (maximum_width - highlighted_glyph_width)
                / highlighted_glyph_advance
            + 1));
    std::string label = name.size() <= maximum_chars
        ? name : name.substr(0, maximum_chars - 3) + "...";
    int width = textPixels(label, highlighted_glyph_advance,
                           highlighted_glyph_width);
    Point label_point{
        std::clamp(start.x + 8, 4, image.width - width - 4),
        std::clamp(start.y - highlighted_glyph_height - 8,
                   4, image.height - highlighted_glyph_height - 4)};
    for (int y = label_point.y - highlighted_label_padding;
         y < label_point.y + highlighted_glyph_height
             + highlighted_label_padding;
         ++y) {
        for (int x = label_point.x - highlighted_label_padding;
             x < label_point.x + width + highlighted_label_padding; ++x) {
            image.set_pixel(x, y, background.r, background.g, background.b,
                            background.a);
        }
    }
    drawText(label, label_point, TextDirection::right, highlighted_color,
             highlighted_glyph_width, highlighted_glyph_height,
             highlighted_glyph_advance, true);
    highlighted_label_drawn = true;
    ++render_stats.highlighted_labels;
}

std::optional<fpga::Visualizer::Point>
fpga::Visualizer::nodePosition(Coord tile, CBNodeNameType type, int node) const
{
    if (node < 0 || node >= CB_MAX_NODES) {
        return std::nullopt;
    }
    // Aliased endpoints must use the owner's dot, color, and route position.
    tile = routingCoord(tile);
    auto found = node_positions.find(
        {tile.x, tile.y, static_cast<uint16_t>(type),
         static_cast<uint16_t>(node)});
    if (found == node_positions.end()) {
        return std::nullopt;
    }
    return found->second;
}

std::optional<fpga::Visualizer::Point>
fpga::Visualizer::drawPhysicalRoute(const std::vector<Wire>& route,
                                    const Color& color, int line_width)
{
    struct RoutePoint
    {
        Coord tile;
        CBNodeNameType type = CB_NODE_LOCAL;
        int node = -1;
    };
    std::optional<Point> first_visible;
    auto draw_pair = [&](const RoutePoint& from, const RoutePoint& to) {
        std::optional<Point> a = nodePosition(from.tile, from.type, from.node);
        std::optional<Point> b = nodePosition(to.tile, to.type, to.node);
        if (!a || !b) {
            return;
        }
        if (!first_visible) first_visible = *a;
        int normal_x = std::abs(b->x - a->x) < std::abs(b->y - a->y);
        int normal_y = normal_x == 0;
        int first_offset = -(line_width - 1) / 2;
        for (int stripe = 0; stripe < line_width; ++stripe) {
            int offset = first_offset + stripe;
            image.draw_line(a->x + normal_x * offset,
                            a->y + normal_y * offset,
                            b->x + normal_x * offset,
                            b->y + normal_y * offset,
                            color.r, color.g, color.b, color.a);
        }
        ++render_stats.route_segments;
    };

    std::optional<RoutePoint> previous;
    for (const Wire& fragment : route) {
        std::vector<RoutePoint> points;
        if (fragment.type == Wire::WIRE_TILE_PIN && fragment.local >= 0) {
            points.push_back({fragment.from, CB_NODE_LOCAL, fragment.local});
        } else if (fragment.type == Wire::WIRE_ROUTE_EDGE
                   && fragment.from_node >= 0 && fragment.to_node >= 0) {
            points.push_back({fragment.from,
                              static_cast<CBNodeNameType>(fragment.from_node_type),
                              fragment.from_node});
            points.push_back({fragment.to,
                              static_cast<CBNodeNameType>(fragment.to_node_type),
                              fragment.to_node});
        } else if (fragment.type == Wire::WIRE_CROSSBAR) {
            if (fragment.local >= 0) {
                points.push_back({fragment.from,
                                  fragment.pos == 0 ? CB_NODE_LOCAL : CB_NODE_DST,
                                  fragment.local});
            }
            if (fragment.joint2 >= 0) {
                points.push_back({fragment.from, CB_NODE_JOINT, fragment.joint2});
            }
            if (fragment.joint >= 0) {
                points.push_back({fragment.from, CB_NODE_JOINT, fragment.joint});
            }
            if (fragment.jump >= 0) {
                points.push_back({fragment.from, CB_NODE_SRC, fragment.jump});
                if (fragment.dst >= 0) {
                    points.push_back({fragment.to, CB_NODE_DST, fragment.dst});
                }
            }
        }
        if (points.empty()) {
            continue;
        }
        if (previous && sameCoord(routingCoord(previous->tile), routingCoord(points.front().tile))
            && (previous->type != points.front().type
                || previous->node != points.front().node)) {
            draw_pair(*previous, points.front());
        }
        for (size_t index = 1; index < points.size(); ++index) {
            draw_pair(points[index - 1], points[index]);
        }
        previous = points.back();
    }
    return first_visible;
}

void fpga::Visualizer::drawRoutes(const std::string& highlighted_net)
{
    if (!crossbars_drawn) {
        throw std::logic_error("Visualizer::drawCB must precede drawRoutes");
    }
    struct BindingKey
    {
        rtl::Net* net = nullptr;
        size_t route_index = std::numeric_limits<size_t>::max();
        bool operator==(const BindingKey&) const = default;
    };
    struct BindingHash
    {
        size_t operator()(const BindingKey& key) const
        {
            return std::hash<void*>{}(key.net)
                ^ (key.route_index * 0x9e3779b97f4a7c15ULL);
        }
    };
    std::unordered_set<BindingKey, BindingHash> bindings;
    std::unordered_set<rtl::Net*> fallback_nets;
    for (int y = center.y - 2; y <= center.y + 2; ++y) {
        for (int x = center.x - 2; x <= center.x + 2; ++x) {
            Tile* tile = device.getTile(x, y);
            if (!tile) {
                continue;
            }
            for (const Tile::RoutedBinding& binding : tile->routed_bindings) {
                if (binding.net && binding.route_id != 0) {
                    size_t route_index =
                        binding.net->findRouteBindingById(binding.route_id);
                    if (route_index < binding.net->routes.size()) {
                        bindings.insert({binding.net, route_index});
                    }
                }
            }
            for (Ref<rtl::Net>& net : tile->routedNets) {
                if (net.peer) {
                    fallback_nets.insert(net.peer);
                }
            }
        }
    }
    for (rtl::Net* net : fallback_nets) {
        for (size_t index = 0; index < net->routes.size(); ++index) {
            bindings.insert({net, index});
        }
    }

    struct DrawableRoute
    {
        const std::vector<Wire>* route = nullptr;
        rtl::Net* net = nullptr;
        uint64_t color_id = 0;
        bool highlighted = false;
    };
    std::vector<DrawableRoute> routes;
    routes.reserve(bindings.size());
    for (const BindingKey& key : bindings) {
        if (key.route_index >= key.net->routes.size()) {
            continue;
        }
        rtl::NetRouteBinding& binding = key.net->routes[key.route_index];
        if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
            continue;
        }
        const std::vector<Wire>& route = binding.owner->wires[binding.route_index];
        bool touches = std::any_of(route.begin(), route.end(), [&](const Wire& wire) {
            return inRegion(routingCoord(wire.from)) || inRegion(routingCoord(wire.to));
        });
        if (touches) {
            uint64_t color_id = binding.route_id != 0
                ? binding.route_id : static_cast<uint64_t>(key.route_index + 1);
            bool highlighted = !highlighted_net.empty()
                && (key.net->name == highlighted_net
                    || binding.route_name == highlighted_net);
            routes.push_back({&route, key.net, color_id, highlighted});
        }
    }
    std::optional<Point> highlighted_start;
    for (bool highlighted_pass : {false, true}) {
        for (const DrawableRoute& route : routes) {
            if (route.highlighted != highlighted_pass) {
                continue;
            }
            std::optional<Point> route_start = drawPhysicalRoute(
                *route.route,
                route.highlighted
                    ? highlighted_color
                    : routeColor(*route.net, route.color_id),
                route.highlighted ? 3 : 1);
            if (route.highlighted && route_start && !highlighted_start) {
                highlighted_start = route_start;
            }
            ++render_stats.routes;
        }
    }
    // Route lines are drawn first visually; repaint node state over them so
    // green/blue occupancy remains directly comparable with the route graph.
    drawNodeDots();
    drawNodeTitles();
    if (highlighted_start) {
        drawHighlightedName(highlighted_net, *highlighted_start);
    }
}

void fpga::Visualizer::drawRoute(const std::vector<Wire>& route,
                                 bool highlighted,
                                 const std::string& highlighted_name)
{
    if (!crossbars_drawn) {
        throw std::logic_error("Visualizer::drawCB must precede drawRoute");
    }
    size_t segments_before = render_stats.route_segments;
    std::optional<Point> route_start = drawPhysicalRoute(
        route, highlighted ? highlighted_color : Color{145, 55, 215, 255},
        highlighted ? 3 : 1);
    if (render_stats.route_segments != segments_before) {
        ++render_stats.routes;
    }
    drawNodeDots();
    drawNodeTitles();
    if (highlighted && route_start) {
        drawHighlightedName(highlighted_name, *route_start);
    }
}

bool fpga::Visualizer::highlightNode(Coord tile, CBNodeNameType type, int node,
                                     const std::string& highlighted_name)
{
    if (!crossbars_drawn) {
        throw std::logic_error("Visualizer::drawCB must precede highlightNode");
    }
    std::optional<Point> point = nodePosition(tile, type, node);
    if (!point) {
        return false;
    }

    constexpr int marker_radius = 7;
    constexpr int marker_width = 3;
    for (int stripe = 0; stripe < marker_width; ++stripe) {
        int offset = stripe - marker_width / 2;
        image.draw_line(point->x - marker_radius,
                        point->y - marker_radius + offset,
                        point->x + marker_radius,
                        point->y + marker_radius + offset,
                        highlighted_color.r, highlighted_color.g,
                        highlighted_color.b, highlighted_color.a);
        image.draw_line(point->x - marker_radius,
                        point->y + marker_radius + offset,
                        point->x + marker_radius,
                        point->y - marker_radius + offset,
                        highlighted_color.r, highlighted_color.g,
                        highlighted_color.b, highlighted_color.a);
    }
    drawHighlightedName(highlighted_name, *point);
    ++render_stats.highlighted_nodes;
    return true;
}

void fpga::Visualizer::drawRoutes(int x, int y,
                                  const std::string& highlighted_net)
{
    if (!crossbars_drawn || center.x != x || center.y != y) {
        drawCB(x, y);
    }
    drawRoutes(highlighted_net);
}

void fpga::Visualizer::writePNG(const std::string& filename)
{
    image.write(filename);
}

void fpga::Visualizer::visualize(int x, int y, const std::string& filename)
{
    drawCB(x, y);
    drawRoutes();
    writePNG(filename);
}

fpga::Visualizer::Color fpga::Visualizer::pixel(int x, int y) const
{
    if (!image.image_data || x < 0 || y < 0
        || x >= image.width || y >= image.height) {
        return {};
    }
    size_t index = (static_cast<size_t>(y) * image.width + x) * 4;
    return {image.image_data[index], image.image_data[index + 1],
            image.image_data[index + 2], image.image_data[index + 3]};
}
