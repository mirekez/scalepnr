#pragma once

#include "Crossbar.h"
#include "Types.h"
#include "png_draw.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fpga {

struct Device;
struct Tile;
struct Wire;

// Renders a fixed 5x5 window of numeric crossbar state and routed wires.
class Visualizer
{
public:
    static constexpr int region_tiles = 5;
    static constexpr int tile_pixels = 512;
    static constexpr int image_pixels = region_tiles * tile_pixels;

    struct Point
    {
        int x = -1;
        int y = -1;
    };

    struct Color
    {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        uint8_t a = 0;

        bool operator==(const Color&) const = default;
    };

    struct Stats
    {
        size_t tiles = 0;
        size_t crossbars = 0;
        size_t attached_tiles = 0;
        size_t tiles_without_crossbar = 0;
        size_t nodes = 0;
        size_t local_nodes = 0;
        size_t joint_nodes = 0;
        size_t src_nodes = 0;
        size_t dst_nodes = 0;
        size_t node_titles = 0;
        size_t title_anchor_collisions = 0;
        size_t occupied_nodes = 0;
        size_t routes = 0;
        size_t route_segments = 0;
        size_t highlighted_labels = 0;
        size_t highlighted_nodes = 0;
    };

    Visualizer();
    explicit Visualizer(Device& device);

    // Prepare and draw the crossbars in the 5x5 window centered on x,y.
    void drawCB(int x, int y);
    // Draw every route fragment touching the prepared 5x5 window; the named
    // route is highlighted in red when a logical or physical name matches.
    void drawRoutes(const std::string& highlighted_net = {});
    void drawRoutes(int x, int y, const std::string& highlighted_net = {});
    // Draw an explicit committed or partial route not yet present in a tile index.
    void drawRoute(const std::vector<Wire>& route, bool highlighted = false,
                   const std::string& highlighted_name = {});
    // Mark an unresolved endpoint when no partial route exists to highlight.
    bool highlightNode(Coord tile, CBNodeNameType type, int node,
                       const std::string& highlighted_name = {});
    // Label the failure window when no surviving route reaches its center.
    void drawFailureLabel(const std::string& name);
    void writePNG(const std::string& filename);
    void visualize(int x, int y, const std::string& filename);

    std::optional<Point> nodePosition(Coord tile, CBNodeNameType type,
                                      int node) const;
    Color pixel(int x, int y) const;
    const Stats& stats() const { return render_stats; }

private:
    struct NodeKey
    {
        int x = -1;
        int y = -1;
        uint16_t type = 0;
        uint16_t node = 0;

        bool operator==(const NodeKey&) const = default;
    };

    struct NodeKeyHash
    {
        size_t operator()(const NodeKey& key) const;
    };

    struct NodeDraw
    {
        NodeKey key;
        Point point;
        bool occupied = false;
    };

    enum class TextDirection : uint8_t
    {
        right,
        down,
        up,
    };

    struct TextDraw
    {
        std::string text;
        Point point;
        TextDirection direction = TextDirection::right;
    };

    Device& device;
    png_draw image;
    Coord center{-1, -1};
    bool crossbars_drawn = false;
    std::unordered_map<NodeKey, Point, NodeKeyHash> node_positions;
    std::unordered_map<NodeKey, Coord, NodeKeyHash> node_directions;
    std::vector<NodeDraw> drawn_nodes;
    std::vector<TextDraw> node_titles;
    bool highlighted_label_drawn = false;
    Stats render_stats;

    bool inRegion(Coord coord) const;
    Point tileOrigin(Coord coord) const;
    // Resolve a resource-side view to the tile owning its crossbar leases.
    Coord routingCoord(Coord coord) const;
    // Build numeric directions for visible SRC/DST edge placement.
    void buildJumpDirections();
    // Draw one valid device tile and register all of its node positions.
    void drawTile(Tile& tile);
    // Repaint node dots above route lines using current lease state.
    void drawNodeDots();
    // Repaint black node labels above route lines and node dots.
    void drawNodeTitles();
    // Draw scalable bitmap text in a horizontal or edge-oriented direction.
    void drawText(const std::string& text, Point point,
                  TextDirection direction, const Color& color,
                  int font_width, int font_height, int advance,
                  int bold_weight = 0);
    // Draw one heavy, single-pass tile-coordinate watermark.
    void drawCoordinateText(const std::string& text, Point point);
    // Draw the selected failed-net name once beside its first visible point.
    void drawHighlightedName(const std::string& name, Point start);
    // Draw one physical route binding through registered numeric positions.
    std::optional<Point> drawPhysicalRoute(const std::vector<Wire>& route,
                                           const Color& color,
                                           int line_width = 1);
};

}
