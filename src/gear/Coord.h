#include <vector>

namespace gear {

  struct Coord {
    long x, y;

    Coord operator+(const Coord& other) const {
      return Coord{x + other.x, y + other.y};
    }

    Coord operator-(const Coord& other) const {
      return Coord{x - other.x, y - other.y};
    }
  };

  struct CoordRect
  {
    Coord beg;
    Coord end;
  };

  struct CoordList
  {
    std::vector<CoordRect> list;
  };


}
