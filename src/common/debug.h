#define GEAR_DEBUG(a...) {} // { std::print(stderr, a) }
#define GEAR_WARNING(a...) { std::print(stderr, "WARNING: " a); }
