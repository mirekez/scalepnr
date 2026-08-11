#include "DeviceFormat.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void package_pin_header_does_not_replace_first_pin()
{
    auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    std::filesystem::path path = std::filesystem::temp_directory_path()
        / ("scalepnr_package_pins_" + std::to_string(stamp) + ".csv");
    {
        std::ofstream output(path);
        output << "pin,bank,site,tile,pin_function\n";
        output << "FIRST,1,PIN_X0Y1,EDGE_X2Y3,FIRST_FUNCTION\n";
        output << "LAST,2,PIN_X0Y2,EDGE_X4Y5,LAST_FUNCTION\n";
    }

    std::vector<PinSpec> pins;
    bool loaded = readPackagePins(path.string(), pins);
    std::filesystem::remove(path);

    require(loaded, "package pin CSV failed to load");
    require(pins.size() == 2, "package pin CSV did not retain both data rows");
    require(pins[0].name == "FIRST" && pins[0].pos == Coord{2, 3},
        "first package pin was replaced by the CSV header");
    require(pins[1].name == "LAST" && pins[1].pos == Coord{4, 5},
        "last package pin was not parsed correctly");
}

}

int main()
{
    try {
        package_pin_header_does_not_replace_first_pin();
    }
    catch (const std::exception& error) {
        std::cerr << "device_format_test failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
