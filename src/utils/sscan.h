#pragma once

#include <print>
#include <string_view>
#include <spanstream>

int sscan(std::string_view data, const std::string_view format)
{
    return 0;
}

template <typename T, typename... Args>
int sscan(std::string_view data, const std::string_view format, T& var, Args&... args)
{
//std::print("~{},{}~", std::string(data).c_str(), std::string(format).c_str());
    // find needle before {} and search for it in data
    size_t format_first;
    if ((format_first = format.find("{}")) == (size_t)-1) {
        return 0;
    }
    size_t data_first;
    if ((data_first = data.find(&format.front(), 0, format_first)) == (size_t)-1) {
        return 0;
    }
    // try to find next {}, take a next needle between {} and {} and search for it in data
    size_t search_next = (size_t)-1;
    size_t found_next = (size_t)-1;
    size_t found_size;
    if (format.size() != format_first + 2 && (search_next = format.find("{}", format_first + 2)) != (size_t)-1) {
        found_size = search_next - format_first - 2;
        found_next = data.find(&format.front() + format_first + 2, data_first + 1, found_size);
//std::print("!{},{},{},{}!", format_first, data_first, search_next, found_size);
    }
    // if we found both needles - take data between them, if just a one - take whole data after first
    std::string_view part(&data.front() + data_first + format_first, found_next == (size_t)-1 ? (data.size() - data_first - format_first) : (found_next - data_first));
//std::print("#{}#", std::string(part));
    std::ispanstream ss(part);
    if (!(ss >> var)) {
        return 0;
    }
    if (found_next != (size_t)-1) {
        return sscan(std::string_view(&data.front() + found_next + found_size, data.size() - found_next - found_size),
                            std::string_view(&format.front() + search_next, format.size() - search_next),
                            args...) + 1;
    }
    else {
        return 1;  // all data was eaten by current {}
    }
}
