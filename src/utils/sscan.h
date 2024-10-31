#pragma once

#include <print>
#include <string_view>
#include <spanstream>

inline int sscan(std::ispanstream& ss, std::string_view format)
{
    return 0;
}

template <typename T, typename... Args>
inline int sscan(std::ispanstream& ss, std::string_view format, T* var, Args*... args)
{
    size_t saved_pos = ss.tellg();
    if (saved_pos == (size_t)-1) {
        return 0;
    }
    auto data = std::string_view(ss.span().data() + ss.tellg(), ss.span().size() - ss.tellg());
//std::print("~{},{}~", std::string(data).c_str(), std::string(format).c_str());
    // find needle before {} and search for it in data
    size_t format_first;
    if ((format_first = format.find("{}")) == (size_t)-1) {
        return 0;
    }
    if (format_first && data.compare(0, std::string::npos, &format.front(), format_first) != 0) {
        return 0;
    }
    // try to find next {}, take a next needle between {} and {} and search for it in data
    size_t search_next = (size_t)-1;
    size_t found_next = (size_t)-1;
    size_t found_size;
    if (format.size() != format_first + 2 && (search_next = format.find("{}", format_first + 2)) != (size_t)-1) {
        found_size = search_next - format_first - 2;
        found_next = data.find(&format.front() + format_first + 2, format_first, found_size);
        if (found_next == (size_t)-1) {  // present in format but not found in data
//std::print("${} {}$", search_next, found_size);
            return 0;
        }
    }
    // if we found both needles - take data between them, if just a one - take whole data after first
    std::string_view part(&data.front() + format_first, found_next == (size_t)-1 ? (data.size() - format_first) : found_next);
    std::ispanstream ss_var(part);
    if (!(ss_var >> *var)) {
//std::print("+++++++++");
        ss.seekg(saved_pos);
        return 0;
    }
//std::print("#{} {}#", std::string(part), var);
    ss.ignore(ss_var.tellg() == -1 ? part.size() : (size_t)ss_var.tellg());
    if (found_next != (size_t)-1) {
        //if ((size_t)ss.tellg() - saved_pos != found_next) {  // must be no other characters after the value
//std::print("^{} {} {}^", (int)ss.tellg() - saved_pos, found_next, (int)ss_var.tellg());
        //    ss.seekg(saved_pos);
        //    return 0;
        //}

        ss.ignore(found_size);
        int ret = sscan(ss, std::string_view(&format.front() + search_next, format.size() - search_next), args...);
        if (!ret) {
            ss.seekg(saved_pos);
            return 0;
        }
//std::print("!{}!", (int)ss.tellg());
        return ret + 1;
    }
    else {
//std::print("!{}!", (int)ss.tellg());
        return 1;  // all data was eaten by current {}
    }
}

template <typename T, typename... Args>
inline int sscan(std::string_view s, std::string_view format, T* var, Args*... args)
{
    std::ispanstream ss(s);
    return sscan(ss, format, var, args...);
}
