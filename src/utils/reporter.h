#include <print>
#include <format>
#include <map>
#include <string>
#include <vector>
#include <ranges>

namespace reporter
{

struct key_line
{
    std::vector<std::pair<double,std::string>> keys;
    std::vector<std::string> values;
    bool use_value_as_key = true;
};

inline bool operator <(const key_line& a, const key_line& b)
{
    for (size_t i=0; i<std::min(a.keys.size(),b.keys.size()); ++i) {
       if (a.keys[i].first < b.keys[i].first) return true;
       if (a.keys[i].first > b.keys[i].first) return false;
       if ((a.use_value_as_key&&a.values.size()>i?a.values[i]:a.keys[i].second) < (b.use_value_as_key&&b.values.size()>i?b.values[i]:b.keys[i].second)) return true;
       if ((a.use_value_as_key&&a.values.size()>i?a.values[i]:a.keys[i].second) > (b.use_value_as_key&&b.values.size()>i?b.values[i]:b.keys[i].second)) return false;
    }
    return false;
}

struct builder
{
    std::multimap<key_line,int> map;
    std::vector<size_t> colwidth;
    const size_t INC = 3;

    void insert(key_line&& line)
    {
        size_t num = 0;
        for (const std::string& str : line.values) {
            if (colwidth.size() < num+1) {
                colwidth.resize(num+1);
            }
            if (str.length() > colwidth[num]) {
                colwidth[num] = str.length();
            }
            ++num;
        }

        map.emplace(std::move(line), 0);
    }

    void print()
    {
        for (auto& it : map) {
            size_t num = 0;
            std::print("|");
            for (const std::string& str : it.first.values) {
                std::print("{:<{}}|", str, colwidth[num]+INC-1);
                ++num;
            }
            std::print("\n");
        }
    }

    void print_table()
    {
        for (size_t d = 0; d < colwidth.size(); ++d) {
            for (size_t i=0; i < colwidth[d]+INC; ++i) {
                std::print("-");
            }
        }
        std::print("-\n");
        print();
        for (size_t d = 0; d < colwidth.size(); ++d) {
            for (size_t i=0; i < colwidth[d]+INC; ++i) {
                std::print("-");
            }
        }
        std::print("-\n");
    }
};

}
/*
int main()
{
    reporter p;
    p.insert({{1,"111"}, {2, "222"}, {3, "333"}}, {"111", "222", "333"});
    p.insert({{5,"111"}, {2, "222"}, {3, "333"}}, {"115", "222", "333"});
    p.insert({{5,"111"}, {5, "225"}, {3, "333"}}, {"115", "222", "333"});
    p.print_table();
}
*/
