#pragma once

#include <format>
#include <ostream>
#include <string>
#include <string_view>
#include <unordered_set>

namespace fpga {

// Stores immutable repeated annotation text once and keeps only a stable pointer in each owner.
class InternedString
{
public:
    InternedString() = default;
    InternedString(const char* value) { assign(value ? std::string_view(value) : std::string_view{}); }
    InternedString(const std::string& value) { assign(value); }
    InternedString(std::string_view value) { assign(value); }

    InternedString& operator=(const char* value)
    {
        assign(value ? std::string_view(value) : std::string_view{});
        return *this;
    }

    InternedString& operator=(const std::string& value)
    {
        assign(value);
        return *this;
    }

    InternedString& operator=(std::string_view value)
    {
        assign(value);
        return *this;
    }

    [[nodiscard]] bool empty() const { return value_ == nullptr; }
    [[nodiscard]] size_t size() const { return empty() ? 0 : value_->size(); }
    [[nodiscard]] const char* c_str() const { return str().c_str(); }
    [[nodiscard]] const std::string& str() const
    {
        static const std::string empty_value;
        return value_ ? *value_ : empty_value;
    }
    [[nodiscard]] std::string_view view() const { return str(); }

    operator const std::string&() const { return str(); }

    friend bool operator==(const InternedString&, const InternedString&) = default;
    friend bool operator==(const InternedString& left, std::string_view right)
    {
        return left.view() == right;
    }
    friend bool operator==(std::string_view left, const InternedString& right)
    {
        return left == right.view();
    }
    friend bool operator==(const InternedString& left, const std::string& right)
    {
        return left.view() == right;
    }
    friend bool operator==(const std::string& left, const InternedString& right)
    {
        return left == right.view();
    }

    friend std::ostream& operator<<(std::ostream& out, const InternedString& value)
    {
        return out << value.view();
    }

private:
    // Standard unordered-container references remain stable across rehashes.
    static const std::string* intern(std::string_view value)
    {
        static std::unordered_set<std::string> values;
        auto [it, inserted] = values.emplace(value);
        (void)inserted;
        return &*it;
    }

    void assign(std::string_view value) { value_ = value.empty() ? nullptr : intern(value); }

    const std::string* value_ = nullptr;
};

}

template<>
struct std::formatter<fpga::InternedString, char> : std::formatter<std::string_view, char>
{
    auto format(const fpga::InternedString& value, std::format_context& context) const
    {
        return std::formatter<std::string_view, char>::format(value.view(), context);
    }
};
