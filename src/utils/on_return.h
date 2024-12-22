#pragma once

#include <functional>

class on_return {
public:
    on_return(std::function<void()> lambda) : lambda_(std::move(lambda)) {}
    ~on_return() { lambda_(); }

private:
    std::function<void()> lambda_;
};

