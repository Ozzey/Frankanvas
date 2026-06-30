#pragma once

#include "time/timespan.h"

#include <chrono>

namespace CppCommon
{
class UtcTime
{
public:
    using time_point = std::chrono::system_clock::time_point;

    UtcTime() : value_(std::chrono::system_clock::now()) {}
    explicit UtcTime(time_point value) : value_(value) {}

    template <typename Clock, typename Duration>
    explicit UtcTime(const std::chrono::time_point<Clock, Duration> &value)
        : value_(std::chrono::time_point_cast<time_point::duration>(value)) {}

    time_point chrono() const noexcept { return value_; }

    UtcTime operator+(const Timespan &timespan) const
    {
        return UtcTime(value_ + timespan.chrono());
    }

private:
    time_point value_;
};
}
