#pragma once

#include <chrono>
#include <cstdint>

namespace CppCommon
{
class Timespan
{
public:
    using duration = std::chrono::system_clock::duration;

    Timespan() = default;
    explicit Timespan(duration value) : value_(value) {}

    template <typename Rep, typename Period>
    explicit Timespan(const std::chrono::duration<Rep, Period> &value)
        : value_(std::chrono::duration_cast<duration>(value)) {}

    static Timespan milliseconds(int64_t value) { return Timespan(std::chrono::milliseconds(value)); }
    static Timespan seconds(int64_t value) { return Timespan(std::chrono::seconds(value)); }
    static Timespan minutes(int64_t value) { return Timespan(std::chrono::minutes(value)); }
    static Timespan hours(int64_t value) { return Timespan(std::chrono::hours(value)); }

    duration chrono() const noexcept { return value_; }
    int64_t seconds() const noexcept
    {
        return std::chrono::duration_cast<std::chrono::seconds>(value_).count();
    }

private:
    duration value_{};
};
}
