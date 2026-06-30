#pragma once

#include <atomic>
#include <cstdint>

namespace CppCommon
{
class UUID
{
public:
    UUID() = default;
    explicit UUID(uint64_t value) : value_(value) {}

    static UUID Sequential()
    {
        static std::atomic<uint64_t> next{1};
        return UUID(next.fetch_add(1));
    }

    bool operator<(const UUID &other) const noexcept { return value_ < other.value_; }
    bool operator==(const UUID &other) const noexcept { return value_ == other.value_; }
    bool operator!=(const UUID &other) const noexcept { return !(*this == other); }

private:
    uint64_t value_{0};
};
}
