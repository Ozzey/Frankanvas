#pragma once

#include <stdexcept>
#include <string>

namespace CppCommon
{
class ArgumentException : public std::invalid_argument
{
public:
    explicit ArgumentException(const std::string &message)
        : std::invalid_argument(message) {}
};
}
