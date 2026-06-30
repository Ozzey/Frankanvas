#pragma once

#include <exception>
#include <iostream>

namespace CppCommon
{
inline void fatality(const std::exception &ex)
{
    std::cerr << ex.what() << std::endl;
    std::terminate();
}

inline void fatality(const char *message)
{
    std::cerr << message << std::endl;
    std::terminate();
}
}

using CppCommon::fatality;
