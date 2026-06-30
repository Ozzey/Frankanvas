#pragma once

#include "errors/exceptions.h"

#include <chrono>
#include <thread>

namespace CppCommon
{
class Thread
{
public:
    template <typename Function>
    static std::thread Start(Function &&function)
    {
        return std::thread(std::forward<Function>(function));
    }

    static void Yield()
    {
        std::this_thread::yield();
    }

    static void Sleep(size_t milliseconds)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }

    static std::thread::id CurrentThreadId()
    {
        return std::this_thread::get_id();
    }
};
}
