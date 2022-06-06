#pragma once

#include <chrono>

namespace vic
{
class CTimer
{
public:
    CTimer()
        : mStart(std::chrono::high_resolution_clock::now())
    { }

    using ClockType = std::chrono::high_resolution_clock;

    template <typename T = double, typename R = std::ratio<1>>
    std::chrono::duration<T, R> GetTime() const
    {
        const auto now = ClockType::now();
        return std::chrono::duration_cast<std::chrono::duration<T, R>>(now - mStart);
    }

    void Reset() { mStart = std::chrono::high_resolution_clock::now(); }

private:
    std::chrono::time_point<ClockType> mStart;
};
} // namespace vic