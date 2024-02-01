

#pragma once

#include <chrono>
#include <thread>

namespace vic
{

// wrapper for Try catch when we are not interested in failure
template <typename TLambda>
void Try(TLambda lambda)
{
    try
    {
        lambda();
    }
    catch(...)
    { }
}

template <typename TLambda, typename TDuration, typename TPollingRate>
bool Deadline(TLambda lambda, TDuration timeout, TPollingRate pollingRate)
{
    // todo: constrain TLambda to a lambda that returns a bool and has no parameters

    const auto startTime = std::chrono::high_resolution_clock::now();
    while(startTime + timeout > std::chrono::high_resolution_clock::now())
    {
        Try([&]() {
            if(lambda())
                return true;
        });

        std::this_thread::sleep_for(pollingRate);
    }
    return false;
}

template <typename TLambda, typename TDuration>
bool Deadline(TLambda lambda, TDuration timeout)
{
    return Deadline(lambda, timeout, std::chrono::milliseconds(10));
}

} // namespace vic