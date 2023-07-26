#pragma once

namespace vic
{
namespace ecs
{

template <typename... TSystems>
class SystemExecutor
{
public:
    static_assert((std::is_pointer_v<TSystems> && ...), "TSystems should be passed as a pointer!");

    SystemExecutor(TSystems&&... systems)
        : mSystems(std::forward<decltype(systems)>(systems)...)
    {
        // todo: setup thread pool, mutex and condition variables for each system, etc.
    }

    void Run()
    {
        // Run all systems,
        // ensuring the order of execution,
        // but allowing systems that do not conflict with each other to run simultaneously.
    }

    // todo: RunWhile, which loops back to the first system after completing the end.
    // there's no need to complete everything before starting the next loop.
    void RunWhile()
    {
        while(false)
        {
            // loop over all components, and then loop back to the start.
            // each system is only allowed to be in the queue once,
        }
    }

private:
    std::tuple<TSystems...> mSystems;
};

} // namespace ecs
} // namespace vic