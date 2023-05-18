#pragma once

namespace vic
{
namespace ecs
{

template <typename... TSystems>
class SystemExecutor
{
public:
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

private:
    std::tuple<TSystems...> mSystems;
};

} // namespace ecs
} // namespace vic