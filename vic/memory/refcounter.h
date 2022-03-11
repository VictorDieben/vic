#pragma once


namespace vic
{
namespace memory
{
// shared_ptr, without the weak_ptr stuff
template <typename T>
class RefCounter
{
public:
    RefCounter() = default;
private:
    T* mObject{ nullptr };
};
}
}