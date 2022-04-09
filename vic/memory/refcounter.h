#pragma once

#include <algorithm>
#include <memory>

namespace vic
{
namespace memory
{

template <typename T>
class RefCounted;

template <typename T>
class RefWrapper : public T
{
public:
    template <typename... Args>
    RefWrapper(Args&&... args)
        : T(std::forward<Args>(args)...)
    { }

private:
    std::size_t mCount{0};

    void Add() { mCount++; }
    void Subtract() { mCount--; }
    std::size_t GetCount() const { return mCount; }
    friend RefCounted<T>;
};

//template <typename T>
//class RefWrapper
//{
//public:
//    template <typename... Args>
//    RefWrapper(Args&&... args)
//        : mData{std::forward<Args>(args)...}
//    { }
//
//private:
//    T mData;
//    std::size_t mCount{0};
//
//    void Add() { mCount++; }
//    void Subtract() { mCount--; }
//    std::size_t GetCount() const { return mCount; }
//    friend RefCounted<T>;
//};

// shared_ptr, without the weak_ptr stuff
template <typename T>
class RefCounted
{
public:
    // nested class that stores object and ref count

    template <typename... Args>
    RefCounted(Args&&... args)
    {
        mObject = new RefWrapper<T>(std::forward<Args>(args)...);
        mObject->Add();
    }

    RefCounted& operator=(RefCounted other)
    {
        mObject = other.mObject;
        mObject->Add();
        return *this;
    }

    RefCounted(RefCounted<T>& other)
    {
        mObject = other.mObject;
        mObject->Add();
    }

    ~RefCounted()
    {
        // check if we still point to an object (might have been moved)
        if(mObject)
        {
            mObject->Subtract();
            if(mObject->GetCount() == 0)
                Destroy();
        }
    }

    std::size_t use_count() const { return mObject->GetCount(); }

private:
    RefWrapper<T>* mObject{nullptr};

    void Destroy()
    {
        if(mObject)
            delete mObject;
        mObject = nullptr;
    }
};
} // namespace memory
} // namespace vic