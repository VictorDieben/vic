#pragma once

#include <algorithm>
#include <memory>

namespace vic
{
namespace memory
{

template <typename T>
class RefCounted;

// shared_ptr, without the weak_ptr stuff.
// not suitable for multi-threading
template <typename T>
class RefCounted
{
public:
    // nested class that stores object and ref count
    template <typename T>
    class RefWrapper
    {
    public:
        template <typename... Args>
        RefWrapper(Args&&... args)
            : mData{std::forward<Args>(args)...}
        { }

    private:
        T mData;
        std::size_t mCount{0};

        void Add() { mCount++; }
        bool Subtract()
        {
            mCount--;
            return mCount == 0;
        }
        std::size_t GetCount() const { return mCount; }
        friend RefCounted<T>;
    };

    RefCounted() = default;

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
        if(mObject && mObject->Subtract())
            Destroy();
    }

    T* Get() { return mObject ? &(mObject->mData) : nullptr; }
    const T* Get() const { return mObject ? &(mObject->mData) : nullptr; }

    T* operator->() { return &(mObject->mData); }
    const T* operator->() const { return &(mObject->mData); }

    std::size_t use_count() const { return mObject ? mObject->GetCount() : 0; }

private:
    RefWrapper<T>* mObject{nullptr};

    void Destroy()
    {
        delete mObject;
        mObject = nullptr;
    }
};

} // namespace memory

template <typename T>
struct ref
{
    ref(T* ptr)
        : mPtr(ptr)
    { }

private:
    T* mPtr;
};

template <typename T, typename... Args>
ref<T> make_refcounted(Args... args)
{
    T* raw = new T(std::forward<Args>(args)...);
    return ref(raw);
}

//
//
//

//template <typename T>
//concept ConceptIsIntrusiveCRTP = requires(T t) {
// T::TmpIsIntrusiveCRTP == true; //
//{
//    t.count()
//} -> uint64_t;
//};

template <typename T>
// requires ConceptIsIntrusiveCRTP<T> // note: seems not to work with crtp
struct intrusive
{
    intrusive() = default;
    intrusive(T* ptr)
        : mPtr(ptr)
    {
        mPtr->Add();
    }
    ~intrusive()
    {
        if(mPtr)
            if(mPtr->Subtract())
                delete mPtr; // delete here, because I don't think we can destroy an object in its own member function
        mPtr = nullptr;
    }

    intrusive(intrusive&& other) noexcept
    {
        this->mPtr = other.mPtr;
        other.mPtr = nullptr;
    }
    intrusive& operator=(intrusive&& other) noexcept
    {
        this->mPtr = other.mPtr;
        other.mPtr = nullptr;
        return *this;
    }
    intrusive(intrusive& other)
    {
        this->mPtr = other.mPtr;
        this->mPtr->Add();
    }
    intrusive& operator=(intrusive& other)
    {
        this->mPtr = other.mPtr;
        this->mPtr->Add();
        return *this;
    }

    bool empty() const { return mPtr == nullptr; }
    std::size_t count() const { return mPtr ? mPtr->count() : 0; }
    void clear()
    {
        if(mPtr)
            if(mPtr->Subtract())
                delete mPtr; // delete here, because I don't think we can destroy an object in its own member function
        mPtr = nullptr;
    }

    T& operator->() const { return *mPtr; }
    operator bool() const { return mPtr != nullptr; }

private:
    T* mPtr{nullptr};
};

// crtp class
template <typename T>
struct intrusive_ref
{
    std::size_t count() const { return mCount; }

    // temp solution, find a better one
    static constexpr bool TmpIsIntrusiveCRTP = true;

    void Add() { mCount++; }
    bool Subtract()
    {
        mCount--;
        return mCount == 0; // return true if we need to destruct
    }

private:
    std::size_t mCount{};
};

template <typename T, typename... Args>
intrusive<T> make_intrusive(Args&&... args)
{
    return intrusive<T>(new T{std::forward<Args>(args)...});
}

} // namespace vic