#pragma once

#include <cstdint>
#include <span>

namespace vic
{

// a container similar to std::span (type erased contiguous range of T)
// Except that it allows for comparing between sub-trispans.
template <typename T> // don't bother with non-dynamic extend for now
class TriSpan
{
public:
    using element_type = T;
    using value_type = std::remove_cv_t<T>;
    using size_type = std::size_T;
    using difference_type = std::ptrdiff_t;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;

    // todo:
    template <bool Const>
    struct _Iterator
    { };
    using iterator = _Iterator<false>;
    using const_iterator = _Iterator<true>;

    iterator begin() { return iterator{}; }
    iterator end() { return iterator{}; }

    const_iterator begin() const { return const_iterator{}; }
    const_iterator end() const { return const_iterator{}; }

    const_iterator cbegin() const { return const_iterator{}; }
    const_iterator cend() const { return const_iterator{}; }

    constexpr size_type size() const noexcept { return mSize - mHead; }
    constexpr bool empty() const noexcept { return mHead == mSize; }

    constexpr reference at(const size_type i) const { return mPtr + mHead + i; }
    constexpr const_reference at(const size_type i) const { return mPtr + mHead + i; }

    static constexpr std::size_t extent = std::dynamic_extent; // no fixed extent for now

    constepxr TriSpan() = default;

private:
    T* mPtr;
    size_type mHead;
    size_type mSize;
};
} // namespace vic