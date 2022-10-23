#pragma once

#include <cstddef>

#include <array>
#include <mutex>
#include <optional>

namespace vic
{
namespace memory
{

// Simple implementation of a ring buffer.
// todo: Head and Tail should have separate mutexes (or be atomic)
// right now, it is still quite inefficient
// todo: fix false sharing
template <typename T, std::size_t N>
class RingBuffer
{
public:
    static_assert(N > 1);

    RingBuffer() = default;

    bool Push(const T& item)
    {
        std::scoped_lock lock{mMutex};
        std::size_t nextHead = NextIndex(mHead);
        if(nextHead == mTail)
            return false;
        mBuffer[mHead] = item;
        mHead = nextHead;
        return true;
    }

    template <typename TIter>
    std::size_t Push(TIter begin, TIter end)
    {
        std::scoped_lock lock{mMutex};
        // push items in range, return how many items were pushed
        return 0;
    }

    std::optional<T> Pop()
    {
        std::scoped_lock lock{mMutex};
        if(mHead == mTail)
            return std::nullopt;

        T object = mBuffer[mTail];
        mTail = NextIndex(mTail);
        return object;
    }

    std::size_t Capacity() const { return N; }
    std::size_t Size() const
    {
        std::scoped_lock lock{mMutex}; //
        return (mHead >= mTail) //
                   ? mHead - mTail
                   : (mHead + N) - mTail;
    }

    static constexpr std::size_t NextIndex(std::size_t i)
    {
        return (i + 1) % N; //
    }

private:
    alignas(64) std::size_t mHead{0};
    alignas(64) std::size_t mTail{0};
    std::array<T, N> mBuffer{};

    // for now, just lock each public function,
    // we can later try to implement fences etc.
    mutable std::mutex mMutex;
};

} // namespace memory
} // namespace vic