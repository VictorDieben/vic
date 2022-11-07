#pragma once

#include <cstddef>

#include <array>
#include <mutex>
#include <optional>
#include <type_traits>

namespace vic
{
namespace memory
{

// todo: use in RingBuffer
enum class QueueType
{
    SingleProducerSingleConsumer,
    MultiProducerSingleConsumer,
    SingleProducerMultiConsumer,
    MultiProducerMultiConsumer
};

// Simple implementation of a ring buffer.
// Could probably be entirely lock free.
// But that's not worth the headacke.
// todo: use in place construction/destruction, so items in array are actually destroyed
// todo: condition variable
// todo: for single producer / single consumer, the mutexes can be removed
template <typename T, std::size_t N>
class RingBuffer
{
public:
    static_assert(N > 1);
    static_assert(std::is_default_constructible_v<T>); // needed for emtpy array
    static_assert(std::is_copy_constructible_v<T>); // needed for inserting into array (move?)

    RingBuffer() = default;

    bool Push(const T& item) noexcept
    {
        std::scoped_lock lock{mPushMutex}; // only needed for multi-producer
        const std::size_t head = mHead; // copy atomic value
        const std::size_t nextHead = NextIndex(head);
        if(nextHead == mTail)
            return false;
        mBuffer[head] = item;
        mHead = nextHead;
        return true;
    }

    std::optional<T> Pop() noexcept
    {
        std::scoped_lock lock{mPopMutex}; // only needed for multi-consumer
        const std::size_t tail = mTail;
        if(mHead == tail)
            return std::nullopt;

        T item = mBuffer[tail];
        if constexpr(!std::is_trivially_destructible_v<T>)
            mBuffer[tail] = {}; // reset, only needed if T destructor is non-trivial
        mTail = NextIndex(tail);
        return item;
    }

    std::size_t Capacity() const { return N; }
    std::size_t Size() const noexcept
    {
        std::size_t head;
        std::size_t tail;
        {
            std::scoped_lock lock(mPushMutex, mPopMutex);
            head = mHead;
            tail = mTail;
        }
        return (head >= tail) //
                   ? head - tail
                   : (head + N) - tail;
    }

    void Clear()
    {
        std::scoped_lock lock(mPushMutex, mPopMutex);
        mHead = 0;
        mTail = 0;
        mBuffer = {}; // reset all
        // todo: replace reset with destruction of in-place objects
    }

    static constexpr std::size_t NextIndex(std::size_t i)
    {
        return (i + 1) % N; //
    }

private:
    // create two separate mutexes for pushing and popping.
    // this way, we can use head and tail as atomic values, without worrying about collisions
    alignas(64) mutable std::mutex mPushMutex;
    alignas(64) mutable std::mutex mPopMutex;

    alignas(64) std::atomic<std::size_t> mHead{0};
    alignas(64) std::atomic<std::size_t> mTail{0};

    std::array<T, N> mBuffer{};
};

} // namespace memory
} // namespace vic