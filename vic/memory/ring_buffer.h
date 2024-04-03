#pragma once

#include <cstddef>

#include <array>
#include <memory>
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
class ArrayRingBuffer
{
public:
    static_assert(N > 1);
    static_assert(std::is_default_constructible_v<T>); // needed for emtpy array
    static_assert(std::is_copy_constructible_v<T>); // needed for inserting into array (move?)

    ArrayRingBuffer() = default;

    bool TryPush(const T& item) noexcept
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

    // todo: instead of performing the modulus when setting head/tail, do it when reading.
    // this would solve the aba problem
    alignas(64) std::atomic<std::size_t> mHead{0};
    alignas(64) std::atomic<std::size_t> mTail{0};

    std::array<T, N> mBuffer{};
};

inline constexpr std::size_t NextRingIndex(const std::size_t i, //
                                           const std::size_t dataSize)
{
    return (i + 1) % dataSize; //
}

inline constexpr std::size_t PreviousRingIndex(const std::size_t i, //
                                               const std::size_t dataSize)
{
    return i == 0 ? dataSize - 1 : i - 1; //
}

//inline constexpr std::size_t ToRingIndex(const std::size_t i, //
//                                         const std::size_t dataSize)
//{
//    return i % dataSize;
//}

// non-thread safe ring buffer wrapper for vector
// mostly copies std::list/std::deque interface where possible.
template <typename T>
class RingBuffer
{
private:
    static_assert(std::is_default_constructible_v<T>, "RingBuffer requires T to be default constructible!");
    static_assert(std::is_move_constructible_v<T>, "RingBuffer requires T to be move constructible!");
    static_assert(std::is_trivially_destructible_v<T>, "RingBuffer requires T to be trivially destructible!");

    std::size_t mHead{0};
    std::size_t mTail{0};
    std::size_t mDataSize = 4;
    std::unique_ptr<T[]> mData;

public:
    RingBuffer() { mData = std::make_unique<T[]>(mDataSize); }

    using value_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = value_type&;
    using const_reference = const value_type&;

    size_type size() const { return mHead - mTail; }

    T& at(std::size_t i) { return mData[(mTail + i) % mDataSize]; }

    bool empty() const { return (mHead == mTail); }
    void clear()
    {
        mHead = 0;
        mTail = 0;
    }

    bool full() const { return (mHead - mTail) == mDataSize; }

    void push_back(const T& item)
    {
        if(full())
            Reallocate();

        mData[mHead % mDataSize] = item;
        mHead++;
    }
    void push_front(const T& item)
    {
        if(full())
            Reallocate();

        mTail = PreviousRingIndex(mTail, mDataSize);
        mData[mTail] = item;
        if(mHead < mTail)
            mHead += mDataSize;
    }

    T pop_front()
    {
        assert(!empty());
        const auto oldTail = mTail;
        mTail += 1;
        if((mTail / mDataSize) > 0)
        {
            mTail -= mDataSize;
            mHead -= mDataSize;
        }
        return mData[oldTail];
    }
    T pop_back()
    {
        assert(!empty());
        mHead -= 1;
        return mData[mHead]; // note: head points to one past the last
    }

    std::optional<T> try_pop_front()
    {
        if(empty())
            return std::nullopt;
        return pop_front();
    }

    std::optional<T> try_pop_back()
    {
        if(empty())
            return std::nullopt;
        return pop_back();
    }

    T& front()
    {
        assert(!empty());
        return mData[mTail % mDataSize];
    }
    T& back()
    {
        assert(!empty());
        return mData[(mHead - 1) % mDataSize];
    }

    const T& front() const
    {
        assert(!empty());
        return mData[mTail % mDataSize];
    }

    const T& back() const
    {
        assert(!empty());
        return mData[(mHead - 1) % mDataSize];
    }

private:
    void Reallocate()
    {
        const std::size_t newSize = 2 * mDataSize; // simply double, maybe look what vector does
        auto newData = std::make_unique<T[]>(newSize);

        std::size_t ni = 0;
        for(std::size_t i = mTail; i < mHead; ++i)
        {
            newData[ni] = std::move(mData[i % mDataSize]);
            ni++;
        }

        mTail = 0;
        mHead = mDataSize;
        mDataSize = newSize;
        mData = std::move(newData); // throws away previous array
    }
};

} // namespace memory
} // namespace vic