
#include <vector>

namespace vic
{

template <typename T>
class FlatLinkedList
{
public:
    FlatLinkedList() = default;

    // todo: instead of wrapping std::vector, we should manually maintain a c array (std::unique_ptr<T[]>).
    // this way, we can force a flush when the buffer needs to expand, and we can flush to a same-size buffer if there is space enough, but on weird indices

    using Index = std::size_t;

    using value_t = T;

    template <typename T>
    struct Node
    {
        Index previous;
        Index next;
        T data;
    };

    T& at(const std::size_t index)
    {
        // todo: for indices near the end, count backwards
        assert(index < mFilled);
        std::size_t dataIndex = mFirstIndex;

        for(std::size_t i = 0; i < index; ++i)
            dataIndex = mData.at(dataIndex).next;

        return mData.at(dataIndex).data;
    }

    const T& at(const std::size_t index) const
    {
        // todo: for indices near the end, count backwards
        assert(index < mFilled);
        std::size_t dataIndex = mFirstIndex;

        for(std::size_t i = 0; i < index; ++i)
            dataIndex = mData.at(dataIndex).next;

        return mData.at(dataIndex).data;
    }

    value_t& operator[](std::size_t idx) { return at(idx); }
    const value_t& operator[](std::size_t idx) const { return at(idx); }

    T& front() { return mData.at(mFirstIndex); }
    const T& front() const { return mData.at(mFirstIndex); }

    T& back() { return mData.at(mLastIndex); }
    const T& back() const { return mData.at(mLastIndex); }

    void Insert(T data, Index index)
    {
        // todo: insert item, update index ptrs in previous and next
    }

    void PushFront(T data)
    {
        if(mFilled == 0)
        {
            mData.push_back(Node{0, 0, data});
            mFilled++;
            mFirstIndex = 0;
            mLastIndex = 0;
        }
        else
        {
            mData.push_back(Node{mData.size(), mFirstIndex, data});
            std::size_t newIndex = mData.size() - 1;
            mData.at(mFirstIndex).previous = newIndex;
            mFilled++;
            mFirstIndex = newIndex;
        }
    }
    void PushBack(T data)
    {
        if(mFilled == 0)
        {
            mData.push_back(Node{0, 0, data});
            mFilled++;
            mFirstIndex = 0;
            mLastIndex = 0;
        }
        else
        {
            mData.push_back(Node{mLastIndex, mLastIndex + 1, data});
            mData.at(mLastIndex).next = mLastIndex + 1;
            mFilled++;
            mLastIndex = mLastIndex + 1;
        }
    }

    void PopFront()
    {
        assert(mFilled != 0); //
        if(mFilled == 1)
            Reset();
        else
        {
            Node<T>& first = mData.at(mFirstIndex);
            Node<T>& next = mData.at(first.next);

            next.previous = first.next; // second last item is now last}
            mFilled--;
            mFirstIndex = first.next;
            // todo: properly delete item at front
        }
    }
    void PopBack()
    {
        assert(mFilled != 0); //
        if(mFilled == 1)
            Reset();
        else
        {
            Node<T>& last = mData.at(mLastIndex);
            Node<T>& previous = mData.at(last.previous);

            previous.next = last.previous; // second last item is now last}
            mFilled--;

            // todo: properly delete item at back
        }
    }

    void Flush()
    {
        // Force the linked list to flush to a new buffer, and write all elements in sorted order

        std::vector<Node<T>> newData;
        newData.reserve(mFilled);

        std::size_t i = 0;
        std::size_t dataIndex = mFirstIndex;
        for(; i < mFilled; ++i)
        {
            newData.push_back(Node<T>{i == 0 ? 0 : i - 1, //
                                      std::min(i + 1, mFilled - 1),
                                      std::move(mData.at(dataIndex).data)});
            dataIndex = mData.at(dataIndex).next;
        }
        std::swap(mData, newData);
        mFirstIndex = 0;
        mLastIndex = mData.size() - 1;
    }

    std::size_t size() const { return mFilled; }
    std::size_t capacity() const { return mData.capacity(); }
    bool empty() const { return mFilled == 0; }

private:
    std::vector<Node<T>> mData;

    std::size_t mFilled{0};

    std::size_t mFirstIndex{0}; // index in mData of first item in list
    std::size_t mLastIndex{0}; // index in mData of last item in list

    void Reset()
    {
        mFilled = 0;
        mData.clear();
        mFirstIndex = 0;
        mLastIndex = 0;
    }
};
} // namespace vic