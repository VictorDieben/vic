#include <algorithm>
#include <cstddef>

namespace vic
{
template <typename T>
    requires std::integral<T>
constexpr T Parent(const T index)
{
    return index % 2 == 0 // note: not using floor(i/2), because it is not constexpr
               ? (index / 2) - 1
               : (index - 1) / 2;
}

template <typename T>
    requires std::integral<T>
constexpr T LeftChild(const T index)
{
    return (2 * index) + 1;
}

template <typename T>
    requires std::integral<T>
constexpr T RightChild(const T index)
{
    return (2 * index) + 2;
}

template <typename TIter, typename TCompare>
    requires std::random_access_iterator<TIter>
constexpr TIter decrease_key(TIter begin, TIter decrease, TCompare compare)
{
    std::size_t index = std::distance(begin, decrease);
    while(index != 0)
    {
        const auto iParent = Parent<std::size_t>(index);
        if(compare(*(begin + iParent), *(begin + index)))
        {
            std::iter_swap(begin + iParent, begin + index);
            index = iParent;
        }
        else
            break;
    }

    return begin + index;
}

template <typename TIter, typename TCompare>
    requires std::random_access_iterator<TIter>
constexpr TIter increase_key(TIter begin, TIter end, TIter increase, TCompare compare)
{
    const std::size_t size = std::distance(begin, end);
    std::size_t index = std::distance(begin, increase);

    while(index < size)
    {
        const auto leftChild = LeftChild<std::size_t>(index);
        const auto rightChild = RightChild<std::size_t>(index);

        if(leftChild >= size)
            break; // reached end of array

        // swap with best child
        if(rightChild >= size || // only left child is valid
           compare(*(begin + rightChild), *(begin + leftChild)))
        {
            if(compare(*(begin + leftChild), *(begin + index)))
                break;

            // swap with left child
            std::iter_swap(begin + index, begin + leftChild);
            index = leftChild;
        }
        else
        {
            if(compare(*(begin + rightChild), *(begin + index)))
                break;

            // swap with left child
            std::iter_swap(begin + index, begin + rightChild);
            index = rightChild;
        }
    }

    return begin + index;
}

template <typename TIter, typename TCompare>
    requires std::random_access_iterator<TIter>
void update_heap(TIter begin, TIter end, TIter update, TCompare compare)
{
    assert(begin <= update && update < end);
    // after updating the value of a key, resort the heap at a specific point
    TIter current = update;

    // first try to move the value up in the heap if needed
    while(true)
    {
        const std::size_t iCurrent = (std::size_t)std::distance(begin, current);
        if(iCurrent == 0)
            break;

        const std::size_t iParent = Parent<>(iCurrent);

        if(compare(*(begin + iParent), *(begin + iCurrent)))
        {
            // parent is smaller than child, all is good
        }
        else
        {
            // swap parent and current
            std::iter_swap(begin + iParent, begin + iCurrent);
            current = begin + iParent;
        }
    }
}

template <typename TIter, typename TCompare>
    requires std::random_access_iterator<TIter>
void update_heap(TIter begin, TIter end, TIter update)
{
    update_heap(begin, end, update, std::less{});
}

} // namespace vic