#pragma once

#include <algorithm>
#include <map>
#include <numeric>
#include <vector>

namespace vic
{
namespace memory
{

template <typename T, typename TLambda>
typename std::vector<T>::iterator insert_sorted(std::vector<T>& vec, const T& item, TLambda lambda)
{
    return vec.insert(std::upper_bound(vec.begin(), vec.end(), item, lambda), item);
}

template <typename T>
class Tree;

template <typename T>
struct TreeNode
{
    using NodeId = uint64_t;
    TreeNode() = default;
    TreeNode(const T& data, const NodeId id, const NodeId parent)
        : mData(data)
        , mId(id)
        , mParentId(parent)
    { }

    NodeId Id() const { return mId; }
    NodeId Parent() const { return mParentId; }

    T& Data() { return mData; }
    const T& Data() const { return mData; }

    bool IsRoot() const { return mId == mParentId; }

private:
    T mData{};
    NodeId mId{};
    NodeId mParentId{};

    friend Tree<T>; // Tree has direct access to ids
};

// container for tree. Nodes will be sorted on Id,
// has iterators for df and bf iteration.
template <typename T>
class Tree
{
public:
    using TreeNodeType = TreeNode<T>;
    using NodeId = typename TreeNodeType::NodeId;

    Tree() = default;

    TreeNodeType& NewRoot(const T& data)
    {
        assert(mNodes.empty());
        TreeNodeType newNode{data, mIdCounter, mIdCounter};
        mNodes.push_back(newNode);
        mIdCounter++;
        return mNodes.back();
    }
    TreeNodeType& NewNode(const T& data, const NodeId parent)
    {
        assert(TryGet(parent) != nullptr);
        TreeNodeType newNode{data, mIdCounter, parent};
        mNodes.push_back(newNode);
        mIdCounter++;
        return mNodes.back();
    }

    void Remove(const NodeId id)
    {
        // todo: once we find the first (last) related node,
        // we know what range we can remove
        for(int i = int(mNodes.size()) - 1; i >= 0; --i)
            if(IsNthChildOf(id, mNodes.at(i).Id()))
                mNodes.erase(std::next(mNodes.begin(), i));
    }

    bool IsNthChildOf(const NodeId parent, const NodeId nth_child) const
    {
        if(parent == nth_child)
            return true;
        if(nth_child < parent)
            return false; // id of child is always larger than parent, these 2 are not related
        return IsNthChildOf(parent, Get(nth_child).Parent());
    }

    TreeNodeType& Get(const NodeId id)
    {
        return IsContinuous() ? mNodes.at(id) : mNodes.at(GetIndexBinarySearch(id)); //
    }

    const TreeNodeType& Get(const NodeId id) const
    {
        return IsContinuous() ? mNodes.at(id) : mNodes.at(GetIndexBinarySearch(id)); //
    }

    TreeNodeType* TryGet(const NodeId id)
    {
        if(IsContinuous())
        {
            return id < mNodes.size() ? &mNodes.at(id) : nullptr;
        }
        else
        {
            auto idx = GetIndexBinarySearch(id);
            return idx < mNodes.size() ? &mNodes.at(idx) : nullptr;
        }
    }

    TreeNodeType& GetIndex(const std::size_t idx) { return mNodes.at(idx); }
    const TreeNodeType& GetIndex(const std::size_t idx) const { return mNodes.at(idx); }

    bool IsContinuous() const { return mNodes.size() == mIdCounter; }
    bool IsEmpty() const { return mNodes.empty(); }

    std::size_t Size() const { return mNodes.size(); }
    TreeNodeType& Root() { return mNodes.at(0); }

    void Relabel()
    {
        // Give all nodes in this tree new ids, making the tree continuous again.
        // this makes all old ids invalid.
        mIdCounter = 0;
        std::map<NodeId, NodeId> newIDs; // maps old to new id
        for(auto& node : mNodes)
        {
            newIDs[node.Id()] = mIdCounter;
            node.mId = mIdCounter;
            mIdCounter++;
        }
        for(auto& node : mNodes)
            node.mParentId = newIDs[node.mParentId];
    }

    std::vector<std::size_t> GetNodeDepths() const
    {
        std::vector<std::size_t> result(mNodes.size());
        for(std::size_t i = 0; i < mNodes.size(); ++i)
        {
            const auto& node = GetIndex(i);
            result.at(i) = result.at(GetIndexBinarySearch(node.parent)) + 1;
        }
        return result;
    }

    std::vector<std::size_t> GetBreathFirstIndexOrder() const
    {
        std::vector<std::size_t> result(mNodes.size());
        std::iota(result.begin(), result.end(), 0);

        // NOTE: two implementations:
        // One needs extra memory of O(n), but sort will still be O(n log(n))
        // The other needs no extra memory but uses recursive predicate, making the sort slower.

        //const auto depths = GetNodeDepths();
        //const auto pred = [&](const auto& a, const auto& b) {
        //    if(depths[a] == depths[b])
        //        return a < b; // keep the order within one depth stable
        //    return depths[a] < depths[b]; //
        //};
        //std::sort(result.begin(), result.end(), pred);

        using Index = std::size_t;

        // use a recursive predicate, might not be very efficient, but avoids extra memory
        const auto predicate = [&](const Index a, //
                                   const Index b) -> bool {
            const auto& nodeA = GetIndex(a);
            const auto& nodeB = GetIndex(b);
            if(nodeA.IsRoot())
                return true; // a < b
            if(nodeB.IsRoot())
                return false; // a > b

            const auto predicate_recursive = [&](const TreeNodeType& a, //
                                                 const TreeNodeType& b,
                                                 const std::size_t aDepth,
                                                 const std::size_t bDepth,
                                                 auto& self) -> bool {
                // end conditions
                if(a.Parent() == b.Parent())
                {
                    if(aDepth == bDepth)
                        return nodeA.Id() < nodeB.Id(); // note: captured from containing lambda
                    else
                        return aDepth < bDepth;
                }

                // else, recurse by popping the highest parent id
                if(a.Parent() < b.Parent())
                {
                    return self(a, Get(b.Parent()), aDepth, bDepth + 1, self);
                }
                else // b.Parent() < a.Parent()
                {
                    return self(Get(a.Parent()), b, aDepth + 1, bDepth, self);
                }
            };

            return predicate_recursive(nodeA, nodeB, 0, 0, predicate_recursive);
        };

        std::sort(result.begin(), result.end(), predicate);

        return result;
    }

    std::vector<std::size_t> GetDepthFirstIndexOrder() const
    {
        std::vector<std::size_t> result(mNodes.size());
        std::iota(result.begin(), result.end(), 0);

        using Index = std::size_t;

        // use a recursive predicate, might not be very efficient, but avoids extra memory
        const auto predicate = [&](const Index a, //
                                   const Index b) -> bool {
            const auto predicate_recursive = [&](const TreeNodeType& a, //
                                                 const TreeNodeType& b,
                                                 auto& self) -> bool {
                // end condition
                if(a.Parent() == b.Parent())
                    return a.Id < b.Id();

                // else, recurse by popping the highest parent id
                if(a.Parent() < b.Parent())
                {
                    return self(a, Get(b.Parent()), self);
                }
                else // b.Parent() < a.Parent()
                {
                    return self(Get(a.Parent()), b, self);
                }
            };

            return predicate_recursive(GetIndex(a), GetIndex(b), predicate_recursive);
        };

        std::sort(result.begin(), result.end(), predicate);
        return result;
    }

    auto begin() { return mNodes.begin(); }
    auto end() { return mNodes.end(); }

    auto begin() const { return mNodes.begin(); }
    auto end() const { return mNodes.end(); }

private:
    std::size_t GetIndexBinarySearch(NodeId id) const
    {
        const auto pred = [](const auto& item, const NodeId id) { return item.Id() < id; };
        const auto it = std::lower_bound(mNodes.begin(), mNodes.end(), id, pred);
        assert(it != mNodes.end());
        return it - mNodes.begin();
    }
    std::size_t GetIndexBinarySearch(NodeId id, //
                                     std::vector<TreeNodeType>::iterator begin,
                                     std::vector<TreeNodeType>::iterator end) const
    {
        // NOTE: returns index in the _subrange_ [begin; end>
        const auto pred = [](const auto& item, const NodeId id) { return item.Id() < id; };
        const auto it = std::lower_bound(begin, end, id, pred);
        assert(it != end);
        return it - begin;
    }

    std::vector<TreeNodeType> mNodes{};
    NodeId mIdCounter{0};
};

//
template <typename TTree>
class DepthFirstIterator
{
public:
    using NodeId = typename TTree::NodeId;
    using NodeType = typename TTree::TreeNodeType;

    struct Iterator
    {
        using iterator_category = std::random_access_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = NodeType;
        using pointer = NodeType*;
        using reference = NodeType&;

        Iterator() = default;
        Iterator(DepthFirstIterator<TTree>* iter, const std::size_t idx)
            : mIterator(iter)
            , mIdx(idx)
        { }

        reference operator*() const { return mIterator->mTree.Get(mIterator->mDFOrder.at(mIdx)); }
        pointer operator->() { return &mIterator->mTree.Get(mIterator->mDFOrder.at(mIdx)); }
        Iterator& operator++()
        {
            mIdx++;
            return *this;
        }
        Iterator operator++(int)
        {
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }
        friend bool operator==(const Iterator& a, const Iterator& b)
        {
            return a.mIterator && b.mIterator && (a.mIterator == b.mIterator) && a.mIdx == b.mIdx; //
        };
        friend bool operator!=(const Iterator& a, const Iterator& b) { return !operator==(a, b); };

    private:
        DepthFirstIterator<TTree>* mIterator{nullptr};
        std::size_t mIdx{0};
    };

    DepthFirstIterator(TTree& tree)
        : mTree(tree)
    {
        Update();
    }

    void Update()
    {
        mDFOrder.clear();
        if(mTree.IsEmpty())
            return;
        NodeId rootId = mTree.Root().Id();
        UpdateRecursive(rootId, std::next(mTree.begin()));
    }

    auto begin() { return Iterator{this, 0u}; }
    auto end() { return Iterator{this, mDFOrder.size()}; }

    auto begin() const { return Iterator(this, 0); }
    auto end() const { return Iterator(this, mDFOrder.size()); }

private:
    TTree& mTree;
    std::vector<NodeId> mDFOrder{};

    using TreeIterator = decltype(mTree.begin());
    void UpdateRecursive(const NodeId id, const TreeIterator itStart)
    {
        // children are always after their parent in the list, so pass itStart along
        mDFOrder.push_back(id);
        for(TreeIterator it = itStart; it < mTree.end(); ++it)
            if(it->Parent() == id)
                UpdateRecursive(it->Id(), std::next(it));
    }
    friend Iterator;
};

template <typename TTree>
class BreathFirstIterator
{
public:
    using NodeId = typename TTree::NodeId;

    BreathFirstIterator(TTree& tree)
        : mTree(tree)
    {
        Update();
    }

    void Update()
    {
        // Todo: construct the order vector
    }

    auto begin() { return mBFOrder.begin(); }
    auto end() { return mBFOrder.end(); }

private:
    TTree& mTree;
    std::vector<NodeId> mBFOrder{};
};

} // namespace memory
} // namespace vic