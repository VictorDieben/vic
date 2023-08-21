#pragma once

#include <algorithm>
#include <map>
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
            if(IsRelated(id, mNodes.at(i).Id()))
                mNodes.erase(std::next(mNodes.begin(), i));
    }

    bool IsRelated(const NodeId parent, const NodeId nth_child) const
    {
        if(parent == nth_child)
            return true;
        if(nth_child < parent)
            return false; // id of child is always larger than parent, these 2 are not related
        return IsRelated(parent, Get(nth_child).Parent());
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

    auto begin() { return mNodes.begin(); }
    auto end() { return mNodes.end(); }

private:
    std::size_t GetIndexBinarySearch(NodeId id) const
    {
        const auto pred = [](const auto& item, const NodeId id) { return item.Id() < id; };
        const auto it = std::lower_bound(mNodes.begin(), mNodes.end(), id, pred);
        return it - mNodes.begin();
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

    //auto begin() const { return Iterator(this, 0); }
    //auto end() const { return Iterator(this, mDFOrder.size()); }

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