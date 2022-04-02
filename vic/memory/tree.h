#pragma once

#include <algorithm>

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
        assert(!mNodes.empty());
        TreeNodeType newNode{data, mIdCounter, parent};
        mNodes.push_back(newNode);
        mIdCounter++;
        return mNodes.back();
    }

    bool Remove(const NodeId id)
    {
        // TODO(vicdie): also remove all children of this node
        const auto it = std::find_if(mNodes.begin(),
                                     mNodes.end(), //
                                     [&](const auto& item) { return item.Id() == id; });

        if(it != mNodes.end())
        {
            mNodes.erase(it);
            return true;
        }
        return false;
    }

    TreeNodeType& Get(NodeId id)
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
    std::size_t Size() const { return mNodes.size(); }

    void Relabel()
    {
        // give all nodes in this tree new ids,
        // making sure that the order is still the same,
        // but removing any gaps in ids.
        // also reset the IdCounter
    }

private:
    std::size_t GetIndexBinarySearch(NodeId id)
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

    struct Iterator
    { };

    DepthFirstIterator(TTree& tree)
        : mTree(tree)
    {
        Update();
    }

    void Update()
    {
        // Todo: build the order vector
    }

    auto begin() { return mDFOrder.begin(); }
    auto end() { return mDFOrder.end(); }

private:
    TTree& mTree;

    std::vector<NodeId> mDFOrder{};
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