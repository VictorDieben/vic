#pragma once

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
        const auto lambda = [](const auto& N1, const auto& N2) { return N1.Id() < N2.Id(); };
        insert_sorted(mNodes, newNode, lambda);
        mIdCounter++;
        return mNodes.back();
    }
    TreeNodeType& NewNode(const T& data, const NodeId parent)
    {
        assert(!mNodes.empty());
        TreeNodeType newNode{data, mIdCounter, parent};
        const auto lambda = [](const auto& n1, const auto& n2) { return n1.Id() < n2.Id(); };
        insert_sorted(mNodes, newNode, lambda);
        mIdCounter++;
        return mNodes.back();
    }

    bool Remove(const NodeId id)
    {
        return true; // todo
    }

    TreeNodeType* TryGet(const NodeId id)
    {
        if(IsContinuous())
        {
            return &mNodes.at(id);
        }
        else
        {
            // use binary search to find index

            return nullptr;
        }
    }

    bool IsContinuous() const { return mNodes.size() == mIdCounter; }

private:
    std::vector<TreeNodeType> mNodes{};
    NodeId mIdCounter{0};
};

//
template <typename TTree>
class DepthFirstIterator
{
public:
    using NodeId = typename TTree::NodeId;

    DepthFirstIterator(TTree& tree)
        : mTree(tree)
    {
        Update();
    }

    void Update()
    {
        // Todo: construct the order vector
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