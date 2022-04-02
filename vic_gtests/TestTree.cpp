#include "pch.h"

#include "vic/memory/tree.h"

namespace vic
{
namespace memory
{
struct TestTreeData
{
    int a;
    float b;
    std::string c;
};

struct TestSmallNode
{
    int i;
};

TEST(TestTree, Setup)
{
    // check adding nodes, removing them, relabeling tree, etc.
    Tree<TestTreeData> tree;

    auto& root = tree.NewRoot({1, 2., "3"});
    EXPECT_EQ(root.Id(), 0);
    EXPECT_EQ(root.Parent(), 0);
    EXPECT_TRUE(root.IsRoot());

    EXPECT_DEATH(tree.NewRoot({}), ""); // there can only be 1 root

    auto& root2 = tree.Get(0u);
    EXPECT_EQ(&root, &root2);

    auto& node1 = tree.NewNode({4, 5., "6"}, 0);
    auto& node1_2 = tree.Get(node1.Id());
    EXPECT_EQ(&node1, &node1_2);

    auto& node2 = tree.NewNode({7, 8., "9"}, 0);
    auto& node2_2 = tree.Get(node2.Id());
    EXPECT_EQ(&node2, &node2_2);

    EXPECT_EQ(tree.Size(), 3);
    EXPECT_TRUE(tree.IsContinuous());

    // try to remove a node that does not exist
    tree.Remove(4);

    EXPECT_EQ(tree.Size(), 3);
    EXPECT_TRUE(tree.IsContinuous());

    // remove a node
    tree.Remove(1);
    EXPECT_EQ(tree.Size(), 2);
    EXPECT_FALSE(tree.IsContinuous());

    // try to get node 2 (tree is no longer continuous)
    auto& node2_3 = tree.Get(2);
    EXPECT_EQ(node2_3.Id(), 2);
}

TEST(TestTree, DepthFirst)
{
    // Make tree, add nodes in breath first order
    Tree<TestSmallNode> tree;
    tree.NewRoot({0}); // 0
    tree.NewNode({1}, 0u); //   1
    tree.NewNode({2}, 0u); //   2
    tree.NewNode({3}, 1u); //     3
    tree.NewNode({4}, 1u); //     4
    tree.NewNode({5}, 2u); //     5
    tree.NewNode({6}, 2u); //     6

    DepthFirstIterator iterator(tree);

    std::vector<decltype(tree)::NodeId> result;
    for(const auto& node : iterator)
        result.push_back(node.Id());

    const std::vector<decltype(tree)::NodeId> answer = {0, 1, 3, 4, 2, 5, 6};
    EXPECT_EQ(result, answer);
}

TEST(TestTree, BreathFirst)
{
    // create tree, add nodes in depth first order
    Tree<TestSmallNode> tree;
    tree.NewRoot({0}); // 0
    tree.NewNode({1}, 0u); //   1
    tree.NewNode({2}, 1u); //     2
    tree.NewNode({3}, 1u); //     3
    tree.NewNode({4}, 0u); //   4
    tree.NewNode({5}, 4u); //     5
    tree.NewNode({6}, 4u); //     6

    BreathFirstIterator iterator(tree);

    std::vector<decltype(tree)::NodeId> result;
    for(const auto& node : iterator)
        result.push_back(node);

    const std::vector<decltype(tree)::NodeId> answer = {0, 1, 4, 2, 3, 5, 6};
    EXPECT_EQ(result, answer);
}

TEST(TestTree, Randomized) { }

//
} // namespace memory
} // namespace vic