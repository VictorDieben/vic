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

TEST(TestTree, Setup)
{

    Tree<TestTreeData> tree;

    auto& root = tree.NewRoot({1, 2., "3"});
    EXPECT_DEATH(tree.NewRoot({}), ""); // there can only be 1 root

    auto& node = tree.NewNode({4, 5., "6"}, root.Id());
}

TEST(TestTree, DepthFirst)
{
    // Test the depth first iterator

    Tree<TestTreeData> tree;
    // todo: add some nodes in breath first order

    DepthFirstIterator iterator(tree);

    for(const auto& node : iterator)
    {
        // todo: verify order
    }
}

TEST(TestTree, BreathFirst)
{
    Tree<TestTreeData> tree;
    // todo: add some nodes in depth first order

    BreathFirstIterator iterator(tree);

    for(const auto& node : iterator)
    {
        // todo: verify order
    }
}
TEST(TestTree, Randomized) { }

//
} // namespace memory
} // namespace vic