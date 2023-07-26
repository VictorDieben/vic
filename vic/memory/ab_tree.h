#pragma once

#include <array>
#include <vector>

namespace vic
{
namespace memory
{

template <typename T, int a, int b>
class ABTree
{
public:
    static constexpr int A = a;
    static constexpr int B = b;

    using NodeType = std::array<T, b>;

    ABTree() = default;

    bool Insert(T& item) { }
    bool Remove(T& item) { }

private:
    std::vector<NodeType> mNodes;
};

} // namespace memory
} // namespace vic