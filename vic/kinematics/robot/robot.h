#pragma once

#include "vic/kinematics/robot/node.h"

#include "vic/memory/tree.h"

namespace vic
{
namespace kinematics
{
namespace robots
{

// robot that contains a forward defined chain of joints
class ForwardRobot
{
public:
    ForwardRobot() = default;

    std::size_t GetNrJoints() const { return mTree.Size(); }

    const auto& GetTree() const { return mTree; }

    auto begin() const { return mIterator.begin(); }
    auto end() const { return mIterator.end(); }

private:
    vic::memory::Tree<Node> mTree{};
    vic::memory::DepthFirstIterator<decltype(mTree)> mIterator{mTree};
};

} // namespace robots
} // namespace kinematics
} // namespace vic