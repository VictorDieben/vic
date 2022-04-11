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

private:
    vic::memory::Tree<Node> mTree{};
    vic::memory::DepthFirstIterator<decltype(mTree)> mIterator{mTree};

public:
    ForwardRobot() = default;

    using NodeId = typename decltype(mTree)::NodeId;

    void AddJoint(const std::optional<NodeId> parent, const Transformation& transform, const Screw& screw)
    {
        if(parent)
            mTree.NewNode(Node{NodeType::Joint, transform, screw}, *parent);
        else
            mTree.NewRoot(Node{NodeType::Joint, transform, screw});
    }

    void AddFrame(const std::optional<NodeId> parent, const Transformation& transform)
    {
        // a frame is always directly connected to a joint,
        // it can be used to calculate the transformation at certain points of the robot
    }
    void AddRigidBody(const std::optional<NodeId> parent, const Transformation& transform, const Inertia<DataType>& inertia)
    {
        // a rigid body is always connected to a joint
    }

    std::size_t GetNrJoints() const { return mTree.Size(); }

    const auto& GetTree() const { return mTree; }

    auto begin() { return mIterator.begin(); }
    auto end() { return mIterator.end(); }
};

} // namespace robots
} // namespace kinematics
} // namespace vic