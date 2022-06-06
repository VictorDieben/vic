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

    NodeId AddJoint(const std::optional<NodeId> parent, const Transformation& transform, const Screw& screw)
    {
        if(parent)
            return mTree.NewNode(Node{NodeType::Joint, transform, screw}, *parent).Id();
        else
            return mTree.NewRoot(Node{NodeType::Joint, transform, screw}).Id();
    }

    NodeId AddFrame(const std::optional<NodeId> parent, const Transformation& transform)
    {
        // a frame is always directly connected to a joint,
        // it can be used to calculate the transformation at certain points of the robot
        return {};
    }
    NodeId AddRigidBody(const std::optional<NodeId> parent, const Transformation& transform, const Inertia<DataType>& inertia)
    {
        // a rigid body is always connected to a joint
        return {};
    }

    void Update() { mIterator.Update(); }

    std::size_t GetNrJoints() const { return mTree.Size(); }

    const auto& GetTree() const { return mTree; }

    auto begin() { return mIterator.begin(); }
    auto end() { return mIterator.end(); }
};

} // namespace robots
} // namespace kinematics
} // namespace vic