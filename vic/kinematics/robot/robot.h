#pragma once

#include "vic/kinematics/robot/node.h"

#include "vic/memory/tree.h"

#include <optional>

namespace vic
{
namespace kinematics
{
namespace robots
{

// robot that contains a forward defined chain of joints
template <typename T>
class ForwardRobot
{

private:
    vic::memory::Tree<Node<T>> mTree{};
    vic::memory::DepthFirstIterator<decltype(mTree)> mIterator{mTree};

public:
    ForwardRobot() = default;

    using NodeId = typename decltype(mTree)::NodeId;

    NodeId AddJoint(const std::optional<NodeId> parent, const Transformation<T>& transform, const Screw<T>& screw)
    {
        if(parent)
            return mTree.NewNode(Node{NodeType::Joint, transform, screw}, *parent).Id();
        else
            return mTree.NewRoot(Node{NodeType::Joint, transform, screw}).Id();
    }

    NodeId AddFrame(const std::optional<NodeId> parent, const Transformation<T>& transform)
    {
        // a frame is always directly connected to a joint,
        // it can be used to calculate the transformation at certain points of the robot
        return {};
    }
    NodeId AddRigidBody(const std::optional<NodeId> parent, const Transformation<T>& transform, const Inertia<T>& inertia)
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