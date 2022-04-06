#pragma once

#include "vic/kinematics/transformation.h"

namespace vic
{
namespace kinematics
{
namespace robots
{

enum class NodeType
{
    Joint,
    Frame,
    Body
};

class Node
{
public:
    Node() = default;
    Node(const NodeType type)
        : mType(type)
    { }

    constexpr bool IsType(const NodeType type) const { return mType == type; }

private:
    NodeType mType{NodeType::Joint};
    Screw mScrew{}; // todo: screw or transform?
};
} // namespace robots
} // namespace kinematics
} // namespace vic