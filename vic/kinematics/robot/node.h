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

// todo
template <typename T>
struct Inertia
{
    T mass{};
    // todo: MoI
};

class Node
{
public:
    Node() = default;
    Node(const NodeType type)
        : mType(type)
    { }
    Node(const NodeType type, const Transformation& transform)
        : mType(type)
        , mTransformation(transform)
    { }
    Node(const NodeType type, const Transformation& transform, const Screw& screw)
        : mType(type)
        , mTransformation(transform)
        , mScrew(screw)
    { }

    constexpr bool IsType(const NodeType type) const { return mType == type; }

    // todo
    Transformation GetTransformation() const { return mTransformation; }
    Screw GetScrew() const
    {
        assert(mType == NodeType::Joint);
        return mScrew;
    }
    Inertia<DataType> GetInertia() const
    {
        assert(mType == NodeType::Body);
        return {};
    }

private:
    NodeType mType{NodeType::Joint};
    Transformation mTransformation{};
    Screw mScrew{};
};
} // namespace robots
} // namespace kinematics
} // namespace vic