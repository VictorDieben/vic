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

template <typename T>
class Node
{
public:
    Node() = default;
    Node(const NodeType type)
        : mType(type)
    { }
    Node(const NodeType type, const Transformation<T>& transform)
        : mType(type)
        , mTransformation(transform)
    { }
    Node(const NodeType type, const Transformation<T>& transform, const Screw<T>& screw)
        : mType(type)
        , mTransformation(transform)
        , mScrew(screw)
    { }

    constexpr bool IsType(const NodeType type) const { return mType == type; }

    // todo
    Transformation<T> GetTransformation() const { return mTransformation; }
    Screw<T> GetScrew() const
    {
        assert(mType == NodeType::Joint);
        return mScrew;
    }
    Inertia<T> GetInertia() const
    {
        assert(mType == NodeType::Body);
        return {};
    }

private:
    NodeType mType{NodeType::Joint};
    Transformation<T> mTransformation{};
    Screw<T> mScrew{};
};
} // namespace robots
} // namespace kinematics
} // namespace vic