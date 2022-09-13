#pragma once

#include "vic/linalg/matrices.h"

namespace vic
{
namespace kinematics
{

struct Translation
{
public:
    constexpr Translation() = default;
    explicit constexpr Translation(const Vector3<DataType> translation)
        : mMatrix(translation)
    { }

    const Vector3<DataType>& ToMatrix() const { return mMatrix; }

private:
    Vector3<DataType> mMatrix{};
};

} // namespace kinematics
} // namespace vic