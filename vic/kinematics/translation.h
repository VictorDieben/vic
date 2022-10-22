#pragma once

#include "vic/linalg/matrices.h"

namespace vic
{
namespace kinematics
{
template <typename T>
struct Translation
{
public:
    constexpr Translation() = default;
    explicit constexpr Translation(const Vector3<T> translation)
        : mMatrix(translation)
    { }

    const Vector3<T>& ToMatrix() const { return mMatrix; }

private:
    Vector3<T> mMatrix{};
};

} // namespace kinematics
} // namespace vic