#pragma once

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/rotation.h"

#include "vic/linalg/tools.h"
#include "vic/linalg/transpose.h"
#include "vic/utils.h"

#include <algorithm>
#include <assert.h>

namespace vic
{
namespace kinematics
{

struct Translation
{
public:
    constexpr Translation() = default;
    constexpr Translation(const Vector3<DataType> translation)
        : mMatrix(translation)
    { }

    const Vector3<DataType>& ToMatrix() const { return mMatrix; }

private:
    Vector3<DataType> mMatrix{};
};

} // namespace kinematics
} // namespace vic