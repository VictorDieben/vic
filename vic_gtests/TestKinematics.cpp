#include "pch.h"

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/rotation.h"

namespace vic
{
namespace kinematics
{
template <typename T, std::size_t rows, std::size_t cols>
constexpr bool IsEqual(const Matrix<T, rows, cols>& v1, //
                       const Matrix<T, rows, cols>& v2,
                       const T eps = 1e-10)
{
    for(std::size_t i = 0; i < rows; ++i)
        for(std::size_t j = 0; j < cols; ++j)
            if(std::fabs(v1.Get(i, j) - v2.Get(i, j)) > eps)
                return false;
    return true;
}

TEST(TestKinematics, rotate)
{
    Matrix<double, 3, 3> matrix{Identity<double, 3>{}};
    auto zRotation = Rotate(matrix, zAxis, pi / 2.);

    // rotating a vector around the z axis for pi/2
    auto tmp = Matmul(zRotation, Vector3<double>({1, 0, 0}));
    EXPECT_TRUE(IsEqual(tmp, Vector3<double>({0, 1, 0})));

    tmp = Matmul(zRotation, Vector3<double>({0, 1, 0}));
    EXPECT_TRUE(IsEqual(tmp, Vector3<double>({-1, 0, 0})));

    tmp = Matmul(zRotation, Vector3<double>({-1, 0, 0}));
    EXPECT_TRUE(IsEqual(tmp, Vector3<double>({0, -1, 0})));

    tmp = Matmul(zRotation, Vector3<double>({0, -1, 0}));
    EXPECT_TRUE(IsEqual(tmp, Vector3<double>({1, 0, 0})));
}

TEST(TestKinematics, rotation)
{
    Matrix3<double> I{Identity<double, 3>{}};
    Rotation r1{Rotate(I, zAxis, pi / 2.)};
    Rotation r1Inverse = r1.Inverse();
    Rotation result = r1 * r1Inverse;
    EXPECT_TRUE(IsEqual(result.ToMatrix(), I));
}

TEST(TestKinematics, transformations)
{
    //
}

} // namespace kinematics
} // namespace vic