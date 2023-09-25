#include "gtest/gtest.h"

#include "../test_base.h"

#include "gtest/gtest.h"

#include "vic/linalg/linalg.h"

#include <random>

using namespace vic::linalg;

template <typename TMat>
void VerifyMatrix(TMat& mat)
{
    EXPECT_TRUE(ConceptMatrix<TMat>);
    RandomFill(mat);
    auto copy = mat;
    EXPECT_TRUE(IsEqual(mat, copy));
}

TEST(Linalg, InitBase)
{
    static constexpr MatrixBaseSelector<Shape<3, 3>> m33(3, 3);
    static constexpr MatrixBaseSelector<Shape<3, UnknownSize>> mr3(3, 3);
    static constexpr MatrixBaseSelector<Shape<UnknownSize, 3>> mc3(3, 3);
    static constexpr MatrixBaseSelector<Shape<UnknownSize, UnknownSize>> md(3, 3);
}

TEST(Linalg, InitZeros)
{
    static constexpr Zeros<double, Shape<3, 3>> m33(3, 3);
    static constexpr Zeros<double, Shape<3, UnknownSize>> mr3(3, 3);
    static constexpr Zeros<double, Shape<UnknownSize, 3>> mc3(3, 3);
    static constexpr Zeros<double, Shape<UnknownSize, UnknownSize>> md(3, 3);
    VerifyMatrix(m33);
    VerifyMatrix(mr3);
    VerifyMatrix(mc3);
    VerifyMatrix(md);
    EXPECT_TRUE(ConceptZeros<decltype(m33)>);
    EXPECT_TRUE(ConceptZeros<decltype(mr3)>);
    EXPECT_TRUE(ConceptZeros<decltype(mc3)>);
    EXPECT_TRUE(ConceptZeros<decltype(md)>);
}

TEST(Linalg, InitDiagonal)
{
    static constexpr Diagonal<double, Shape<3, 3>> m33{3, 3};
    Diagonal<double, Shape<3, UnknownSize>> mr3{3, 3};
    Diagonal<double, Shape<UnknownSize, 3>> mc3{3, 3};
    Diagonal<double, Shape<UnknownSize, UnknownSize>> md{3, 3};
    VerifyMatrix(m33);
    VerifyMatrix(mr3);
    VerifyMatrix(mc3);
    VerifyMatrix(md);
    EXPECT_TRUE(ConceptDiagonal<decltype(m33)>);
    EXPECT_TRUE(ConceptDiagonal<decltype(mr3)>);
    EXPECT_TRUE(ConceptDiagonal<decltype(mc3)>);
    EXPECT_TRUE(ConceptDiagonal<decltype(md)>);
}

TEST(Linalg, InitIdentity)
{
    static constexpr Identity<double, Shape<3, 3>> m33{3, 3};
    static constexpr Identity<double, Shape<3, UnknownSize>> mr3{3, 3};
    static constexpr Identity<double, Shape<UnknownSize, 3>> mc3{3, 3};
    static constexpr Identity<double, Shape<UnknownSize, UnknownSize>> md{3, 3};
    VerifyMatrix(m33);
    VerifyMatrix(mr3);
    VerifyMatrix(mc3);
    VerifyMatrix(md);
    EXPECT_TRUE(ConceptIdentity<decltype(m33)>);
    EXPECT_TRUE(ConceptIdentity<decltype(mr3)>);
    EXPECT_TRUE(ConceptIdentity<decltype(mc3)>);
    EXPECT_TRUE(ConceptIdentity<decltype(md)>);

    EXPECT_TRUE(ConceptDiagonal<decltype(m33)>);
    EXPECT_TRUE(ConceptDiagonal<decltype(mr3)>);
    EXPECT_TRUE(ConceptDiagonal<decltype(mc3)>);
    EXPECT_TRUE(ConceptDiagonal<decltype(md)>);
}

TEST(Linalg, InitMatrix)
{
    static constexpr Matrix<double, Shape<3, 3>> static33{3, 3};
    Matrix<double, Shape<3, 3>> m33{3, 3};
    Matrix<double, Shape<3, UnknownSize>> mr3{3, 3};
    Matrix<double, Shape<UnknownSize, 3>> mc3{3, 3};
    Matrix<double, Shape<UnknownSize, UnknownSize>> md{3, 3};
    VerifyMatrix(m33);
    VerifyMatrix(mr3);
    VerifyMatrix(mc3);
    VerifyMatrix(md);
}

TEST(Linalg, InitSparse)
{
    Sparse<double, Shape<3, 3>> m33{3, 3};
    Sparse<double, Shape<3, UnknownSize>> mr3{3, 3};
    Sparse<double, Shape<UnknownSize, 3>> mc3{3, 3};
    Sparse<double, Shape<UnknownSize, UnknownSize>> md{3, 3};
    VerifyMatrix(m33);
    VerifyMatrix(mr3);
    VerifyMatrix(mc3);
    VerifyMatrix(md);
}

TEST(Linalg, InitBracket)
{
    Vector3<double> vec{};
    Bracket3<double> bracket{vec};
    VerifyMatrix(bracket);
    EXPECT_TRUE(ConceptConstexprMatrix<decltype(bracket)>);
}

TEST(Linalg, InitRowStack)
{
    // stack two constexpr sized matrices, check that result is constexpr
    constexpr Matrix3<double> mat33;
    constexpr auto rowStack1 = ToRowStack(mat33, mat33);
    EXPECT_TRUE(ConceptConstexprMatrix<decltype(rowStack1)>);

    // stack one matrix with dynamic colsize and one with static colsize, check that result is constexpr
    constexpr Zeros<double, Shape<3, UnknownSize>> zeros3d{3, 3};
    const auto rowStack2 = ToRowStack(mat33, zeros3d);
    EXPECT_TRUE(ConceptConstexprMatrix<decltype(rowStack2)>);

    const auto rowstack = ToRowStack(Vector1<double>{{1.}}, Vector1<double>{{2.}});
    EXPECT_TRUE(IsEqual(rowstack, Vector2<double>{{1., 2.}}));
    EXPECT_DOUBLE_EQ(rowstack.Get(0, 0), 1.);
    EXPECT_DOUBLE_EQ(rowstack.Get(1, 0), 2.);
}

TEST(Linalg, InitColStack)
{
    // stack two constexpr sized matrices, check that result is constexpr
    constexpr Matrix3<double> mat33;
    constexpr auto colStack1 = ToColStack(mat33, mat33);
    EXPECT_TRUE(ConceptConstexprMatrix<decltype(colStack1)>);

    // stack one matrix with dynamic rowsize and one with static rowsize, check that result is constexpr
    constexpr Zeros<double, Shape<UnknownSize, 3>> zeros3d{3, 3};
    const auto colStack2 = ToColStack(mat33, zeros3d);
    EXPECT_TRUE(ConceptConstexprMatrix<decltype(colStack2)>);

    const auto dynamic = ToColStack(zeros3d, zeros3d);
    EXPECT_FALSE(ConceptConstexprMatrix<decltype(dynamic)>);

    const auto colstack = ToColStack(Matrix1<double>{{1.}}, Matrix1<double>{{2.}});
    EXPECT_TRUE(IsEqual(colstack, Matrix<double, Shape<1, 2>>{{1., 2.}}));
    EXPECT_DOUBLE_EQ(colstack.Get(0, 0), 1.);
    EXPECT_DOUBLE_EQ(colstack.Get(0, 1), 2.);
}

TEST(Linalg, InitTriDiagonal)
{
    constexpr Matrix3<double> mat33(1, 2, 0, 3, 4, 5, 0, 6, 7);

    constexpr auto tri3 = ToTriDiagonal(mat33);
    static_assert(ConceptMatrix<decltype(tri3)>);
    EXPECT_TRUE(IsEqual(mat33, tri3));

    const auto d33 = To<Matrix<double, UnknownShape>>(mat33);
    EXPECT_TRUE(IsEqual(mat33, d33));

    const auto trid3 = ToTriDiagonal(d33);
    static_assert(ConceptMatrix<decltype(trid3)>);
    EXPECT_TRUE(IsEqual(d33, trid3));
}

TEST(Linalg, MakeVector)
{
    constexpr Vector1<double> d1 = MakeVector(1.);
    constexpr Vector2<double> d2 = MakeVector(1., 2.);
    constexpr Vector3<double> d3 = MakeVector(1., 2., 3.);

    constexpr Vector1<float> f1 = MakeVector(1.f);
    constexpr Vector2<float> f2 = MakeVector(1.f, 2.f);
    constexpr Vector3<float> f3 = MakeVector(1.f, 2.f, 3.f);
}

TEST(Linalg, ConstMatrixConstructor)
{
    constexpr Vector1<double> d1(1.); //
}