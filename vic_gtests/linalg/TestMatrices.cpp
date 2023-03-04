#include "../test_base.h"
#include "pch.h"

#include "vic/linalg/linalg.h"
#include "vic/linalg/tools.h"

#include <random>

namespace vic
{
namespace linalg
{

template <typename TMat>
void RandomFill(TMat& mat)
{
    if constexpr(ConceptAssignable<TMat>)
    {
        std::default_random_engine g;
        std::uniform_real_distribution<double> randomValue(-1., 1.);

        if constexpr(TMat::Distribution == EDistribution::Diagonal)
        {
            for(MatrixSize i = 0; i < Min(mat.GetRows(), mat.GetColumns()); ++i)
                mat.At(i, i) = randomValue(g);
        }
        else if constexpr(TMat::Distribution == EDistribution::Full)
        {
            for(Row i = 0; i < mat.GetRows(); ++i)
                for(Col j = 0; j < mat.GetColumns(); ++j)
                    mat.At(i, j) = randomValue(g);
        }
    }
}

template <typename TMat>
void VerifyMatrix(TMat& mat)
{
    EXPECT_TRUE(ConceptMatrix<TMat>);
    RandomFill(mat);
    auto copy = mat;
    EXPECT_TRUE(IsEqual(mat, copy));
}

TEST(Matrices, InitBase)
{
    static constexpr MatrixBaseSelector<double, Shape<3, 3>> m33{3, 3};
    static constexpr MatrixBaseSelector<double, Shape<3, UnknownSize>> mr3{3, 3};
    static constexpr MatrixBaseSelector<double, Shape<UnknownSize, 3>> mc3{3, 3};
    static constexpr MatrixBaseSelector<double, Shape<UnknownSize, UnknownSize>> md{3, 3};
}

TEST(Matrices, InitZeros)
{
    static constexpr Zeros<double, Shape<3, 3>> m33{3, 3};
    static constexpr Zeros<double, Shape<3, UnknownSize>> mr3{3, 3};
    static constexpr Zeros<double, Shape<UnknownSize, 3>> mc3{3, 3};
    static constexpr Zeros<double, Shape<UnknownSize, UnknownSize>> md{3, 3};
    VerifyMatrix(m33);
    VerifyMatrix(mr3);
    VerifyMatrix(mc3);
    VerifyMatrix(md);
    EXPECT_TRUE(ConceptZeros<decltype(m33)>);
    EXPECT_TRUE(ConceptZeros<decltype(mr3)>);
    EXPECT_TRUE(ConceptZeros<decltype(mc3)>);
    EXPECT_TRUE(ConceptZeros<decltype(md)>);
}

TEST(Matrices, InitDiagonal)
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

TEST(Matrices, InitIdentity)
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

TEST(Matrices, InitMatrix)
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

TEST(Matrices, InitSparse)
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

} // namespace linalg
} // namespace vic