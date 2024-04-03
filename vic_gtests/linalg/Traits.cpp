#include "../test_base.h"
#include "gtest/gtest.h"

#include "vic/linalg/linalg.h"

using namespace vic::linalg;

TEST(Linalg, ConceptRows)
{
    struct s1
    {
        constexpr static Row GetRows() { return 1; }
    };

    struct s2
    {
        static Row GetRows() { return 1; }
    };

    struct s3
    {
        Row GetRows() const { return 1; }
    };

    EXPECT_TRUE(ConceptConstexprRows<s1>);
    EXPECT_FALSE(ConceptConstexprRows<s2>);
    EXPECT_FALSE(ConceptConstexprRows<s3>);
}

TEST(Linalg, Concepts)
{
    //using mat = Matrix<float, 1, 1>;
    //using identity = Identity<float, 1>;
    //using diag = Diagonal<float, 1, 1>;
    //using zeros = Zeros<float, 1, 1>;

    //// base types
    //EXPECT_TRUE((ConceptMatrix<mat>));
    //EXPECT_TRUE((ConceptMatrix<identity>));
    //EXPECT_TRUE((ConceptMatrix<diag>));
    //EXPECT_TRUE((ConceptMatrix<zeros>));

    //// view transpose
    //EXPECT_TRUE((ConceptMatrix<ViewTranspose<mat>>));
    //EXPECT_TRUE((ConceptMatrix<ViewTranspose<identity>>));
    //EXPECT_TRUE((ConceptMatrix<ViewTranspose<diag>>));
    //EXPECT_TRUE((ConceptMatrix<ViewTranspose<zeros>>));

    //// view rowstack
    //EXPECT_TRUE((ConceptMatrix<ViewRowStack<mat, identity>>));
    //EXPECT_TRUE((ConceptMatrix<ViewRowStack<mat, diag>>));
    //EXPECT_TRUE((ConceptMatrix<ViewRowStack<mat, zeros>>));
    //EXPECT_TRUE((ConceptMatrix<ViewRowStack<identity, diag>>));
    //EXPECT_TRUE((ConceptMatrix<ViewRowStack<identity, zeros>>));
    //EXPECT_TRUE((ConceptMatrix<ViewRowStack<diag, zeros>>));

    //// view block diagonal
    //EXPECT_TRUE((ConceptMatrix<ViewBlockDiagonal<mat, identity>>));
    //EXPECT_TRUE((ConceptMatrix<ViewBlockDiagonal<mat, diag>>));
    //EXPECT_TRUE((ConceptMatrix<ViewBlockDiagonal<mat, zeros>>));
    //EXPECT_TRUE((ConceptMatrix<ViewBlockDiagonal<identity, zeros>>));
    //EXPECT_TRUE((ConceptMatrix<ViewBlockDiagonal<diag, zeros>>));

    //// view submatrix
    //EXPECT_TRUE((ConceptMatrix<ViewSubMatrix<mat, 0, 1, 0, 1>>));
    //EXPECT_TRUE((ConceptMatrix<ViewSubMatrix<identity, 0, 1, 0, 1>>));
    //EXPECT_TRUE((ConceptMatrix<ViewSubMatrix<diag, 0, 1, 0, 1>>));
    //EXPECT_TRUE((ConceptMatrix<ViewSubMatrix<zeros, 0, 1, 0, 1>>));

    //EXPECT_FALSE(ConceptMatrix<float>);
    //EXPECT_FALSE(ConceptMatrix<double>);
    //EXPECT_FALSE(ConceptMatrix<int>);
    //EXPECT_FALSE((ConceptMatrix<std::pair<int, float>>));

    //// square matrices
    //EXPECT_TRUE((ConceptSquareMatrix<Matrix<float, 1, 1>>));
    //EXPECT_TRUE((ConceptSquareMatrix<Matrix<float, 2, 2>>));
    //EXPECT_FALSE((ConceptSquareMatrix<Matrix<float, 1, 2>>));
    //EXPECT_FALSE((ConceptSquareMatrix<Matrix<float, 2, 1>>));

    //// vector
    //EXPECT_TRUE((ConceptVector<Matrix<float, 1, 1>>));
    //EXPECT_TRUE((ConceptVector<Matrix<float, 2, 1>>));
    //EXPECT_TRUE((ConceptVector<Matrix<float, 3, 1>>));
    //EXPECT_TRUE((ConceptVector<Matrix<float, 10, 1>>));
    //EXPECT_FALSE((ConceptVector<Matrix<float, 1, 2>>));
    //EXPECT_FALSE((ConceptVector<Matrix<float, 12, 124>>));
}
