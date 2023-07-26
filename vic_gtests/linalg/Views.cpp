
#include "../test_base.h"
#include "gtest/gtest.h"

namespace vic
{
namespace linalg
{
//
//TEST(Views, TestViewTranspose)
//{
//    constexpr Matrix<double, 3, 3> matrix({1, 2, 3, 4, 5, 6, 7, 8, 9});
//    constexpr Matrix<double, 3, 3> answer({1, 4, 7, 2, 5, 8, 3, 6, 9});
//
//    const auto transposeView = ViewTranspose(matrix);
//    ExpectMatrixEqual(transposeView, answer);
//
//    // test constructing a new matrix from a view
//    Matrix<double, 3, 3> matrixT{transposeView};
//    ExpectMatrixEqual(matrixT, answer);
//}
//
//TEST(Views, TestViewRowStack)
//{
//    constexpr Matrix<double, 3, 3> mat1({1, 2, 3, 4, 5, 6, 7, 8, 9});
//    constexpr Identity<double, 3> identity3{};
//
//    const auto rowStackView = ViewRowStack(mat1, identity3);
//    EXPECT_EQ(rowStackView.GetRows(), 6u);
//    EXPECT_EQ(rowStackView.GetColumns(), 3u);
//
//    // verify matrix part
//    EXPECT_DOUBLE_EQ(rowStackView.Get(0, 0), 1.);
//    EXPECT_DOUBLE_EQ(rowStackView.Get(2, 2), 9.);
//
//    // verify identity part
//    EXPECT_DOUBLE_EQ(rowStackView.Get(3, 0), 1.);
//    EXPECT_DOUBLE_EQ(rowStackView.Get(3, 2), 0.);
//    EXPECT_DOUBLE_EQ(rowStackView.Get(5, 0), 0.);
//    EXPECT_DOUBLE_EQ(rowStackView.Get(5, 2), 1.);
//}
//
//TEST(Views, TestViewColumnStack)
//{
//    constexpr Diagonal<double, 3, 3> mat1({1, 2, 3});
//    constexpr Diagonal<double, 3, 3> mat2({4, 5, 6});
//
//    const auto colStackView = ViewColumnStack(mat1, mat2);
//    EXPECT_EQ(colStackView.GetRows(), 3u);
//    EXPECT_EQ(colStackView.GetColumns(), 6u);
//
//    // mat1
//    EXPECT_DOUBLE_EQ(colStackView.Get(0, 0), 1.);
//    EXPECT_DOUBLE_EQ(colStackView.Get(0, 2), 0.);
//    EXPECT_DOUBLE_EQ(colStackView.Get(2, 0), 0.);
//    EXPECT_DOUBLE_EQ(colStackView.Get(2, 2), 3.);
//
//    // mat2
//    EXPECT_DOUBLE_EQ(colStackView.Get(0, 3), 4.);
//    EXPECT_DOUBLE_EQ(colStackView.Get(2, 3), 0.);
//    EXPECT_DOUBLE_EQ(colStackView.Get(0, 5), 0.);
//    EXPECT_DOUBLE_EQ(colStackView.Get(2, 5), 6.);
//}
//
//TEST(Views, TestViewRowColumnCombined)
//{
//    // test putting a column stack in a row stack
//    constexpr Identity<double, 2> identity2{};
//    const ViewColumnStack colStack(identity2, identity2);
//    const ViewRowStack rowStack(colStack, colStack);
//
//    EXPECT_EQ(rowStack.GetRows(), 4u);
//    EXPECT_EQ(rowStack.GetColumns(), 4u);
//
//    for(std::size_t i = 0; i < 4; ++i)
//        for(std::size_t j = 0; j < 4; ++j)
//            EXPECT_DOUBLE_EQ(rowStack.Get(i, j), (i + j) % 2 == 0 ? 1. : 0);
//
//    Matrix<double, 4, 4> full{rowStack};
//    ExpectMatrixEqual(full, rowStack);
//}
//
//TEST(Views, TestViewBlockDiagonal)
//{
//    constexpr Matrix<double, 2, 2> mat({1, 2, 3, 4});
//    const ViewBlockDiagonal blockDiag(mat, mat);
//
//    EXPECT_DOUBLE_EQ(blockDiag.Get(0, 0), 1.);
//    EXPECT_DOUBLE_EQ(blockDiag.Get(1, 1), 4.);
//    EXPECT_DOUBLE_EQ(blockDiag.Get(2, 2), 1.);
//    EXPECT_DOUBLE_EQ(blockDiag.Get(3, 3), 4.);
//
//    EXPECT_DOUBLE_EQ(blockDiag.Get(0, 3), 0.);
//    EXPECT_DOUBLE_EQ(blockDiag.Get(3, 0), 0.);
//
//    const Matrix<double, 4, 4> mat4(blockDiag);
//    ExpectMatrixEqual(mat4, blockDiag);
//
//    // test nesting
//    const ViewBlockDiagonal blockDiag8(blockDiag, blockDiag);
//    const Matrix<double, 8, 8> mat8(blockDiag8);
//    ExpectMatrixEqual(mat8, blockDiag8);
//
//    // test constructing some other combinations
//    ViewBlockDiagonal nestedBlockDiag( //
//        ViewBlockDiagonal(Identity<double, 1>{}, Identity<double, 2>{}), //
//        ViewBlockDiagonal(Identity<double, 3>{}, Identity<double, 4>{}));
//}
//
//TEST(Views, TestViewTriangle)
//{
//    // Test views for upper / lower parts of matrix.
//    // useful when making LU decompositions
//    constexpr Matrix<double, 2, 2> mat({1, 2, 3, 4});
//
//    // TODO: find out why the decltype is needed
//    ExpectMatrixEqual(ViewUpperTriangle<decltype(mat)>{mat}, //
//                      Matrix<double, 2, 2>{{1, 2, 0, 4}});
//
//    ExpectMatrixEqual(ViewStrictUpperTriangle<decltype(mat)>{mat}, //
//                      Matrix<double, 2, 2>{{0, 2, 0, 0}});
//
//    ExpectMatrixEqual(ViewLowerTriangle<decltype(mat)>{mat}, //
//                      Matrix<double, 2, 2>{{1, 0, 3, 4}});
//
//    ExpectMatrixEqual(ViewStrictLowerTriangle<decltype(mat)>{mat}, //
//                      Matrix<double, 2, 2>{{0, 0, 3, 0}});
//}

} // namespace linalg
} // namespace vic