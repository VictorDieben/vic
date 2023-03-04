#include "../pch.h"
//#include "../test_base.h"
//
//#include "vic/linalg/algorithms/eigenvalues.h"
//#include "vic/linalg/inverse.h"
//#include "vic/linalg/matrices.h"
//#include "vic/linalg/matrices_dynamic.h"
//#include "vic/linalg/traits.h"
//#include "vic/linalg/transpose.h"
//
//#include "vic/linalg/tools.h"
//#include "vic/utils.h"
//#include <random>
//
//namespace vic
//{
//namespace linalg
//{
//namespace algorithms
//{
//
//TEST(TestLinalg, PowerMethod)
//{
//    // power method finds the largest eigenvector.
//    // and eigenvector is the vector for which holds:
//    // A x = lambda x
//    // with x the eigen vector and lambda the eigenvalue
//    const Matrix<double, 10, 10> matrix{Diagonal<double, 10, 10>{{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}}};
//    const auto res = PowerMethod(matrix, 1E-14);
//
//    const auto Ax = Matmul(matrix, res);
//    const auto vec1 = Normalize(Ax);
//    const auto vec2 = Normalize(res);
//    ASSERT_TRUE(IsEqual(vec1, vec2, 1E-7)); // check that normalized(Ax) == normalized(lambda*x)
//    ASSERT_NEAR(Norm(Ax), 10, 1E-10); // check that eigenvalue corresponding to res is 10
//}
//
//TEST(TestLinalg, PowerMethod2d)
//{
//
//    constexpr Matrix<double, 2, 2> constexpPowerMethodMatrix{{1, 0, 0, 2}};
//    const auto vec2x2 = PowerMethod(constexpPowerMethodMatrix);
//    (void)vec2x2;
//
//    auto lambda = [&](double a, double b, double c, double d) {
//        const auto T = (a + d); // trace
//        const auto D = (a * d) - (b * c); // determinant
//
//        const auto l1 = (T / 2.) + vic::Sqrt((T * T / 4.) - D); // eigenvalue 1
//        const auto l2 = (T / 2.) - vic::Sqrt((T * T / 4.) - D); // eigenvalue 2
//
//        // todo
//
//        const Matrix<double, 2, 2> mat{{a, b, c, d}};
//    };
//
//    // https://people.math.harvard.edu/~knill/teaching/math21b2004/exhibits/2dmatrices/index.html
//    constexpr double a = 1, b = 2, c = 3, d = 4;
//
//    lambda(1, 2, 3, 4);
//    lambda(1, 0, 0, 2);
//}
//
//TEST(TestLinalg, QRMethod)
//{
//    std::default_random_engine g;
//    std::uniform_real_distribution<double> dist(0.01, 100.);
//
//    constexpr std::size_t n = 25;
//
//    // test a bunch of random matrices
//    for(std::size_t i = 0; i < 1; ++i)
//    {
//        Matrix<double, n, n> matrix{};
//        for(Row r = 0; r < n; ++r)
//            for(Col c = 0; c < n; ++c)
//                matrix.At(r, c) = dist(g);
//
//        const auto [EigenVectors, EigenValues] = QRMethod(matrix);
//
//        (void)EigenVectors;
//        (void)EigenValues;
//    }
//}
//
//} // namespace algorithms
//} // namespace linalg
//} // namespace vic