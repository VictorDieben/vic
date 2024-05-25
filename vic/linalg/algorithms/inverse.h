#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/index.h"
#include "vic/linalg/traits.h"

#include <cassert>

namespace vic
{
namespace linalg
{

template <typename TShape>
using InverseResultShape = SquareShape<TShape>;
//using InverseResultShape = Shape<Min(TShape::rows, TShape::cols), Min(TShape::rows, TShape::cols)>;

template <typename TMat>
    requires ConceptMatrix<TMat>
constexpr auto InverseDiagonal(const TMat& matrix)
{
    assert(matrix.GetRows() == matrix.GetColumns());

    using ResultType = typename TMat::DataType;
    using ResultShape = InverseResultShape<typename TMat::ShapeType>;
    Diagonal<typename TMat::DataType, ResultShape> inverse{matrix.GetRows(), matrix.GetColumns()};
    const auto n = Min(matrix.GetRows(), matrix.GetColumns());
    for(MatrixSize i = 0; i < n; ++i)
        inverse.At(i, i) = 1. / matrix.Get(i, i);
    return inverse;
}

template <typename TMat>
    requires ConceptMatrix<TMat>
constexpr auto InverseHotellingBodewig(const TMat& A, const double eps = 1E-12)
{
    // Hotelling-Bodewig algorithm: V_n+1 = V_n * (2*I - A*V_n)
    assert(A.GetRows() == A.GetColumns());
    constexpr MatrixSize n = A.GetRows();

    using ResultType = typename TMat::DataType;
    using ResultShape = InverseResultShape<typename TMat::ShapeType>;

    const auto identity = Identity<ResultType, ResultShape>{A.GetRows(), A.GetColumns()};
    const auto twoI = Matmul(2., identity);

    auto inverse = ToFull(InverseDiagonal(ToDiagonal(A))); // initial guess

    uint32_t iter = 0;
    double largestDiff = 0.;
    for(; iter < 1000; ++iter)
    {
        const auto tmp1 = Matmul(A, inverse);
        const auto tmp2 = Matmul(-1., tmp1);
        inverse = Matmul(inverse, Add(twoI, tmp2));

        // todo: helper for largest (abs) element in matrix
        const auto diffMatrix = Subtract(tmp1, identity);
        largestDiff = 0.;
        for(Row i = 0; i < n; ++i)
            for(Col j = 0; j < n; ++j)
                largestDiff = Max(largestDiff, Abs(diffMatrix.Get(i, j)));
        if(largestDiff < eps)
            break;
    }
    // std::cout << "InverseHotellingBodewig(): iter = " << iter << "; diff = " << largestDiff << "; Done!" << std::endl;

    return inverse;
}

template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
constexpr auto Adjugate2x2(const TMatrix& matrix)
{
    assert(matrix.GetRows() == 2 && matrix.GetColumns() == 2);
    const auto a = matrix.Get(0, 0);
    const auto b = matrix.Get(0, 1);
    const auto c = matrix.Get(1, 0);
    const auto d = matrix.Get(1, 1);

    using ResultType = typename TMatrix::DataType;
    using ResultShape = Shape<2, 2>;
    using TRes = Matrix<typename TMatrix::DataType, ResultShape>;

    return TRes{d, -b, -c, a};
}

//template <typename TMatrix>
//    requires ConceptMatrix<TMatrix>
//constexpr auto Adjugate3x3(const TMatrix& matrix)
//{
//    assert(matrix.GetRows() == 3 && matrix.GetColumns() == 3);
//    const auto a = matrix.Get(0, 0);
//    const auto b = matrix.Get(0, 1);
//    const auto c = matrix.Get(0, 2);
//
//    const auto d = matrix.Get(1, 0);
//    const auto e = matrix.Get(1, 1);
//    const auto f = matrix.Get(1, 2);
//
//    const auto g = matrix.Get(2, 0);
//    const auto h = matrix.Get(2, 1);
//    const auto i = matrix.Get(2, 2);
//
//    // todo
//}

template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
constexpr auto Inverse2x2(const TMatrix& matrix)
{
    using DataType = typename TMatrix::DataType;

    const auto a = matrix.Get(0, 0);
    const auto b = matrix.Get(0, 1);
    const auto c = matrix.Get(1, 0);
    const auto d = matrix.Get(1, 1);

    const auto det = (a * d) - (b * c);
    if(det == 0)
        return Matrix2<DataType>{};
    const auto invDet = (DataType)1. / det;

    return Matrix2<DataType>{
        invDet * d, //
        -invDet * b,
        -invDet * c,
        invDet * a,
    };
}

template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
constexpr auto Inverse3x3(const TMatrix& matrix)
{
    // based on:
    // https://en.wikipedia.org/wiki/Invertible_matrix

    // note: explicit algorithm, should be easy to optimize
    // calculates (1/det(A)) * adj(A)

    // with A:
    // a, b, c
    // d, e, f
    // g, h, i

    using DataType = typename TMatrix::DataType;

    const auto a = matrix.Get(0, 0);
    const auto b = matrix.Get(0, 1);
    const auto c = matrix.Get(0, 2);

    const auto d = matrix.Get(1, 0);
    const auto e = matrix.Get(1, 1);
    const auto f = matrix.Get(1, 2);

    const auto g = matrix.Get(2, 0);
    const auto h = matrix.Get(2, 1);
    const auto i = matrix.Get(2, 2);

    const auto ei_fh = (e * i) - (f * h);
    const auto bi_ch = (b * i) - (c * h);
    const auto bf_ce = (b * f) - (c * e);

    const auto di_fg = (d * i) - (f * g);
    const auto ai_cg = (a * i) - (c * g);
    const auto af_cd = (a * f) - (c * d);

    const auto dh_eg = (d * h) - (e * g);
    const auto ah_bg = (a * h) - (b * g);
    const auto ae_bd = (a * e) - (b * d);

    const DataType det = (a * ei_fh) - (b * di_fg) + (c * dh_eg);
    if(det == 0)
        return Matrix3<DataType>{}; // no errors, keep it constexpr

    const DataType invDet = (DataType)1. / det;

    return Matrix3<DataType>{invDet * ei_fh, //
                             -invDet * bi_ch,
                             invDet * bf_ce,
                             -invDet * di_fg,
                             invDet * ai_cg,
                             -invDet * af_cd,
                             invDet * dh_eg,
                             -invDet * ah_bg,
                             invDet * ae_bd};
}

template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
constexpr auto Inverse4x4(const TMatrix& matrix)
{
    // todo: this will break with a 4x4 identity matrix
    // const auto [a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p] = *matrix.data();

    const auto a = matrix.Get(0, 0);
    const auto b = matrix.Get(0, 1);
    const auto c = matrix.Get(0, 2);
    const auto d = matrix.Get(0, 3);

    const auto e = matrix.Get(1, 0);
    const auto f = matrix.Get(1, 1);
    const auto g = matrix.Get(1, 2);
    const auto h = matrix.Get(1, 3);

    const auto i = matrix.Get(2, 0);
    const auto j = matrix.Get(2, 1);
    const auto k = matrix.Get(2, 2);
    const auto l = matrix.Get(2, 3);

    const auto m = matrix.Get(3, 0);
    const auto n = matrix.Get(3, 1);
    const auto o = matrix.Get(3, 2);
    const auto p = matrix.Get(3, 3);

    // a, b, c, d,
    // e, f, g, h,
    // i, j, k, l,
    // m, n, o, p

    const auto s0 = a * f - e * b;
    const auto s1 = a * g - e * c;
    const auto s2 = a * h - e * d;
    const auto s3 = b * g - f * c;
    const auto s4 = b * h - f * d;
    const auto s5 = c * h - g * d;

    const auto c5 = k * p - o * l;
    const auto c4 = j * p - n * l;
    const auto c3 = j * o - n * k;
    const auto c2 = i * p - m * l;
    const auto c1 = i * o - m * k;
    const auto c0 = i * n - m * j;

    // Should check for 0 determinant
    const auto det = (s0 * c5) - (s1 * c4) + (s2 * c3) + (s3 * c2) - (s4 * c1) + (s5 * c0);
    if(det == 0)
        return Matrix4<typename TMatrix::DataType>{};
    const auto invdet = 1.0 / det;

    const auto b00 = (f * c5 - g * c4 + h * c3) * invdet;
    const auto b01 = (-b * c5 + c * c4 - d * c3) * invdet;
    const auto b02 = (n * s5 - o * s4 + p * s3) * invdet;
    const auto b03 = (-j * s5 + k * s4 - l * s3) * invdet;

    const auto b10 = (-e * c5 + g * c2 - h * c1) * invdet;
    const auto b11 = (a * c5 - c * c2 + d * c1) * invdet;
    const auto b12 = (-m * s5 + o * s2 - p * s1) * invdet;
    const auto b13 = (i * s5 - k * s2 + l * s1) * invdet;

    const auto b20 = (e * c4 - f * c2 + h * c0) * invdet;
    const auto b21 = (-a * c4 + b * c2 - d * c0) * invdet;
    const auto b22 = (m * s4 - n * s2 + p * s0) * invdet;
    const auto b23 = (-i * s4 + j * s2 - l * s0) * invdet;

    const auto b30 = (-e * c3 + f * c1 - g * c0) * invdet;
    const auto b31 = (a * c3 - b * c1 + c * c0) * invdet;
    const auto b32 = (-m * s3 + n * s1 - o * s0) * invdet;
    const auto b33 = (i * s3 - j * s1 + k * s0) * invdet;

    return Matrix4<typename TMatrix::DataType>{b00, b01, b02, b03, b10, b11, b12, b13, b20, b21, b22, b23, b30, b31, b32, b33};
}

// Selector for Inverse algorithm
template <typename TMat>
    requires ConceptMatrix<TMat>
constexpr auto Inverse(const TMat& matrix)
{
    // TODO: if is:
    // - rotation
    // - transformation
    // - banded
    // - block

    // TODO: check if user has made a specialization for this specific type of matrix

    // TODO: split Inverse() up into 2 parts:
    // - preconditioner, initial guess, partly dependant on solver
    // - solving algorithm, either iterative/LU-decomposition/something else

    if constexpr(ConceptIdentity<TMat>)
        return matrix;
    else if constexpr(ConceptDiagonal<TMat>)
        return InverseDiagonal(matrix);
    else
        return InverseHotellingBodewig(matrix);
}

} // namespace linalg
} // namespace vic