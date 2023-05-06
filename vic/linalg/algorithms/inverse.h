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