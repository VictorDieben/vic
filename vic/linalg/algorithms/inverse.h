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
using InverseResultShape = Shape<Min(TShape::rows, TShape::cols), Min(TShape::rows, TShape::cols)>;

template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto InverseDiagonal(const TMat& matrix)
{
    assert(matrix.GetRows() == matrix.GetColumns());
    Matrix<typename TMat::DataType, InverseResultShape<typename TMat::ShapeType>> inverse{matrix.GetRows(), matrix.GetColumns()};
    const auto n = Min(matrix.GetRows(), matrix.GetColumns());
    for(MatrixSize i = 0; i < n; ++i)
        inverse.At(i, i) = 1. / matrix.Get(i, i);
    return inverse;
}

template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto InverseHotellingBodewig(const TMat& matrix)
{
    assert(matrix.GetRows() == matrix.GetColumns());
    Matrix<typename TMat::DataType, InverseResultShape<typename TMat::ShapeType>> inverse{matrix.GetRows(), matrix.GetColumns()};

    // todo

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