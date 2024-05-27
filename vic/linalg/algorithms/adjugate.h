#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/index.h"
#include "vic/linalg/traits.h"

#include <cassert>

namespace vic
{
namespace linalg
{

template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
constexpr auto Cofactor2x2(const TMatrix& matrix)
{
    //
}

template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
constexpr auto Cofactor3x3(const TMatrix& matrix)
{
    //
}

template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
constexpr ConceptMatrix auto Adjugate2x2(const TMatrix& matrix)
{
    assert(matrix.GetRows() == 2 && matrix.GetColumns() == 2);
    const auto a = matrix.Get(0, 0);
    const auto b = matrix.Get(0, 1);
    const auto c = matrix.Get(1, 0);
    const auto d = matrix.Get(1, 1);

    using TRes = Matrix<typename TMat::DataType, Shape<2, 2>>;

    return TRes{d, -b, -c, a};
}

template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
constexpr ConceptMatrix auto Adjugate3x3(const TMatrix& matrix)
{
    assert(matrix.GetRows() == 3 && matrix.GetColumns() == 3);
    const auto a = matrix.Get(0, 0);
    const auto b = matrix.Get(0, 1);
    const auto c = matrix.Get(0, 2);

    const auto d = matrix.Get(1, 0);
    const auto e = matrix.Get(1, 1);
    const auto f = matrix.Get(1, 2);

    const auto g = matrix.Get(2, 0);
    const auto h = matrix.Get(2, 1);
    const auto i = matrix.Get(2, 2);

    using TRes = Matrix<typename TMat::DataType, Shape<3, 3>>;

    return TRes{}; // todo
}

template <typename TMat>
    requires ConceptMatrix<TMat>
constexpr auto Adjugate(const TMat& matrix)
{
    return Transpose(CofactorMatrix(matrix));
}

} // namespace linalg
} // namespace vic