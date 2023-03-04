#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/traits.h"

namespace vic
{
namespace linalg
{

template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto DeterminantDiagonal(const TMat& matrix)
{
    typename TMat::DataType val{1.};
    for(std::size_t i = 0; i < TMat::GetRows(); ++i)
        val = val * matrix.Get(i, i);
    return val;
}

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto Determinant2x2(const TMatrix& matrix)
{
    assert(matrix.GetRows() == 2 && matrix.GetColumns() == 2);
    using TRet = typename TMatrix::DataType;
    const TRet a = matrix.Get(0, 0);
    const TRet b = matrix.Get(0, 1);
    const TRet c = matrix.Get(1, 0);
    const TRet d = matrix.Get(1, 1);
    return (a * d) - (b * c);
}

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto Determinant3x3(const TMatrix& matrix)
{
    assert(matrix.GetRows() == 3 && matrix.GetColumns() == 3);
    using TRet = typename TMatrix::DataType;
    const TRet a = matrix.Get(0, 0);
    const TRet b = matrix.Get(0, 1);
    const TRet c = matrix.Get(0, 2);
    const TRet d = matrix.Get(1, 0);
    const TRet e = matrix.Get(1, 1);
    const TRet f = matrix.Get(1, 2);
    const TRet g = matrix.Get(2, 0);
    const TRet h = matrix.Get(2, 1);
    const TRet i = matrix.Get(2, 2);
    return (a * e * i) + (b * f * g) + (c * d * h) - (c * e * g) - (b * d * i) - (a * f * h);
}

template <typename TMatrix>
requires ConceptConstexprMatrix<TMatrix>
constexpr auto DeterminantStatic(const TMatrix& matrix)
{
    if constexpr(matrix.GetRows() == 2 && matrix.GetColumns() == 2)
        return Determinant2x2(matrix);
    else if constexpr(matrix.GetRows() == 3 && matrix.GetColumns() == 3)
        return Determinant3x3(matrix);
    else
    {
        assert(false); // todo: implement
        return typename TMatrix::DataType{}; //
    }
}

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto DeterminantDynamic(const TMatrix& matrix)
{
    assert(matrix.GetRows() == matrix.GetColumns());
    if(matrix.GetRows() == 2 && matrix.GetColumns() == 2)
        return Determinant2x2(matrix);
    else if(matrix.GetRows() == 3 && matrix.GetColumns() == 3)
        return Determinant3x3(matrix);
    else
    {
        assert(false); // todo: implement
        return typename TMatrix::DataType{}; //
    }
}

template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto Determinant(const TMat& matrix)
{
    using TRet = typename TMat::DataType;
    if constexpr(ConceptIdentity<TMat>)
        return TRet{1};

    else if constexpr(ConceptDiagonal<TMat>)
        return DeterminantDiagonal(matrix);

    else if constexpr(ConceptConstexprMatrix<TMat>)
        return DeterminantStatic(matrix);

    else
        return DeterminantDynamic(matrix);
}

} // namespace linalg
} // namespace vic