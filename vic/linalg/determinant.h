#pragma once

#include "linalg.h"
#include "matrices.h"
#include "traits.h"

namespace vic
{
namespace linalg
{

template <typename TMatrix>
requires ConceptConstexprMatrix<TMatrix>
constexpr auto DeterminantStatic(const TMatrix& matrix)
{
    return typename TMatrix::DataType{}; //
}

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto DeterminantDynamic(const TMatrix& matrix)
{
    assert(matrix.GetRows() == matrix.GetColumns());
    typename TMatrix::DataType val{1}; // todo
    return val;
}

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto DeterminantDiagonal(const TMatrix& matrix)
{
    typename TMatrix::DataType val{1};
    for(std::size_t i = 0; i < TMatrix::GetRows(); ++i)
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
requires ConceptSquareMatrix<TMatrix>
constexpr auto DeterminantGeneral(const TMatrix& matrix)
{
    using TRet = typename TMatrix::DataType;
    constexpr auto nrows = TMatrix.GetRows();
    constexpr auto ncols = TMatrix.GetRows();

    if(nrows == 1 && ncols == 1)
        return matrix.Get(0, 0);

    Matrix<TRet, nrows - 1, ncols - 1> submatrix{};

    TRet ret{0};
    for(std::size_t i = 0; i < ncols; ++i)
    {
        // construct submatrix
        for(std::size_t j = 0; j < nrows - 1; ++j)
            for(std::size_t k = 0; k < ncols - 1; ++k)
                submatrix.At(j, k) = matrix.Get(j + 1, (i + k + 1) % ncols);

        const auto subdet = matrix.Get(0, i) * DeterminantGeneral(submatrix);
        ret += (i % 2) ? -subdet : subdet;
    }
    return ret;
}

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto Determinant(const TMatrix& matrix)
{
    using TRet = typename TMatrix::DataType;
    if constexpr(std::is_same_v<TMatrix, Identity<TMatrix::DataType, TMatrix::GetRows()>>)
    {
        return TRet{1};
    }
    else if constexpr(IsDiagonal<TMatrix>::value)
    {
        return DeterminantDiagonal(matrix);
    }
    else if constexpr(ConceptConstexprMatrix<TMatrix>)
    {
        return DeterminantStatic(matrix);
    }
    else
    {
        return DeterminantDynamic(matrix);
    }
}
} // namespace linalg
} // namespace vic