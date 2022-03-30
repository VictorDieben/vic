#pragma once

#include "vic/linalg/matrices.h"
#include "vic/linalg/traits.h"

namespace vic
{
namespace linalg
{

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto AddConstant(const TMatrix& matrix, const typename TMatrix::DataType& value)
{
    Matrix<TMatrix::DataType, TMatrix::GetRows(), TMatrix::GetColumns()> result{};
    for(std::size_t i = 0; i < TMatrix::GetRows(); ++i)
        for(std::size_t j = 0; j < TMatrix::GetColumns(); ++j)
            result.At(i, j) = matrix.Get(i, j) + value;
    return result;
}

template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2> && HasSameShape<TMat1, TMat2>::value constexpr auto AddGeneral(const TMat1& mat1, const TMat2& mat2)
{
    Matrix<double, TMat1::GetRows(), TMat1::GetColumns()> result{};
    for(std::size_t i = 0; i < TMat1::GetRows(); ++i)
        for(std::size_t j = 0; j < TMat1::GetColumns(); ++j)
            result.At(i, j) = mat1.Get(i, j) + mat2.Get(i, j);
    return result;
}

template <typename TMat1, typename TMat2>
constexpr auto Add(const TMat1& mat1, const TMat2& mat2)
{
    // add constant to matrix
    constexpr bool isNumber1 = IsFloatOrIntegral<std::decay_t<TMat1>>::value;
    constexpr bool isNumber2 = IsFloatOrIntegral<std::decay_t<TMat2>>::value;
    if constexpr(isNumber1 && isNumber2)
        return mat1 + mat2;

    else if constexpr(isNumber1 && !isNumber2)
        return AddConstant(mat2, mat1);

    else if constexpr(!isNumber1 && isNumber2)
        return AddConstant(mat1, mat2);

    else
        return AddGeneral(mat1, mat2);
}

} // namespace linalg
} // namespace vic