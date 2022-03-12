#pragma once

// This file implements the matrix multiplication for all different types of matrices
#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/linalg/traits.h"

namespace vic
{
namespace linalg
{

// multiplication of two equal matrix types
template <typename TMat, typename TFloat, class TRet = decltype(typename TMat::DataType() * TFloat())>
requires ConceptMatrix<TMat>
constexpr auto MatmulScalar(const TMat& mat, const TFloat& scalar)
{
    using T = typename TMat::DataType;
    constexpr auto rows = TMat::GetRows();
    constexpr auto cols = TMat::GetColumns();

    if constexpr(std::is_same_v<TMat, Zeros<T, rows, cols>>)
    {
        return Zeros<T, rows, cols>{};
    }
    else if constexpr(std::is_same_v<TMat, Identity<T, rows>>)
    {
        return Diagonal<T, rows, cols>{scalar};
    }
    else if constexpr(std::is_same_v<TMat, Diagonal<T, rows, cols>>)
    {
        Diagonal<T, rows, cols> result{};
        for(std::size_t i = 0; i < Min(rows, cols); ++i)
            result.At(i, i) = mat.Get(i, i) * scalar;
        return result;
    }
    else
    {
        Matrix<TRet, rows, cols> result{};
        for(std::size_t i = 0; i < rows; ++i)
            for(std::size_t j = 0; j < cols; ++j)
                result.At(i, j) = mat.Get(i, j) * scalar;
        return result;
    }
}

// multiplication of any 2 matrices of constant size
template <typename TMat1, typename TMat2, class TRet = decltype(typename TMat1::DataType() * typename TMat2::DataType())>
requires ConceptConstexprMatrix<TMat1> && ConceptConstexprMatrix<TMat2>
constexpr auto MatmulStatic(const TMat1& mat1, const TMat2& mat2)
{
    // TODO(vicdie): specialize for certain types of matrix multiplications (e.g. diag*diag)

    static_assert(mat1.GetColumns() == mat2.GetRows());
    Matrix<TRet, mat1.GetRows(), mat2.GetColumns()> result{};
    for(std::size_t i = 0; i < mat1.GetRows(); ++i)
        for(std::size_t j = 0; j < mat2.GetColumns(); ++j)
            for(std::size_t k = 0; k < mat1.GetColumns(); ++k)
                result.At(i, j) += (mat1.Get(i, k) * mat2.Get(k, j));
    return result;
}

// multiplication of any 2 matrices where size is not (fully) known
template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto MatmulDynamic(const TMat1& mat1, const TMat2& mat2)
{
    using ReturnType = decltype(typename TMat1::DataType() * typename TMat2::DataType());
    assert(mat1.GetColumns() == mat2.GetRows());

    if constexpr(ConceptConstexprRows<TMat1> && ConceptConstexprColumns<TMat2>)
    {
        // output can be fixed size, even though the two inputs are not
        return Matrix<ReturnType, mat1.GetRows(), mat2.GetColumns()>();
    }
    else if constexpr(ConceptConstexprRows<TMat1>)
    {
        // output can have constant row size
        return MatrixRowConst<ReturnType, mat1.GetRows()>(mat2.GetColumns());
    }
    else if constexpr(ConceptConstexprColumns<TMat2>)
    {
        // output can have constant col size
        return MatrixColConst<ReturnType, mat2.GetColumns()>(mat1.GetRows());
    }
    else
    {
        // don't bother optimizing, no dimension is known
        MatrixDynamic<ReturnType> result{mat1.GetRows(), mat2.GetColumns()};
        for(std::size_t i = 0; i < mat1.GetRows(); ++i)
            for(std::size_t j = 0; j < mat2.GetColumns(); ++j)
                for(std::size_t k = 0; k < mat1.GetColumns(); ++k)
                    result.At(i, j) += (mat1.Get(i, k) * mat2.Get(k, j));
        return result;
    }
}

// selector for the correct type of matrix multiplication
template <typename TMat1, typename TMat2>
constexpr auto Matmul(const TMat1& mat1, const TMat2& mat2)
{
    constexpr bool isFloat1 = IsFloatOrIntegral<std::decay_t<TMat1>>::value;
    constexpr bool isFloat2 = IsFloatOrIntegral<std::decay_t<TMat2>>::value;

    if constexpr(isFloat1 && isFloat2)
        return mat1 * mat2;

    else if constexpr(isFloat1 && !isFloat2)
        return MatmulScalar(mat2, mat1);

    else if constexpr(!isFloat1 && isFloat2)
        return MatmulScalar(mat1, mat2);

    else if constexpr(ConceptConstexprMatrix<TMat1> && ConceptConstexprMatrix<TMat2>)
    {
        static_assert(mat1.GetColumns() == mat2.GetRows());
        return MatmulStatic(mat1, mat2);
    }
    else
    {
        assert(mat1.GetColumns() == mat2.GetRows());
        return MatmulDynamic(mat1, mat2);
    }
}

} // namespace linalg
} // namespace vic