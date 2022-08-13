#pragma once

// This file implements the matrix multiplication for all different types of matrices
#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/linalg/traits.h"

namespace vic
{
namespace linalg
{

// selector for default Matmul(mat1, mat2) return type
template <typename TMat1, typename TMat2>
struct default_matmul
{
private:
    static constexpr bool row_const = ConceptConstexprRows<TMat1>;
    static constexpr bool col_const = ConceptConstexprColumns<TMat2>;

    static constexpr std::size_t CalculateRows()
    {
        if constexpr(ConceptConstexprRows<TMat1>)
            return TMat1::GetRows();
        else
            return 0; // cannot deduce
    }
    static constexpr std::size_t CalculateColumns()
    {
        if constexpr(ConceptConstexprColumns<TMat2>)
            return TMat2::GetColumns();
        else
            return 0; // cannot deduce
    }
    static constexpr std::size_t rows = CalculateRows();
    static constexpr std::size_t cols = CalculateColumns();

    using value_type = decltype(typename TMat1::DataType() * typename TMat2::DataType());

    // possible results
    using MatConst = Matrix<value_type, rows, cols>;
    using MatRowConst = MatrixRowConst<value_type, rows>;
    using MatColConst = MatrixColConst<value_type, cols>;
    using MatDynamic = MatrixDynamic<value_type>;

public:
    using type = std::conditional_t<row_const && col_const, //
                                    MatConst, //
                                    std::conditional_t<row_const, //
                                                       MatRowConst, //
                                                       std::conditional_t<col_const, //
                                                                          MatColConst, //
                                                                          MatDynamic>>>; //
};

template <typename TMat1, typename TMat2>
using default_matmul_t = default_matmul<TMat1, TMat2>::type;

// multiplication of two equal matrix types
template <typename TMat, typename TFloat>
requires ConceptMatrix<TMat>
constexpr auto MatmulScalar(const TMat& mat, const TFloat& scalar)
{
    using T = typename TMat::DataType;
    using TRet = decltype(T{} * TFloat());

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
        Diagonal<TRet, rows, cols> result{};
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
template <typename TMat1, typename TMat2, typename TMatRet = default_matmul_t<TMat1, TMat2>>
requires ConceptConstexprMatrix<TMat1> && ConceptConstexprMatrix<TMat2>
constexpr auto MatmulStatic(const TMat1& mat1, const TMat2& mat2)
{
    // TODO: specialize for certain types of matrix multiplications (e.g. diag*diag)

    static_assert(mat1.GetColumns() == mat2.GetRows());
    TMatRet result{};
    for(std::size_t i = 0; i < mat1.GetRows(); ++i)
        for(std::size_t j = 0; j < mat2.GetColumns(); ++j)
            for(std::size_t k = 0; k < mat1.GetColumns(); ++k)
                result.At(i, j) += (mat1.Get(i, k) * mat2.Get(k, j));
    return result;
}

// multiplication of any 2 matrices where size is not (fully) known
template <typename TMat1, typename TMat2, typename TMatRet = default_matmul_t<TMat1, TMat2>>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto MatmulDynamic(const TMat1& mat1, const TMat2& mat2)
{
    assert(mat1.GetColumns() == mat2.GetRows());

    const auto MatmulLambda = [&](const auto& mat1, const auto& mat2, auto& result) {
        for(std::size_t i = 0; i < mat1.GetRows(); ++i)
            for(std::size_t j = 0; j < mat2.GetColumns(); ++j)
                for(std::size_t k = 0; k < mat1.GetColumns(); ++k)
                    result.At(i, j) += (mat1.Get(i, k) * mat2.Get(k, j));
    };

    TMatRet result{mat1.GetRows(), mat2.GetColumns()};
    MatmulLambda(mat1, mat2, result);
    return result;
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
        return MatmulScalar<TMat2, TMat1>(mat2, mat1);

    else if constexpr(!isFloat1 && isFloat2)
        return MatmulScalar<TMat1, TMat2>(mat1, mat2);

    else if constexpr(ConceptConstexprMatrix<TMat1> && ConceptConstexprMatrix<TMat2>)
    {
        static_assert(mat1.GetColumns() == mat2.GetRows());
        return MatmulStatic<TMat1, TMat2>(mat1, mat2);
    }
    else
    {
        assert(mat1.GetColumns() == mat2.GetRows());
        return MatmulDynamic(mat1, mat2);
    }
}

// TODO: make a selector for proper algorithms
// TODO: For now just inverse the order: (a * (b * (c*d)))
template <typename TMat1, typename TMat2, typename... Types>
constexpr auto Matmul(const TMat1& mat1, const TMat2& mat2, const Types... others)
{
    return Matmul(Matmul(mat1, mat2), others...);
}

//template <typename TMat1, typename TMat2, typename... Types>
//constexpr auto Matmul(const Types... others, const TMat1& mat1, const TMat2& mat2)
//{
//    return Matmul(others..., Matmul(mat1, mat2));
//}

} // namespace linalg
} // namespace vic