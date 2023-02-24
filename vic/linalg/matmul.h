#pragma once

// This file implements the matrix multiplication for all different types of matrices
#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/linalg/traits.h"

namespace vic
{
namespace linalg
{

template <typename TMat1, typename TMat2>
struct matmul_result_size
{
    static constexpr bool RowConst = ConceptConstexprRows<TMat1>;
    static constexpr bool ColConst = ConceptConstexprColumns<TMat2>;

    // determine if the dimension along which the multiplication happens is constexpr.
    static constexpr bool MidDimensionConst = ConceptConstexprRows<TMat2> || ConceptConstexprColumns<TMat1>;

private:
    static constexpr std::size_t GetRows()
    {
        if constexpr(RowConst)
            return TMat1::GetRows();
        else
            return 0u;
    }
    static constexpr std::size_t GetCols()
    {
        if constexpr(ColConst)
            return TMat2::GetColumns();
        else
            return 0u;
    }
    static constexpr std::size_t GetMidDimension()
    {
        if constexpr(ConceptConstexprRows<TMat2>)
            return TMat2::GetRows();
        else if constexpr(ConceptConstexprColumns<TMat1>)
            return TMat1::GetColumns();
        else
            return 0u;
    }

public:
    constexpr static std::size_t Rows = GetRows();
    constexpr static std::size_t Columns = GetCols();

    constexpr static std::size_t MidDimension = GetMidDimension();

    constexpr static auto Size = ToSize<RowConst, ColConst>();
};

// selector for default Matmul(mat1, mat2) return type
template <typename TMat1, typename TMat2>
struct default_matmul
{
private:
    using Size = matmul_result_size<TMat1, TMat2>;

    using value_type = decltype(typename TMat1::DataType() * typename TMat2::DataType());

    // possible results
    using MatConst = Matrix<value_type, Size::Rows, Size::Columns>;
    using MatRowConst = MatrixRowConst<value_type, Size::Rows>;
    using MatColConst = MatrixColConst<value_type, Size::Columns>;
    using MatDynamic = MatrixDynamic<value_type>;

public:
    using type = std::conditional_t<Size::RowConst && Size::ColConst, //
                                    MatConst, //
                                    std::conditional_t<Size::RowConst, //
                                                       MatRowConst, //
                                                       std::conditional_t<Size::ColConst, //
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

    if constexpr(ConceptConstexprMatrix<TMat>)
    {
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
        else if constexpr(rows == cols && std::is_same_v<TMat, DiagonalConstant<T, rows>>)
        {
            return DiagonalConstant<TRet, rows>{mat.mValue * scalar};
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
    else if constexpr(TMat::Distribution == EDistribution::Sparse)
    {
        auto copy = mat;
        for(auto it = copy.begin(); it < copy.end(); ++it)
            *it = (*it) * scalar;
        return copy;
    }
    else if constexpr(TMat::Distribution == EDistribution::Diagonal)
    {
        DiagonalDynamic<T> result{mat.GetRows(), mat.GetRows()};
        for(std::size_t i = 0; i < mat.GetRows(); ++i)
            result.At(i, i) = mat.Get(i, i);
        return result;
    }
    else
    {
        auto copy = mat;
        for(std::size_t j = 0; j < copy.GetColumns(); ++j)
            for(std::size_t i = 0; i < copy.GetRows(); ++i)
                copy.At(i, j) = mat.Get(i, j) * scalar;
        return copy;
    }
}

// multiplication of any 2 matrices where size is not (fully) known
template <typename TMat1, typename TMat2, typename TMatRet = default_matmul_t<TMat1, TMat2>>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto MatmulGeneric(const TMat1& mat1, const TMat2& mat2)
{
    assert(mat1.GetColumns() == mat2.GetRows());

    TMatRet result{mat1.GetRows(), mat2.GetColumns()};

    for(std::size_t i = 0; i < mat1.GetRows(); ++i)
        for(std::size_t j = 0; j < mat2.GetColumns(); ++j)
            for(std::size_t k = 0; k < mat1.GetColumns(); ++k)
                result.At(i, j) += (mat1.Get(i, k) * mat2.Get(k, j));

    return result;
}

template <typename TMat1, typename TMat2>
requires ConceptSparse<TMat1> && ConceptVector<TMat2>
auto MatmulSparseVec(const TMat1& mat1, const TMat2& mat2)
{
    assert(mat1.GetColumns() == mat2.GetRows()); //
    MatrixDynamic<typename TMat1::DataType> result{mat1.GetRows(), 1};
    for(auto it = mat1.begin(); it != mat1.end(); ++it)
    {
        const auto i = it->first.first;
        const auto k = it->first.second;
        const auto val = it->second;
        result.At(i, 0) += val * mat2.Get(k, 0);
    }
    return result;
}

template <typename TMat1, typename TMat2>
requires ConceptDiagonal<TMat1> && ConceptDiagonal<TMat2>
constexpr auto MatmulDiagDiag(const TMat1& mat1, const TMat2& mat2)
{
    using MatmulResult = matmul_result_size<TMat1, TMat2>;

    if constexpr(MatmulResult::RowConst && MatmulResult::ColConst)
    {
        Diagonal<typename TMat1::DataType, MatmulResult::Rows, MatmulResult::Columns> result;
        for(std::size_t i = 0; i < MatmulResult::MidDimension; ++i)
            result.At(i, i) = mat1.Get(i, i) * mat2.Get(i, i);
        return result;
    }
    else
    {
        assert(mat1.GetColumns() == mat2.GetRows()); //
        DiagonalDynamic<typename TMat1::DataType> result{mat1.GetRows(), mat2.GetColumns()};
        for(std::size_t i = 0; i < Min(mat1.GetColumns(), mat2.GetRows()); ++i)
            result.At(i, i) = mat1.Get(i, i) * mat2.Get(i, i);
        return result;
    }
}

template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto MatmulMatrix(const TMat1& mat1, const TMat2& mat2)
{

    if constexpr(ConceptIdentity<TMat1> && ConceptIdentity<TMat2>) // I * I
    {
        static_assert(mat1.GetColumns() == mat2.GetRows());
        return mat1;
    }
    else if constexpr(ConceptIdentity<TMat1>) // I * mat
    {
        static_assert(mat1.GetColumns() == mat2.GetRows());
        return mat2;
    }
    else if constexpr(ConceptIdentity<TMat2>) // mat * I
    {
        static_assert(mat1.GetColumns() == mat2.GetRows());
        return mat1;
    }
    else if constexpr(ConceptSparse<TMat1> && ConceptVector<TMat2>)
        return MatmulSparseVec(mat1, mat2);

    else if constexpr(TMat1::Distribution == EDistribution::Diagonal && //
                      TMat2::Distribution == EDistribution::Diagonal)
        return MatmulDiagDiag(mat1, mat2);
    else
    {
        // generic case, result is a full matrix (with some constness)
        return MatmulGeneric<TMat1, TMat2>(mat1, mat2);
    }
}

// selector for the correct type of matrix multiplication
template <typename TMat1, typename TMat2>
constexpr auto Matmul(const TMat1& mat1, const TMat2& mat2)
{
    constexpr bool isFloat1 = is_float_or_integral_v<std::decay_t<TMat1>>;
    constexpr bool isFloat2 = is_float_or_integral_v<std::decay_t<TMat2>>;

    if constexpr(isFloat1 && isFloat2)
        return mat1 * mat2;

    else if constexpr(isFloat1 && !isFloat2)
        return MatmulScalar<TMat2, TMat1>(mat2, mat1);

    else if constexpr(!isFloat1 && isFloat2)
        return MatmulScalar<TMat1, TMat2>(mat1, mat2);

    else if constexpr(ConceptMatrix<TMat1> && ConceptMatrix<TMat2>)
        return MatmulMatrix(mat1, mat2);
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