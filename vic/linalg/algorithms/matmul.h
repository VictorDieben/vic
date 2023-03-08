#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/matrices/diagonal.h"
#include "vic/linalg/matrices/matrix.h"
#include "vic/linalg/traits.h"

#include <vic/utils.h>

namespace vic
{
namespace linalg
{

constexpr EDistribution MatmulDistribution(const EDistribution first, const EDistribution second)
{
    if(first > second)
        return MatmulDistribution(second, first);

    else if(first == EDistribution::Full)
        return EDistribution::Full;

    else if(first == EDistribution::Sparse)
        return EDistribution::Full;

    else if(first == EDistribution::Diagonal)
    {
        if(second == EDistribution::Diagonal)
            return EDistribution::Diagonal;
        else
            return EDistribution::Full;
    }
    else
        return EDistribution::Full;
}

template <typename TShape1, typename TShape2>
using MatmulResultShape = Shape<TShape1::rows, TShape2::cols>;

template <typename TMat1, typename TMat2>
constexpr auto MatmulDiagonal(const TMat1& mat1, const TMat2& mat2)
{
    assert(mat1.GetColumns() == mat2.GetRows());
    using TValue = decltype(typename TMat1::DataType() * typename TMat2::DataType());
    using TShape = MatmulResultShape<typename TMat1::ShapeType, typename TMat2::ShapeType>;

    Diagonal<TValue, TShape> result{mat1.GetRows(), mat2.GetColumns()};
    for(MatrixSize i = 0; i < Min(result.GetRows(), result.GetColumns()); ++i)
        result.At(i, i) = mat1.Get(i, i) * mat2.Get(i, i);

    return result;
}

template <typename TMat1, typename TMat2>
constexpr auto MatmulFull(const TMat1& mat1, const TMat2& mat2)
{
    assert(mat1.GetColumns() == mat2.GetRows());
    using TValue = decltype(typename TMat1::DataType() * typename TMat2::DataType());
    using shape = MatmulResultShape<typename TMat1::ShapeType, typename TMat2::ShapeType>;

    Matrix<TValue, shape> result{mat1.GetRows(), mat2.GetColumns()};

    for(Row i = 0; i < mat1.GetRows(); ++i)
        for(Col j = 0; j < mat2.GetColumns(); ++j)
            for(MatrixSize k = 0; k < mat1.GetColumns(); ++k)
                result.At(i, j) += (mat1.Get(i, k) * mat2.Get(k, j));

    return result;
}

template <typename TMat, typename TValue>
constexpr auto MatmulConstant(const TMat& mat, const TValue& value)
{
    using TRes = decltype(typename TMat::DataType{} * TValue{});
    if constexpr(TMat::Distribution == EDistribution::Diagonal)
    {
        Diagonal<TRes, typename TMat::ShapeType> result{mat.GetRows(), mat.GetColumns()};
        for(MatrixSize i = 0; i < Min(mat.GetRows(), mat.GetColumns()); ++i)
            result.At(i, i) = mat.Get(i, i) * value;
        return result;
    }
    else
    {
        Matrix<TRes, typename TMat::ShapeType> result{mat.GetRows(), mat.GetColumns()};
        for(Row i = 0; i < result.GetRows(); ++i)
            for(Col j = 0; j < result.GetColumns(); ++j)
                result.At(i, j) = mat.Get(i, j) * value;
        return result;
    }
}

// selector for the correct type of matrix multiplication
template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto MatmulMatrix(const TMat1& mat1, const TMat2& mat2)
{
    constexpr auto distribution = MatmulDistribution(TMat1::Distribution, TMat2::Distribution);

    if constexpr(distribution == EDistribution::Diagonal)
        return MatmulDiagonal(mat1, mat2);
    else
        return MatmulFull(mat1, mat2);
}

// selector for the correct type of matrix multiplication
template <typename TMat1, typename TMat2>
constexpr auto Matmul(const TMat1& mat1, const TMat2& mat2)
{
    if constexpr(!ConceptMatrix<TMat1> && !ConceptMatrix<TMat2>)
        return mat1 * mat2;
    else if constexpr(ConceptMatrix<TMat1> && !ConceptMatrix<TMat2>)
        return MatmulConstant(mat1, mat2);
    else if constexpr(!ConceptMatrix<TMat1> && ConceptMatrix<TMat2>)
        return MatmulConstant(mat2, mat1);
    else
        return MatmulMatrix(mat1, mat2);
}

// TODO: make a selector for proper algorithms
// TODO: For now just inverse the order: (a * (b * (c*d)))
template <typename TMat1, typename TMat2, typename... Types>
constexpr auto Matmul(const TMat1& mat1, const TMat2& mat2, const Types... others)
{
    return Matmul(Matmul(mat1, mat2), others...);
}

} // namespace linalg
} // namespace vic