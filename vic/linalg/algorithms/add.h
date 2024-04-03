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

constexpr EDistribution AdditionDistribution(const EDistribution first, const EDistribution second)
{
    // assuming the addition sizes are valid, what would the result distribution be?
    if(first == second)
        return first;
    if(first > second)
        return AdditionDistribution(second, first);

    // start of actual rules
    if(first == EDistribution::Full || first == EDistribution::Unknown)
        return EDistribution::Full;

    if(first == EDistribution::Sparse)
        return EDistribution::Sparse;

    if(first == EDistribution::UpperTriangular)
    {
        if(second == EDistribution::UpperTriangular || //
           second == EDistribution::Diagonal || //
           second == EDistribution::StrictUpperTriangular)
            return EDistribution::UpperTriangular;
        else if(second == EDistribution::LowerTriangular || //
                second == EDistribution::StrictLowerTriangular)
            return EDistribution::Full;
    }

    if(first == EDistribution::LowerTriangular)
    {
        if(second == EDistribution::Diagonal || //
           second == EDistribution::StrictLowerTriangular)
            return EDistribution::LowerTriangular;
        else if(second == EDistribution::UpperTriangular || //
                second == EDistribution::StrictUpperTriangular)
            return EDistribution::Full;
    }

    if(first == EDistribution::Diagonal)
    {
        if(second == EDistribution::Diagonal)
            return EDistribution::Diagonal;
        else if(second == EDistribution::StrictUpperTriangular)
            return EDistribution::UpperTriangular;
        else if(second == EDistribution::StrictLowerTriangular)
            return EDistribution::LowerTriangular;
    }

    if(first == EDistribution::StrictUpperTriangular)
        if(second == EDistribution::StrictLowerTriangular)
            return EDistribution::Full;

    return EDistribution::Unknown; // Not ideal, find way to let user override this
}

template <typename TShape1, typename TShape2>
using AddResultShape = Shape<Min(TShape1::rows, TShape2::rows), Min(TShape1::cols, TShape2::cols)>;

template <typename TMat1, typename TMat2>
    requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2> // note: _not_ diagonal, this way you can add the diagonal of two non-diagonal matrices
constexpr auto AddDiagonal(const TMat1& mat1, const TMat2& mat2)
{
    assert(mat1.GetRows() == mat2.GetRows() && mat1.GetColumns() == mat2.GetColumns());
    using TValue = decltype(typename TMat1::DataType() * typename TMat2::DataType());

    using shape = AddResultShape<typename TMat1::ShapeType, typename TMat2::ShapeType>;

    Diagonal<TValue, shape> result{};
    for(MatrixSize i = 0; i < Min(result.GetRows(), result.GetColumns()); ++i)
        result.At(i, i) = mat1.Get(i, i) + mat2.Get(i, i);

    return result;
}

template <typename TMat1, typename TMat2>
constexpr auto AddFull(const TMat1& mat1, const TMat2& mat2)
{
    assert(mat1.GetRows() == mat2.GetRows() && mat1.GetColumns() == mat2.GetColumns());
    using TValue = decltype(typename TMat1::DataType() * typename TMat2::DataType());
    using shape = AddResultShape<typename TMat1::ShapeType, typename TMat2::ShapeType>;

    Matrix<TValue, shape> result{mat1.GetRows(), mat1.GetColumns()};
    for(Row i = 0; i < result.GetRows(); ++i)
        for(Col j = 0; j < result.GetColumns(); ++j)
            result.At(i, j) = mat1.Get(i, j) + mat2.Get(i, j);
    return result;
}

template <typename TMat, typename TValue>
constexpr auto AddConstant(const TMat& mat, const TValue& value)
{
    // todo: specialize for different matrix types
    using TRes = decltype(typename TMat::DataType() + TValue{});
    Matrix<TRes, typename TMat::ShapeType> result{mat.GetRows(), mat.GetColumns()};
    for(Row i = 0; i < result.GetRows(); ++i)
        for(Col j = 0; j < result.GetColumns(); ++j)
            result.At(i, j) = mat.Get(i, j) + value;
    return result;
}

template <typename TMat1, typename TMat2>
    requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto AddMatrix(const TMat1& mat1, const TMat2& mat2)
{
    constexpr auto distribution = AdditionDistribution(TMat1::Distribution, TMat2::Distribution);

    // todo: specialize for:
    // - sparse
    // - zeros
    // - identity

    if constexpr(distribution == EDistribution::Diagonal)
        return AddDiagonal(mat1, mat2);
    else
        return AddFull(mat1, mat2);
}

template <typename TMat1, typename TMat2>
    requires(ConceptMatrix<TMat1> || ConceptMatrix<TMat2>)
constexpr auto Add(const TMat1& mat1, const TMat2& mat2)
{
    if constexpr(ConceptMatrix<TMat1> && !ConceptMatrix<TMat2>)
        return AddConstant(mat1, mat2);
    else if constexpr(!ConceptMatrix<TMat1> && ConceptMatrix<TMat2>)
        return AddConstant(mat2, mat1);
    else
        return AddMatrix(mat1, mat2);
}

template <typename TMat1, typename TMat2, typename... Types>
constexpr auto Add(const TMat1& mat1, const TMat2& mat2, const Types... others)
{
    return ::vic::linalg::Add(::vic::linalg::Add(mat1, mat2), others...);
}

} // namespace linalg
} // namespace vic