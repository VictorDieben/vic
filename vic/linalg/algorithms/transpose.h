#pragma once

#include "vic/linalg/matrices/identity.h"
#include "vic/linalg/matrices/matrix.h"
#include "vic/linalg/matrices/sparse.h"
#include "vic/linalg/matrices/zeros.h"

#include "vic/linalg/definitions.h"
#include "vic/linalg/traits.h"

namespace vic
{
namespace linalg
{
constexpr EDistribution TransposeDistribution(const EDistribution dist)
{
    if(dist == EDistribution::Full)
        return EDistribution::Full;

    else if(dist == EDistribution::Sparse)
        return EDistribution::Sparse;

    else if(dist == EDistribution::UpperTriangular)
        return EDistribution::LowerTriangular;

    else if(dist == EDistribution::LowerTriangular)
        return EDistribution::UpperTriangular;

    else if(dist == EDistribution::Diagonal)
        return EDistribution::Diagonal;

    else if(dist == EDistribution::StrictUpperTriangular)
        return EDistribution::StrictLowerTriangular;

    else if(dist == EDistribution::StrictLowerTriangular)
        return EDistribution::StrictUpperTriangular;

    else
        return EDistribution::Unknown;
}

template <typename TShape>
using TransposeResultShape = Shape<TShape::cols, TShape::rows>;

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto Transpose(const TMatrix& matrix)
{
    using ResultDataType = typename TMatrix::DataType;
    using ResultShape = TransposeResultShape<TMatrix::ShapeType>;
    constexpr EDistribution ResultDist = TransposeDistribution(TMatrix::Distribution);

    if constexpr(ConceptIdentity<TMatrix>)
        return Identity<ResultDataType, ResultShape>{matrix.GetCols(), matrix.GetRows()};

    else if constexpr(ConceptDiagonal<TMatrix>)
    {
        // constexpr-ness of rows and columns needs to be swapped:
        Diagonal<ResultDataType, ResultShape> result{matrix.GetCols(), matrix.GetRows()};
        for(MatrixSize i = 0; i < Min(matrix.GetRows(), matrix.GetCols()); ++i)
            result.At(i, i) = matrix.At(i, i);
        return matrix;
    }
    else if constexpr(ConceptSparse<TMatrix>)
    {
        Sparse<ResultDataType, ResultShape> result{matrix.GetCols(), matrix.GetRows()};
        for(const auto& [key, value] : matrix)
            result.At(key.second, key.first) = value;
        return result;
    }
    else
    {
        Matrix<ResultDataType, ResultShape> result{matrix.GetCols(), matrix.GetRows()};
        for(Row i = 0; i < matrix.GetRows(); ++i)
            for(Col j = 0; j < matrix.GetColumns(); ++j)
                result.At(j, i) = matrix.At(i, j);
        return result;
    }
}

} // namespace linalg
} // namespace vic