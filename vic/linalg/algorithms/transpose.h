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
    using ResultShape = TransposeResultShape<typename TMatrix::ShapeType>;
    constexpr EDistribution ResultDist = TransposeDistribution(TMatrix::Distribution);

    if constexpr(ConceptIdentity<TMatrix>)
        return Identity<ResultDataType, ResultShape>{matrix.GetColumns(), matrix.GetRows()};

    else if constexpr(ConceptDiagonal<TMatrix>)
    {
        // constexpr-ness of rows and columns needs to be swapped:
        Diagonal<ResultDataType, ResultShape> result{matrix.GetColumns(), matrix.GetRows()};
        for(MatrixSize i = 0; i < Min(matrix.GetRows(), matrix.GetColumns()); ++i)
            result.At(i, i) = matrix.Get(i, i);
        return result;
    }
    else if constexpr(ConceptSparse<TMatrix>)
    {
        Sparse<ResultDataType, ResultShape> result{matrix.GetColumns(), matrix.GetRows()};
        for(const auto& [key, value] : matrix)
            result.At(key.second, key.first) = value; // todo: add all values to list, sort manually, then insert
        return result;
    }
    else
    {
        // todo: swap ordering? (row mayer <-> col mayor)
        Matrix<ResultDataType, ResultShape> result{matrix.GetColumns(), matrix.GetRows()};
        for(Row i = 0; i < matrix.GetRows(); ++i)
            for(Col j = 0; j < matrix.GetColumns(); ++j)
                result.At(j, i) = matrix.Get(i, j);
        return result;
    }
}

} // namespace linalg
} // namespace vic