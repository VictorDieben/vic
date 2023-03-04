#pragma once

#include <cstdint>
#include <limits>

namespace vic
{
namespace linalg
{
using MatrixSize = uint32_t;
using Row = MatrixSize;
using Col = MatrixSize; // todo: make unique type for rows and cols, so that we can use them separately

constexpr static MatrixSize UnknownSize = std::numeric_limits<MatrixSize>::max();

// an enum that declares in what order the matrix elements are listed
// todo: make diagonal ordering?
// todo: some matrices can have multiple types (diag is both row and column mayor), change?
enum class EOrdering
{
    Any, // ordering is not relevant
    RowMayor,
    ColumnMayor,
    Custom,
    Unknown
};

// gives a hint about the distribution of the elements in a matrix
// NOTE: try to order these from strongest to weakest type
enum class EDistribution
{
    Full,
    Unknown,
    Sparse,
    UpperTriangular,
    LowerTriangular,
    Diagonal,
    StrictUpperTriangular,
    StrictLowerTriangular
};

enum class ESizeType
{
    Constant,
    RowConstant,
    ColConstant,
    Dynamic
};

template <Row _rows, Col _cols>
struct Shape
{
    constexpr static Row rows = _rows;
    constexpr static Col cols = _cols;
    constexpr static bool IsRowConst() { return rows != UnknownSize; }
    constexpr static bool IsColConst() { return cols != UnknownSize; }
};

template <Row rows, Col cols>
using ConstShape = Shape<rows, cols>;

template <Row rows>
using RowConstShape = Shape<rows, UnknownSize>;

template <Col cols>
using ColConstShape = Shape<UnknownSize, cols>;

using UnknownShape = Shape<UnknownSize, UnknownSize>;

template <typename TShape, typename TConst, typename TRowConst, typename TColConst, typename TDynamic>
using TypeSelector = std::conditional_t<TShape::IsRowConst() && TShape::IsColConst(), //
                                        TConst, //
                                        std::conditional_t<TShape::IsRowConst(), //
                                                           TRowConst, //
                                                           std::conditional_t<TShape::IsColConst(), //
                                                                              TColConst, //
                                                                              TDynamic>>>; //

} // namespace linalg
} // namespace vic