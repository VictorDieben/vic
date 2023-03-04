#pragma once

#include "vic/linalg/definitions.h"
#include <utility>

namespace vic
{
namespace linalg
{

template <Col columns>
constexpr MatrixSize RowMayorRowColToIndex(const Row i, const Col j)
{
    return (i * columns) + j;
}

constexpr MatrixSize RowMayorRowColToIndex(const Row i, const Col j, const Col columns)
{
    return (i * columns) + j; //
}

template <Col columns>
constexpr std::pair<Row, Col> RowMayorIndexToRowCol(const MatrixSize index)
{
    const auto i = index / columns;
    const auto j = index - (i * columns);
    return std::pair(i, j);
}

constexpr std::pair<Row, Col> RowMayorIndexToRowCol(const MatrixSize index, const Col columns)
{
    const auto i = index / columns;
    const auto j = index - (i * columns);
    return std::pair(i, j);
}

//
//
//

template <Row rows>
constexpr MatrixSize ColMayorRowColToIndex(const Row i, const Col j)
{
    return (j * rows) + i;
}

constexpr MatrixSize ColMayorRowColToIndex(const Row i, const Col j, const Row rows)
{
    return (j * rows) + i; //
}

template <Row rows>
constexpr std::pair<Row, Col> ColMayorIndexToRowCol(const MatrixSize index)
{
    const auto j = index / rows;
    const auto i = index - (j * rows);
    return std::pair(i, j);
}

constexpr std::pair<Row, Col> ColMayorIndexToRowCol(const MatrixSize index, const Col columns, const Row rows)
{
    const auto j = index / rows;
    const auto i = index - (j * rows);
    return std::pair(i, j);
}

} // namespace linalg
} // namespace vic