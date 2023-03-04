#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/traits.h"
#include "vic/memory/flat_map.h"

#include <cassert>
#include <map>
#include <unordered_map>
#include <vector>

namespace vic
{
namespace linalg
{
namespace detail
{

template <typename T, typename TShape>
struct MatrixBaseConst
{
    using DataType = T;
    using ShapeType = TShape;
    static_assert(TShape::IsRowConst() && TShape::IsColConst());
    constexpr MatrixBaseConst() = default;
    constexpr MatrixBaseConst(const Row rows, const Col cols) { assert(TShape::rows == rows && TShape::cols == cols); }

    constexpr static Row GetRows() { return TShape::rows; }
    constexpr static Col GetColumns() { return TShape::cols; }
};

template <typename T, typename TShape>
struct MatrixBaseRowConst
{
    using DataType = T;
    using ShapeType = TShape;
    static_assert(TShape::IsRowConst() && !TShape::IsColConst());
    constexpr MatrixBaseRowConst(const Row rows, const Col cols)
        : mColumns(cols)
    {
        assert(TShape::cols == cols);
    }

    constexpr static Row GetRows() { return TShape::rows; }
    Col GetColumns() const { return mColumns; }

protected:
    const Col mColumns{0};
};

template <typename T, typename TShape>
struct MatrixBaseColConst
{
    using DataType = T;
    using ShapeType = TShape;
    static_assert(!TShape::IsRowConst() && TShape::IsColConst());
    constexpr MatrixBaseColConst(const Row rows, const Col cols)
        : mRows(rows)
    {
        assert(TShape::rows == rows);
    }

    Row GetRows() const { return mRows; }
    constexpr static Col GetColumns() { return TShape::cols; }

protected:
    const Row mRows{0};
};

template <typename T, typename TShape>
struct MatrixBaseDynamic
{
    using DataType = T;
    using ShapeType = TShape;
    static_assert(!TShape::IsRowConst() && !TShape::IsColConst());
    constexpr MatrixBaseDynamic(const Row rows, const Col cols)
        : mRows(rows)
        , mColumns(cols)
    { }

    Row GetRows() const { return mRows; }
    Col GetColumns() const { return mColumns; }

protected:
    const Row mRows{0};
    const Col mColumns{0};
};

} // namespace detail

template <typename T, typename TShape>
using MatrixBaseSelector = TypeSelector<T, //
                                        TShape,
                                        detail::MatrixBaseConst<T, TShape>,
                                        detail::MatrixBaseRowConst<T, TShape>,
                                        detail::MatrixBaseColConst<T, TShape>,
                                        detail::MatrixBaseDynamic<T, TShape>>;

} // namespace linalg
} // namespace vic