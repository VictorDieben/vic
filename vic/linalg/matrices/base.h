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

template <typename TShape>
constexpr void VerifyShape(const Row rows, const Col cols) noexcept
{
    // todo: throwing in a noexcept function should still be possible in a constexpr function.
    // this way, we can exit if an error is detected

    //#ifdef _DEBUG
    //    if constexpr(TShape::IsRowConst())
    //        if constexpr(TShape::rows != rows)
    //            throw std::logic_error("invalid shape");
    //    if constexpr(TShape::IsColConst())
    //        if constexpr(TShape::cols != cols)
    //            throw std::logic_error("invalid shape");
    //#endif
}

template <typename TShape>
struct MatrixBaseConst
{
    using ShapeType = TShape;
    static_assert(TShape::IsRowConst() && TShape::IsColConst());
    constexpr MatrixBaseConst() = default;
    constexpr MatrixBaseConst(const Row rows, const Col cols)
    {
        VerifyShape<TShape>(rows, cols); //
    }

    constexpr static Row GetRows() { return TShape::rows; }
    constexpr static Col GetColumns() { return TShape::cols; }
};

template <typename TShape>
struct MatrixBaseRowConst
{
    using ShapeType = TShape;
    static_assert(TShape::IsRowConst() && !TShape::IsColConst());
    constexpr MatrixBaseRowConst(const Row rows, const Col cols)
        : mColumns(cols)
    {
        VerifyShape<TShape>(rows, cols); //
    }

    constexpr static Row GetRows() { return TShape::rows; }
    Col GetColumns() const { return mColumns; }

protected:
    Col mColumns{0};
};

template <typename TShape>
struct MatrixBaseColConst
{
    using ShapeType = TShape;
    static_assert(!TShape::IsRowConst() && TShape::IsColConst());
    constexpr MatrixBaseColConst(const Row rows, const Col cols)
        : mRows(rows)
    {
        VerifyShape<TShape>(rows, cols);
    }

    Row GetRows() const { return mRows; }
    constexpr static Col GetColumns() { return TShape::cols; }

protected:
    Row mRows{0};
};

template <typename TShape>
struct MatrixBaseDynamic
{
    using ShapeType = TShape;
    static_assert(!TShape::IsRowConst() && !TShape::IsColConst());
    constexpr MatrixBaseDynamic(const Row rows, const Col cols) noexcept
        : mRows(rows)
        , mColumns(cols)
    { }

    Row GetRows() const { return mRows; }
    Col GetColumns() const { return mColumns; }

protected:
    Row mRows{0};
    Col mColumns{0};
};

} // namespace detail

template <typename TShape>
using MatrixBaseSelector = TypeSelector<TShape, //
                                        detail::MatrixBaseConst<TShape>,
                                        detail::MatrixBaseRowConst<TShape>,
                                        detail::MatrixBaseColConst<TShape>,
                                        detail::MatrixBaseDynamic<TShape>>;

} // namespace linalg
} // namespace vic