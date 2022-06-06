#pragma once

#include <cstddef>
#include <type_traits>

// This file specifies the requirements for what a matrix/vector constitutes

namespace vic
{
namespace linalg
{

// This helper only compiles if template argument is constexpr std::size_t
template <std::size_t>
using ConstexprSizeHelper = void;

template <typename T>
concept ConceptConstexprRows = requires
{
    T::GetRows();
    requires std::is_same_v<std::size_t, decltype(T::GetRows())>;
    typename ConstexprSizeHelper<T::GetRows()>;
};

template <typename T>
concept ConceptConstexprColumns = requires
{
    T::GetColumns();
    requires std::is_same_v<std::size_t, decltype(T::GetColumns())>;
    typename ConstexprSizeHelper<T::GetColumns()>;
};

template <typename T>
concept ConceptMatrix = requires(T mat)
{
    // TODO: check that these statements have the correct return type
    T::DataType;
    mat.GetRows();
    mat.GetColumns();
    mat.Get(std::size_t(0), std::size_t(0));
};

// TODO: implement, check that GetRows() and GetColumns() are constexpr
template <typename T>
concept ConceptConstexprMatrix = ConceptMatrix<T> && ConceptConstexprRows<T> && ConceptConstexprColumns<T>;

template <typename T>
concept ConceptVector = ConceptMatrix<T> && requires(T mat)
{
    requires(mat.GetColumns() == 1);
};

template <typename TMat>
struct IsSquare
{
    constexpr static bool value = (TMat::GetRows() == TMat::GetColumns());
};

// TODO: fix this, IsSquare should not be needed
template <typename T>
concept ConceptSquareMatrix = ConceptMatrix<T> && IsSquare<T>::value;

template <typename T>
using Base = std::remove_const_t<std::decay_t<T>>;

template <typename TMat>
struct IsDiagonal
{
    constexpr static bool value = IsSquare<TMat>::value; // todo: make this a property of matrix types
};

template <typename T1, typename T2>
struct HasSameType
{
    constexpr static bool value = std::is_same_v<T1::DataType, T2::DataType>;
};

template <typename T1, typename T2>
struct HasSameShape
{
    constexpr static bool value = (T1::GetRows() == T2::GetRows()) && (T1::GetColumns() == T2::GetColumns());
};

template <typename T>
struct IsFloatOrIntegral
{
    constexpr static bool value = (std::is_integral_v<T> || std::is_floating_point_v<T>);
};

template <typename TMatrix>
constexpr bool IsPositiveDefinite(const TMatrix& matrix)
{
    return true; // TODO: implement
}

template <typename TMatrix>
constexpr bool IsTotallyPositive(const TMatrix& matrix)
{
    if constexpr(!ConceptSquareMatrix<TMatrix>)
    {
        return false;
    }
    else
    {
        return true; // TODO: implement
    }
}

} // namespace linalg
} // namespace vic