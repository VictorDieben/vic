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
struct is_square
{
    constexpr static bool value = (TMat::GetRows() == TMat::GetColumns());
};

// TODO: fix this, IsSquare should not be needed
template <typename T>
concept ConceptSquareMatrix = ConceptMatrix<T> && is_square<T>::value;

template <typename T>
using Base = std::remove_const_t<std::decay_t<T>>;

template <typename T>
struct is_diagonal
{
    static constexpr bool value = false;
};

template <typename T1, typename T2>
struct is_same_type
{
    constexpr static bool value = std::is_same_v<typename T1::DataType, typename T2::DataType>;
};

template <typename T1, typename T2>
struct is_same_shape
{
    constexpr static bool value = (T1::GetRows() == T2::GetRows()) && (T1::GetColumns() == T2::GetColumns());
};

template <typename T>
struct is_float_or_integral
{
    constexpr static bool value = (std::is_integral_v<T> || std::is_floating_point_v<T>);
};

template <typename T>
struct is_positive_definite
{
    static constexpr bool value = false;
};

template <typename TMatrix>
constexpr bool is_totally_positive(const TMatrix& matrix)
{
    if constexpr(!ConceptSquareMatrix<TMatrix>)
    {
        return false;
    }
    else
    {
        return false; // TODO: implement
    }
}

template <typename T>
inline constexpr bool is_float_or_integral_v = is_float_or_integral<T>::value;
template <typename T>
inline constexpr bool is_same_shape_v = is_same_shape<T>::value;
template <typename T>
inline constexpr bool is_diagonal_v = is_diagonal<T>::value;
template <typename T>
inline constexpr bool is_square_v = is_square<T>::value;
template <typename T1, typename T2>
inline constexpr bool is_same_type_v = is_same_type<T1, T2>::value;
template <typename T>
inline constexpr bool is_positive_definite_v = is_positive_definite<T>::value;
template <typename T>
inline constexpr bool is_totally_positive_v = is_totally_positive<T>::value;

} // namespace linalg
} // namespace vic