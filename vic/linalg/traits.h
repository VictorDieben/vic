#pragma once

#include <cstddef>
#include <type_traits>

// This file specifies the requirements for what a matrix/vector constitutes

namespace vic
{
namespace linalg
{

// an enum that declares in what order the matrix elements are listed
// todo: make diagonal ordering?
enum class EOrdering
{
    Any, // ordering is not relevant
    RowMayor,
    ColumnMayor,
    Custom,
    Unknown
};

// gives a hint about the distribution of the elements in a matrix
enum class EDistribution
{
    Diagonal,
    Full,
    Sparse,
    UpperTriangular,
    LowerTriangular,
    StrictUpperTriangular,
    StrictLowerTriangular,
    Unknown
};

enum class ESizeType
{
    Dynamic,
    RowConstant,
    ColConstant,
    Constant
};

//template <ESizeType size, std::size_t rows, std::size_t cols>
//struct Size
//{
//    static constexpr ESizeType SizeType = size;
//    static constexpr std::size_t Rows = rows;
//    static constexpr std::size_t Columns = cols;
//};
//template <std::size_t rows, std::size_t cols>
//using ConstSize = Size<ESizeType::Constant, rows, cols>;
//template <std::size_t rows>
//using RowConstSize = Size<ESizeType::RowConstant, rows, 0u>;
//template <std::size_t cols>
//using ColConstSize = Size<ESizeType::ColConstant, 0u, cols>;
//using DynamicSize = Size<ESizeType::Dynamic, 0u, 0u>;

template <bool RowConst, bool ColConst>
constexpr ESizeType ToSize()
{
    if constexpr(!RowConst && !ColConst)
        return ESizeType::Dynamic;
    else if constexpr(RowConst && !ColConst)
        return ESizeType::RowConstant;
    else if constexpr(!RowConst && ColConst)
        return ESizeType::ColConstant;
    else
        return ESizeType::Constant;
}

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
    // todo: check that the two constructors exist
    //T{};
    //T{0u, 0u};
    // TODO: check that these statements have the correct return type
    typename T::DataType;
    T::Size;
    T::Ordering;
    T::Distribution;
    mat.GetRows();
    mat.GetColumns();
    mat.Get(std::size_t(0), std::size_t(0));
};

template <typename T>
concept ConceptSparse = requires(T mat)
{
    typename T::KeyType;
    T::Distribution == EDistribution::Sparse;
    // todo: check that T::Distribution is Sparse
    mat.Prune();
    mat.begin(); // todo: const iters, verify type
    mat.end();
};

template <typename T>
concept ConceptDiagonal = ConceptMatrix<T> && requires(T mat)
{
    T::Distribution == EDistribution::Diagonal;
};

// TODO: implement, check that GetRows() and GetColumns() are constexpr
template <typename T>
concept ConceptConstexprMatrix = ConceptMatrix<T> && ConceptConstexprRows<T> && ConceptConstexprColumns<T>;

template <typename T>
concept ConceptIdentity = ConceptConstexprMatrix<T> && requires(T mat)
{
    T::Distribution == EDistribution::Diagonal;
    T::GetRows() == T::GetColumns();
    T::TempIsIdentity == true; // todo: remove this boolean
    //std::same_as<T, Identity<typename T::DataType, T::GetRows()>>;
};

template <typename T>
concept ConceptVector = ConceptMatrix<T> && requires(T mat)
{
    requires(T::GetColumns() == 1);
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

template <typename T>
struct is_sparse
{
    static constexpr bool value = false;
    // todo: maybe this should be a concept,
    // which checks that the matrix has begin() and end() iterators to all non-zero elements
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
    static constexpr bool value = false; // todo: overloads for specific types
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
template <typename T1, typename T2>
inline constexpr bool is_same_shape_v = is_same_shape<T1, T2>::value;
template <typename T>
inline constexpr bool is_diagonal_v = is_diagonal<T>::value;
template <typename T>
inline constexpr bool is_sparse_v = is_sparse<T>::value;
template <typename T>
inline constexpr bool is_square_v = is_square<T>::value;
template <typename T1, typename T2>
inline constexpr bool is_same_type_v = is_same_type<T1, T2>::value;
template <typename T>
inline constexpr bool is_positive_definite_v = is_positive_definite<T>::value;

} // namespace linalg
} // namespace vic