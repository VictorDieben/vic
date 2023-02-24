#pragma once

#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/linalg/traits.h"

#include <type_traits>

namespace vic
{
namespace linalg
{

template <typename TMat1, typename TMat2>
struct add_result_size
{
    static constexpr bool RowConst = ConceptConstexprRows<TMat1> || ConceptConstexprRows<TMat2>;
    static constexpr bool ColConst = ConceptConstexprColumns<TMat1> || ConceptConstexprColumns<TMat2>;

    constexpr static std::size_t Rows = !RowConst //
                                            ? 0u //
                                            : ConceptConstexprRows<TMat1> //
                                                  ? TMat1::GetRows() //
                                                  : TMat2::GetRows();
    constexpr static std::size_t Columns = !ColConst //
                                               ? 0u //
                                               : ConceptConstexprColumns<TMat1> //
                                                     ? TMat1::GetColumns() //
                                                     : TMat2::GetColumns();

    constexpr static ESizeType Size = ToSize<RowConst, ColConst>();
};

// selector for default Add(mat1, mat2) return type
template <typename TMat1, typename TMat2>
struct default_add
{

private:
    static constexpr bool row_const = ConceptConstexprRows<TMat1> || ConceptConstexprRows<TMat2>;
    static constexpr bool col_const = ConceptConstexprColumns<TMat1> || ConceptConstexprColumns<TMat2>;

    static constexpr std::size_t CalculateRows()
    {
        if constexpr(ConceptConstexprRows<TMat1>)
            return TMat1::GetRows();
        else if constexpr(ConceptConstexprRows<TMat2>)
            return TMat2::GetRows();
        else
            return 0; // cannot deduce
    }
    static constexpr std::size_t CalculateColumns()
    {
        if constexpr(ConceptConstexprColumns<TMat1>)
            return TMat1::GetColumns();
        else if constexpr(ConceptConstexprColumns<TMat2>)
            return TMat2::GetColumns();
        else
            return 0; // cannot deduce
    }
    static constexpr std::size_t rows = CalculateRows();
    static constexpr std::size_t cols = CalculateColumns();

    using value_type = decltype(typename TMat1::DataType() + typename TMat2::DataType());

    // possible results
    using MatConst = Matrix<value_type, rows, cols>;
    using MatRowConst = MatrixRowConst<value_type, rows>;
    using MatColConst = MatrixColConst<value_type, cols>;
    using MatDynamic = MatrixDynamic<value_type>;

public:
    // default type that should be retured by Add():
    using type = std::conditional_t<row_const && col_const, //
                                    MatConst, //
                                    std::conditional_t<row_const, //
                                                       MatRowConst, //
                                                       std::conditional_t<col_const, //
                                                                          MatColConst, //
                                                                          MatDynamic>>>; //
};

template <typename TMat1, typename TMat2>
using default_add_t = default_add<TMat1, TMat2>::type;

// addition of any 2 matrices of constant size
template <typename TMat1, typename TMat2, class TMatRet = default_add_t<TMat1, TMat2>>
requires ConceptConstexprMatrix<TMat1> && ConceptConstexprMatrix<TMat2>
constexpr auto AddStatic(const TMat1& mat1, const TMat2& mat2)
{
    static_assert(TMat1::GetColumns() == TMat2::GetColumns());
    static_assert(TMat1::GetRows() == TMat2::GetRows());

    TMatRet result{};
    for(std::size_t i = 0; i < TMat1::GetRows(); ++i)
        for(std::size_t j = 0; j < TMat1::GetColumns(); ++j)
            result.At(i, j) = mat1.Get(i, j) + mat2.Get(i, j);
    return result;
}

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto AddConstant(const TMatrix& matrix, const typename TMatrix::DataType& value)
{
    Matrix<typename TMatrix::DataType, TMatrix::GetRows(), TMatrix::GetColumns()> result{};
    for(std::size_t i = 0; i < TMatrix::GetRows(); ++i)
        for(std::size_t j = 0; j < TMatrix::GetColumns(); ++j)
            result.At(i, j) = matrix.Get(i, j) + value;
    return result;
}
template <typename TMat1, typename TMat2, typename TMatRet = default_add_t<TMat1, TMat2>>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto AddGeneral(const TMat1& mat1, const TMat2& mat2)
{
    TMatRet result{mat1.GetRows(), mat1.GetColumns()};
    for(std::size_t i = 0; i < mat1.GetRows(); ++i)
        for(std::size_t j = 0; j < mat1.GetColumns(); ++j)
            result.At(i, j) = mat1.Get(i, j) + mat2.Get(i, j);
    return result;
}

template <typename TMat1, typename TMat2>
constexpr auto Add(const TMat1& mat1, const TMat2& mat2)
{
    // add constant to matrix
    constexpr bool isNumber1 = is_float_or_integral_v<std::decay_t<TMat1>>;
    constexpr bool isNumber2 = is_float_or_integral_v<std::decay_t<TMat2>>;
    if constexpr(isNumber1 && isNumber2)
        return mat1 + mat2;

    else if constexpr(isNumber1 && !isNumber2)
        return AddConstant(mat2, mat1);

    else if constexpr(!isNumber1 && isNumber2)
        return AddConstant(mat1, mat2);

    else
        return AddGeneral(mat1, mat2);
}

// TODO: make a selector for proper algorithms
template <typename TMat1, typename TMat2, typename... Types>
constexpr auto Add(const TMat1& mat1, const TMat2& mat2, const Types... others)
{
    return Add(Add(mat1, mat2), others...);
}

} // namespace linalg
} // namespace vic