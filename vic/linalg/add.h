#pragma once

#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/linalg/traits.h"

#include <type_traits>

namespace vic
{
namespace linalg
{

constexpr EDistribution AdditionDistribution(const EDistribution first, const EDistribution second)
{
    // assuming the addition sizes are valid, what would the result distribution be?
    if(first == second)
        return first;
    if(first > second)
        return AdditionDistribution(second, first);

    // start of actual rules
    if(first == EDistribution::Full || first == EDistribution::Unknown)
        return EDistribution::Full;

    if(first == EDistribution::Sparse)
        return EDistribution::Sparse;

    if(first == EDistribution::UpperTriangular)
    {
        if(second == EDistribution::UpperTriangular || //
           second == EDistribution::Diagonal || //
           second == EDistribution::StrictUpperTriangular)
            return EDistribution::UpperTriangular;
        else if(second == EDistribution::LowerTriangular || //
                second == EDistribution::StrictLowerTriangular)
            return EDistribution::Full;
    }

    if(first == EDistribution::LowerTriangular)
    {
        if(second == EDistribution::Diagonal || //
           second == EDistribution::StrictLowerTriangular)
            return EDistribution::LowerTriangular;
        else if(second == EDistribution::UpperTriangular || //
                second == EDistribution::StrictUpperTriangular)
            return EDistribution::Full;
    }

    if(first == EDistribution::Diagonal)
    {
        if(second == EDistribution::Diagonal)
            return EDistribution::Diagonal;
        else if(second == EDistribution::StrictUpperTriangular)
            return EDistribution::UpperTriangular;
        else if(second == EDistribution::StrictLowerTriangular)
            return EDistribution::LowerTriangular;
    }

    if(first == EDistribution::StrictUpperTriangular)
        if(second == EDistribution::StrictLowerTriangular)
            return EDistribution::Full;

    return EDistribution::Unknown; // Not ideal, find way to let user override this
}

constexpr ESizeType AdditionSizeType(const ESizeType first, const ESizeType second)
{
    if(first == second)
        return first;
    if(first > second)
        return AdditionSizeType(second, first);

    if(first == ESizeType::Constant)
    {
        return ESizeType::Constant;
    }
    else if(first == ESizeType::RowConstant)
    {
        if(second == ESizeType::ColConstant)
            return ESizeType::Constant;
        else if(second == ESizeType::Dynamic)
            return ESizeType::RowConstant;
    }
    else if(first == ESizeType::ColConstant)
    {
        if(second == ESizeType::Dynamic)
            return ESizeType::ColConstant;
    }

    return ESizeType::Dynamic; // not ideal, should not happen
}

template <typename TMat1, typename TMat2>
constexpr std::size_t AdditionRows()
{
    if constexpr(ConceptConstexprRows<TMat1>)
        return TMat1::GetRows();
    else if constexpr(ConceptConstexprRows<TMat2>)
        return TMat2::GetRows();
    else
        return 0u;
}

template <typename TMat1, typename TMat2>
constexpr std::size_t AdditionColumns()
{
    if constexpr(ConceptConstexprColumns<TMat1>)
        return TMat1::GetColumns();
    else if constexpr(ConceptConstexprColumns<TMat2>)
        return TMat2::GetColumns();
    else
        return 0u;
}

template <typename TMat1, typename TMat2>
struct add_result_size
{
    static constexpr ESizeType SizeType = AdditionSizeType(TMat1::Size, TMat2::Size);

    static constexpr bool RowConst = IsRowConstant(SizeType);
    static constexpr bool ColConst = IsColumnConstant(SizeType);

    constexpr static std::size_t Rows = AdditionRows<TMat1, TMat2>();
    constexpr static std::size_t Columns = AdditionColumns<TMat1, TMat2>();
};

// selector for default Add(mat1, mat2) return type
template <typename TMat1, typename TMat2>
struct default_add
{
private:
    using ResultSize = add_result_size<TMat1, TMat2>;
    constexpr static ESizeType restype = ResultSize::SizeType;

    using value_type = decltype(typename TMat1::DataType() + typename TMat2::DataType());

    //// possible results
    //using MatConst = Matrix<value_type, ResultSize::Rows, ResultSize::Columns>;
    //using MatRowConst = MatrixRowConst<value_type, ResultSize::Rows>;
    //using MatColConst = MatrixColConst<value_type, ResultSize::Columns>;
    //using MatDynamic = MatrixDynamic<value_type>;

public:
    // default type that should be retured by Add():
    using type = std::conditional_t<restype == ESizeType::Constant, //
                                    Matrix<value_type, ResultSize::Rows, ResultSize::Columns>, //
                                    std::conditional_t<restype == ESizeType::RowConstant, //
                                                       MatrixRowConst<value_type, ResultSize::Rows>, //
                                                       std::conditional_t<restype == ESizeType::ColConstant, //
                                                                          MatrixColConst<value_type, ResultSize::Columns>, //
                                                                          MatrixDynamic<value_type>>>>; //
};

template <typename TMat1, typename TMat2>
using default_add_t = default_add<TMat1, TMat2>::type;

//// addition of any 2 matrices of constant size
//template <typename TMat1, typename TMat2, class TMatRet = default_add_t<TMat1, TMat2>>
//requires ConceptConstexprMatrix<TMat1> && ConceptConstexprMatrix<TMat2>
//constexpr auto AddStatic(const TMat1& mat1, const TMat2& mat2)
//{
//    static_assert(TMat1::GetColumns() == TMat2::GetColumns());
//    static_assert(TMat1::GetRows() == TMat2::GetRows());
//
//    TMatRet result{};
//    for(std::size_t i = 0; i < TMat1::GetRows(); ++i)
//        for(std::size_t j = 0; j < TMat1::GetColumns(); ++j)
//            result.At(i, j) = mat1.Get(i, j) + mat2.Get(i, j);
//    return result;
//}

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
    assert(mat1.GetRows() == mat2.GetRows() && mat1.GetColumns() == mat2.GetColumns());
    using AddResult = add_result_size<TMat1, TMat2>;

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