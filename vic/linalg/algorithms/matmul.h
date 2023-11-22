#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/matrices/diagonal.h"
#include "vic/linalg/matrices/matrix.h"
#include "vic/linalg/matrices/zeros.h"
#include "vic/linalg/traits.h"

#include <vic/utils.h>

namespace vic
{
namespace linalg
{

constexpr EDistribution MatmulDistribution(const EDistribution first, const EDistribution second)
{
    if(first > second)
        return MatmulDistribution(second, first);

    else if(first == EDistribution::Full)
        return EDistribution::Full;

    else if(first == EDistribution::Sparse)
        return EDistribution::Full;

    else if(first == EDistribution::Diagonal)
    {
        if(second == EDistribution::Diagonal)
            return EDistribution::Diagonal;
        else
            return EDistribution::Full;
    }
    else
        return EDistribution::Full;
}

template <typename TShape1, typename TShape2>
using MatmulResultShape = Shape<TShape1::rows, TShape2::cols>;

// todo: TriDiagonal x vector
template <typename TMat, typename TVec>
    requires ConceptTriDiagonal<TMat> && ConceptVector<TVec>
constexpr auto MatmulTriDiagVector(const TMat& mat, const TVec& vec)
{
    // multiplication between square diagonal and vector
    assert(mat.GetRows() == mat.GetColumns());
    assert(mat.GetColumns() == vec.GetRows());
    assert(vec.GetColumns() == 1u);

    using TValue = decltype(typename TMat::DataType() * typename TVec::DataType());
    using TShape = MatmulResultShape<typename TMat::ShapeType, typename TVec::ShapeType>;

    Matrix<TValue, TShape> result{mat.GetRows(), 1};

    // Note: separate indices to help with loop unrolling
    MatrixSize ia = 0;
    for(const auto& a : mat.A())
    {
        result.At(ia + 1, 0) = a * vec.Get(ia, 0);
        ia++;
    }

    MatrixSize ib = 0;
    for(const auto& b : mat.B())
    {
        result.At(ib, 0) += b * vec.Get(ib, 0);
        ib++;
    }

    MatrixSize ic = 0;
    for(const auto& c : mat.C())
    {
        result.At(ic, 0) += c * vec.Get(ic + 1, 0);
        ic++;
    }

    return result;
}

template <typename TMat1, typename TMat2>
    requires ConceptMatrix<TMat1> && ConceptVector<TMat2>
constexpr auto MatmulDiagonalVector(const TMat1& mat1, const TMat2& mat2)
{
    // multiplication between square diagonal and vector
    assert(mat1.GetRows() == mat1.GetColumns());
    assert(mat1.GetColumns() == mat2.GetRows());
    assert(mat2.GetColumns() == 1u);

    using TValue = decltype(typename TMat1::DataType() * typename TMat2::DataType());
    using TShape = MatmulResultShape<typename TMat1::ShapeType, typename TMat2::ShapeType>;

    Matrix<TValue, TShape> result{mat1.GetRows(), mat2.GetColumns()};
    for(MatrixSize i = 0; i < Min(result.GetRows(), result.GetColumns()); ++i)
        result.At(i, 0) = mat1.Get(i, i) * mat2.Get(i, 0);

    return result;
}

template <typename TMat1, typename TMat2>
    requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto MatmulDiagonal(const TMat1& mat1, const TMat2& mat2)
{
    assert(mat1.GetColumns() == mat2.GetRows());
    using TValue = decltype(typename TMat1::DataType() * typename TMat2::DataType());
    using TShape = MatmulResultShape<typename TMat1::ShapeType, typename TMat2::ShapeType>;

    Diagonal<TValue, TShape> result{mat1.GetRows(), mat2.GetColumns()};
    for(MatrixSize i = 0; i < Min(result.GetRows(), result.GetColumns()); ++i)
        result.At(i, i) = mat1.Get(i, i) * mat2.Get(i, i);

    return result;
}

template <typename TMat1, typename TMat2>
    requires ConceptRowStack<TMat1> && ConceptVector<TMat2>
constexpr auto MatmulRowStack(const TMat1& mat1, const TMat2& mat2)
{
    // todo: if mat2 is a vector, perform the two sub-multiplications separately,
    // if mat2 is a matrix, construct a new stack matrix as the answer?
    assert(mat1.GetColumns() == mat2.GetRows());
    using TValue = decltype(typename TMat1::DataType() * typename TMat2::DataType());

    const auto res1 = Matmul(mat1.mMat1, mat2);
    const auto res2 = Matmul(mat1.mMat2, mat2);

    // todo: stack more efficiently?
    return ToFull(ToRowStack(res1, res2));
}

template <typename TMat, typename TVec>
    requires ConceptColStack<TMat> && ConceptVector<TVec>
constexpr auto MatmulColStack(const TMat& mat, const TVec& vec)
{
    // todo: if mat2 is a vector, perform the two sub-multiplications separately,
    // if mat2 is a matrix, construct a new stack matrix as the answer?
    assert(mat.GetColumns() == vec.GetRows());
    using TValue = decltype(typename TMat::DataType() * typename TVec::DataType());

    // todo: some kind of Split() or Extract() function

    VectorN<TValue, decltype(mat.mMat1)::ShapeType::cols> vec1{mat.mMat1.GetRows(), 1};
    const Row mat1Rows = mat.mMat1.GetRows();
    for(uint32_t i = 0; i < mat1Rows; ++i)
        vec1.At(i, 0) = vec.Get(i, 0);

    VectorN<TValue, decltype(mat.mMat2)::ShapeType::cols> vec2{mat.mMat2.GetRows(), 1};
    const Row mat2Rows = mat.mMat2.GetRows();
    for(uint32_t i = 0; i < mat2Rows; ++i)
        vec2.At(i, 0) = vec.Get(mat1Rows + i, 0);

    const auto res1 = Matmul(mat.mMat1, vec1);
    const auto res2 = Matmul(mat.mMat2, vec2);

    // todo: stack more efficiently?
    return ToFull(Add(res1, res2));
}

template <typename TMat1, typename TMat2>
    requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto MatmulFull(const TMat1& mat1, const TMat2& mat2)
{
    assert(mat1.GetColumns() == mat2.GetRows());
    using TValue = decltype(typename TMat1::DataType() * typename TMat2::DataType());
    using shape = MatmulResultShape<typename TMat1::ShapeType, typename TMat2::ShapeType>;

    Matrix<TValue, shape> result{mat1.GetRows(), mat2.GetColumns()};

    for(Row i = 0; i < mat1.GetRows(); ++i)
        for(Col j = 0; j < mat2.GetColumns(); ++j)
            for(MatrixSize k = 0; k < mat1.GetColumns(); ++k)
                result.At(i, j) += (mat1.Get(i, k) * mat2.Get(k, j));

    // note: I tried transposing the second matrix first, but it did not result in performance improvements
    // I think because the size of the matrix is known at compile time, the compiler is capable of reordering the operations nicely.
    // about 20ns for one 4x4d multiplication is good enough for now

    //const auto tmp2 = Transpose(mat2);
    //for(Row i = 0; i < mat1.GetRows(); ++i)
    //    for(Row j = 0; j < tmp2.GetRows(); ++j)
    //        for(MatrixSize k = 0; k < mat1.GetColumns(); ++k)
    //            result.At(i, j) += (mat1.Get(i, k) * tmp2.Get(j, k));

    return result;
}

template <typename TMat, typename TValue>
    requires ConceptMatrix<TMat>
constexpr auto MatmulConstant(const TMat& mat, const TValue& value)
{
    using TResType = decltype(typename TMat::DataType() * TValue());
    using TResShape = typename TMat::ShapeType;

    if constexpr(ConceptZeros<TMat>)
    {
        return Zeros<TResType, TResShape>{mat.GetRows(), mat.GetColumns()};
    }
    if constexpr(TMat::Distribution == EDistribution::Diagonal)
    {
        Diagonal<TResType, TResShape> result{mat.GetRows(), mat.GetColumns()};
        for(MatrixSize i = 0; i < Min(mat.GetRows(), mat.GetColumns()); ++i)
            result.At(i, i) = mat.Get(i, i) * value;
        return result;
    }
    else
    {
        Matrix<TResType, TResShape> result{mat.GetRows(), mat.GetColumns()};
        for(Row i = 0; i < result.GetRows(); ++i)
            for(Col j = 0; j < result.GetColumns(); ++j)
                result.At(i, j) = mat.Get(i, j) * value;
        return result;
    }
}

// selector for the most efficient type of matrix multiplication
template <typename TMat1, typename TMat2>
    requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto MatmulMatrix(const TMat1& mat1, const TMat2& mat2)
{
    constexpr auto distribution = MatmulDistribution(TMat1::Distribution, TMat2::Distribution);
    if constexpr(ConceptZeros<TMat1> || ConceptZeros<TMat2>)
    {
        using ResulType = decltype(typename TMat1::DataType() * typename TMat2::DataType());
        using ResultShape = MatmulResultShape<TMat1, TMat2>;
        return Zeros<ResulType, ResultShape>{mat1.GetRows(), mat2.GetColumns()};
    }
    else if constexpr(ConceptIdentity<TMat1> || ConceptIdentity<TMat2>)
    {
        // todo: if datatype of mat1 is not equal to mat2, we might need to cast the matrix to another data type
        if constexpr(ConceptIdentity<TMat1>)
            return mat2;
        else // TMat2 is identity
            return mat1;
    }
    else if constexpr(ConceptStack<TMat1> && ConceptVector<TMat2>)
    {
        if constexpr(ConceptRowStack<TMat1>)
            return MatmulRowStack(mat1, mat2);
        else
            return MatmulColStack(mat1, mat2);
    }
    else if constexpr(ConceptDiagonal<TMat1> && ConceptVector<TMat2>)
        return MatmulDiagonal(mat1, mat2);
    else if constexpr(ConceptTriDiagonal<TMat1> && ConceptVector<TMat2>)
        return MatmulTriDiagVector(mat1, mat2);
    else
        return MatmulFull(mat1, mat2);
}

// selector for the correct type of matrix multiplication
template <typename TMat1, typename TMat2>
constexpr auto Matmul(const TMat1& mat1, const TMat2& mat2)
{
    if constexpr(!ConceptMatrix<TMat1> && !ConceptMatrix<TMat2>)
        return mat1 * mat2;
    else if constexpr(ConceptMatrix<TMat1> && !ConceptMatrix<TMat2>)
        return MatmulConstant(mat1, mat2);
    else if constexpr(!ConceptMatrix<TMat1> && ConceptMatrix<TMat2>)
        return MatmulConstant(mat2, mat1);
    else
        return MatmulMatrix(mat1, mat2);
}

// TODO: make a selector for proper algorithms
// TODO: For now just inverse the order: (a * (b * (c*d)))
template <typename TMat1, typename TMat2, typename... Types>
constexpr auto Matmul(const TMat1& mat1, const TMat2& mat2, const Types... others)
{
    // todo: use fold expression
    return Matmul(Matmul(mat1, mat2), others...);
}

} // namespace linalg
} // namespace vic