#pragma once

#include "vic/linalg/matrices/identity.h"
//
#include "vic/linalg/algorithms/add.h"
#include "vic/linalg/algorithms/matmul.h"

#include "vic/utils.h"

namespace vic
{
namespace linalg
{

//// Bracket operator for vec3s.
//// (matrix form of cross product)
//// Should have the following shape:
//// |x1|   | 0  -x3  x2|
//// |x2| = | x3  0  -x1|
//// |x3|   |-x2  x1  0 |
template <typename TVec>
requires ConceptVector<TVec>
class Bracket3
{
public:
    using DataType = TVec::DataType;
    using ShapeType = Shape<3, 3>;

    constexpr Bracket3() = default;
    constexpr Bracket3(const TVec& vec)
        : mVector(vec)
    {
        assert(vec.GetRows() == 3);
    }

    constexpr static Row GetRows() { return ShapeType::rows; }
    constexpr static Col GetColumns() { return ShapeType::cols; }

    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Full;

    constexpr DataType Get(const std::size_t i, const std::size_t j)
    {
        assert(i < GetRows() && j < GetColumns());
        if(i == j)
            return DataType{0};

        // No need to manually optimize, the compiler is better at it
        if(i == 1 && j == 0)
            return mVector.mData.at(2);
        if(i == 2 && j == 0)
            return -mVector.mData.at(1);
        if(i == 2 && j == 1)
            return mVector.mData.at(0);

        if(i == 0 && j == 1)
            return -mVector.mData.at(2);
        if(i == 0 && j == 2)
            return mVector.mData.at(1);
        if(i == 1 && j == 2)
            return -mVector.mData.at(0);

        assert(false); // should be unreachable
        return DataType{0};
    }

    TVec mVector{};
};

// trace is sum of all diagonal elements
template <typename TMat>
requires ConceptSquareMatrix<TMat>
constexpr auto Trace(const TMat& matrix)
{
    assert(matrix.GetRows() == matrix.GetColumns());
    typename TMat::DataType val{0};
    for(MatrixSize i = 0; i < Min(matrix.GetRows(), matrix.GetColumns()); ++i)
        val += matrix.Get(i, i);
    return val;
}

// 2d cross product
template <typename TVec>
requires ConceptVector<TVec> &&(TVec::GetRows() == 2) //
    constexpr auto Cross(const TVec& vec1, const TVec& vec2)
{
    return (vec1.Get(0, 0) * vec2.Get(1, 0)) - (vec1.Get(1, 0) * vec2.Get(0, 0));
}

template <typename T>
requires ConceptVector<T>
constexpr auto SquaredNorm(const T& vec)
{
    typename T::DataType sum = 0;
    for(MatrixSize i = 0; i < vec.GetRows(); ++i)
        sum += (vec.Get(i, 0) * vec.Get(i, 0));
    return sum;
}

template <typename T>
requires ConceptVector<T>
constexpr auto Norm(const T& vec)
{
    typename T::DataType sum = 0.;
    for(MatrixSize i = 0; i < vec.GetRows(); ++i)
        sum += (vec.Get(i, 0) * vec.Get(i, 0));
    return ::vic::Sqrt(sum);
}

template <typename TVec>
requires ConceptVector<TVec>
constexpr auto Normalize(const TVec& vec)
{
    TVec res{vec.GetRows(), vec.GetColumns()};
    const auto oneOverNorm = 1. / Norm(vec);
    for(MatrixSize i = 0; i < vec.GetRows(); ++i)
        res.At(i, 0) = vec.Get(i, 0) * oneOverNorm;
    return res;
}

// 3d cross product
template <typename TVec1, typename TVec2>
requires ConceptVector<TVec1> && ConceptVector<TVec2>
constexpr auto Cross(const TVec1& vec1, const TVec2& vec2)
{
    assert(vec1.GetRows() == 3 && vec2.GetRows() == 3);
    const double ax = vec1.Get(0), ay = vec1.Get(1), az = vec1.Get(2);
    const double bx = vec2.Get(0), by = vec2.Get(1), bz = vec2.Get(2);
    return Vector3<double>({(ay * bz) - (az * by), //
                            (az * bx) - (ax * bz), //
                            (ax * by) - (ay * bx)});
}

//// dot product between two vectors
//template <typename TMat1, typename TMat2> // requires ConceptVector<TMat1> && ConceptVector<TMat2> && is_same_shape<TMat1, TMat2>::value //
//constexpr auto Dot(const TMat1& mat1, const TMat2& mat2)
//{
//    static_assert(TMat1::GetColumns() == 1 && TMat2::GetColumns() == 1 && TMat1::GetRows() == TMat2::GetRows());
//    using TRet = decltype((typename TMat1::DataType{}) * (typename TMat2::DataType{}));
//    TRet val{0};
//    for(std::size_t i = 0; i < TMat1::GetRows(); ++i)
//        val += (mat1.Get(i, 0) * mat2.Get(i, 0));
//    return val;
//}
//
//// todo: TMatTarget needs to be assignable
//template <typename TMatTarget, typename TMatSource>
//void Assign(TMatTarget& target, //
//            const TMatSource& source,
//            const std::size_t row,
//            const std::size_t col)
//{
//    assert(target.GetRows() >= source.GetRows() + row);
//    assert(target.GetColumns() >= source.GetColumns() + col);
//
//    for(std::size_t i = 0; i < source.GetRows(); ++i)
//    {
//        for(std::size_t j = 0; j < source.GetColumns(); ++j)
//        {
//            target.At(row + i, col + j) = source.At(i, j);
//        }
//    }
//}
//
//// todo: TMatTarget needs to be asignable
//template <std::size_t row, std::size_t col, typename TMatTarget, typename TMatSource>
//void Assign(TMatTarget& target, const TMatSource& source)
//{
//    static_assert(target.GetRows() >= source.GetRows() + row);
//    static_assert(target.GetColumns() >= source.GetColumns() + col);
//
//    for(std::size_t i = 0; i < source.GetRows(); ++i)
//    {
//        for(std::size_t j = 0; j < source.GetColumns(); ++j)
//        {
//            target.At(row + i, col + j) = source.Get(i, j);
//        }
//    }
//}
//
//// todo: TMatResult needs to have a static size, assignable
//template <typename TMatResult, std::size_t row, std::size_t col, typename TMatInput>
//TMatResult Extract(const TMatInput& source)
//{
//    static_assert(source.GetRows() >= TMatResult::GetRows() + row);
//    static_assert(source.GetColumns() >= TMatResult::GetColumns() + col);
//
//    TMatResult res{};
//    for(std::size_t i = 0; i < TMatResult::GetRows(); ++i)
//        for(std::size_t j = 0; j < TMatResult::GetColumns(); ++j)
//            res.At(i, j) = source.Get(i + row, j + col);
//
//    return res;
//}
//
//template <typename TMatResult, typename TMatInput>
//TMatResult Extract(const TMatInput& source, std::size_t row, std::size_t col)
//{
//    assert(source.GetRows() >= TMatResult::GetRows() + row);
//    assert(source.GetColumns() >= TMatResult::GetColumns() + col);
//
//    TMatResult res{};
//    for(std::size_t i = 0; i < TMatResult::GetRows(); ++i)
//        for(std::size_t j = 0; j < TMatResult::GetColumns(); ++j)
//            res.At(i, j) = source.Get(i + row, j + col);
//
//    return res; // todo
//}
//
// mostly used for tests, but also useful outside of it
template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto IsEqual(const TMat1& mat1, const TMat2& mat2, const double eps = 1e-10)
{
    if((mat1.GetRows() != mat2.GetRows()) || (mat1.GetColumns() != mat2.GetColumns()))
        return false;
    for(Row i = 0; i < mat1.GetRows(); ++i)
        for(Col j = 0; j < mat1.GetColumns(); ++j)
            if(std::fabs(mat1.Get(i, j) - mat2.Get(i, j)) > eps)
                return false;
    return true;
}

// Verify that a matrix is orthogonal (e.g. A.T*A == I)
template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto IsOrthogonal(const TMat& mat, const double eps = 1e-10)
{
    if(mat.GetRows() != mat.GetColumns())
        return false;
    Identity<typename TMat::DataType, UnknownShape> identity{mat.GetRows(), mat.GetColumns()};
    return IsEqual(Matmul(Transpose(mat), mat), identity, eps);
}

template <typename TMat>
//requires ConceptMatrix<TMat>
constexpr auto Negative(const TMat& mat)
{
    return Matmul(-1., mat);
}

// 3d cross product
template <typename TMat1, typename TMat2>
//requires ConceptVector<TMat1> && ConceptVector<TMat2>
constexpr auto Subtract(const TMat1& mat1, const TMat2& mat2)
{
    return Add(mat1, Negative(mat2));
}

} // namespace linalg
} // namespace vic