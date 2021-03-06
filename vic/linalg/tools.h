#pragma once

#include "vic/linalg/matrices.h"

#include "vic/utils.h"

namespace vic
{
namespace linalg
{

// Bracket operator for vec3s.
// (matrix form of cross product)
// Should have the following shape:
// |x1|   | 0  -x3  x2|
// |x2| = | x3  0  -x1|
// |x3|   |-x2  x1  0 |
template <typename T>
class Bracket3
{
public:
    constexpr Bracket3(const Vector3<T>& vec)
        : mVector(vec)
    { }

    constexpr static std::size_t Rows = 3;
    constexpr static std::size_t Columns = 3;
    constexpr static std::size_t GetRows() { return Rows; }
    constexpr static std::size_t GetColumns() { return Columns; }

    using DataType = T;

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < Rows) && (j < Columns)));
        if(i == j)
            return T{0};

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
        return T{0};
    }

    Vector3<T> mVector{};
};

// trace is sum of all diagonal elements
template <typename TMat> // requires ConceptSquareMatrix<TMat>
constexpr auto Trace(const TMat& matrix)
{
    static_assert(IsSquare<TMat>::value);
    typename TMat::DataType val{0};
    for(std::size_t i = 0; i < TMat::Rows; ++i)
        val += matrix.Get(i, i);
    return val;
}

// 2d cross product
template <typename T> // requires ConceptVector<T> &&(T::GetRows() == 2) //
constexpr auto Cross(const T& vec1, const T& vec2)
{
    return (vec1.Get(0, 0) * vec2.Get(1, 0)) - (vec1.Get(1, 0) * vec2.Get(0, 0));
}

template <typename T> // requires ConceptVector<T>
constexpr auto SquaredNorm(const T& vec)
{
    typename T::DataType sum = 0;
    for(std::size_t i = 0; i < T::GetRows(); ++i)
    {
        sum += (vec.Get(i, 0) * vec.Get(i, 0));
    }
    return sum;
}

template <typename T> // requires ConceptVector<T>
constexpr auto Norm(const T& vec)
{
    typename T::DataType sum = 0;
    for(std::size_t i = 0; i < T::GetRows(); ++i)
    {
        sum += (vec.Get(i, 0) * vec.Get(i, 0));
    }
    return ::vic::Sqrt(sum);
}

template <typename T> // requires ConceptVector<T>
constexpr auto Normalize(const T& vec)
{
    T res{};
    const auto oneOverNorm = 1. / Norm(vec);
    for(std::size_t i = 0; i < T::GetRows(); ++i)
        res.At(i, 0) = vec.Get(i, 0) * oneOverNorm;
    return res;
}

// 3d cross product
template <typename T> // requires ConceptVector<T> &&(T::GetRows() == 3) //
constexpr auto Cross(const Vector3<T>& vec1, const Vector3<T>& vec2)
{
    const double ax = vec1.Get(0), ay = vec1.Get(1), az = vec1.Get(2);
    const double bx = vec2.Get(0), by = vec2.Get(1), bz = vec2.Get(2);
    return Vector3<double>({(ay * bz) - (az * by), //
                            (az * bx) - (ax * bz), //
                            (ax * by) - (ay * bx)});
}

// dot product between two vectors
template <typename TMat1, typename TMat2> // requires ConceptVector<TMat1> && ConceptVector<TMat2> && HasSameShape<TMat1, TMat2>::value //
constexpr auto Dot(const TMat1& mat1, const TMat2& mat2)
{
    static_assert(TMat1::GetColumns() == 1 && TMat2::GetColumns() == 1 && TMat1::GetRows() == TMat2::GetRows());
    using TRet = decltype((typename TMat1::DataType{}) * (typename TMat2::DataType{}));
    TRet val{0};
    for(std::size_t i = 0; i < TMat1::GetRows(); ++i)
        val += (mat1.Get(i, 0) * mat2.Get(i, 0));
    return val;
}

// todo: TMatTarget needs to be asignable
template <typename TMatTarget, typename TMatSource>
constexpr void Assign(TMatTarget& target,
                      const TMatSource& source, //
                      const std::size_t row,
                      const std::size_t col)
{
    assert(target.GetRows() >= source.GetRows() + row);
    assert(target.GetColumns() >= source.GetColumns() + col);

    for(std::size_t i = 0; i < source.GetRows(); ++i)
    {
        for(std::size_t j = 0; j < source.GetColumns(); ++j)
        {
            target.At(row + i, col + j) = source.At(i, j);
        }
    }
}

// todo: TMatTarget needs to be asignable
template <std::size_t row, std::size_t col, typename TMatTarget, typename TMatSource>
constexpr void Assign(TMatTarget& target, const TMatSource& source)
{
    static_assert(target.GetRows() >= source.GetRows() + row);
    static_assert(target.GetColumns() >= source.GetColumns() + col);

    for(std::size_t i = 0; i < source.GetRows(); ++i)
    {
        for(std::size_t j = 0; j < source.GetColumns(); ++j)
        {
            target.At(row + i, col + j) = source.Get(i, j);
        }
    }
}

// todo: TMatResult needs to have a static size, assignable
template <typename TMatResult, std::size_t row, std::size_t col, typename TMatInput>
constexpr TMatResult Extract(const TMatInput& source)
{
    static_assert(source.GetRows() >= TMatResult::GetRows() + row);
    static_assert(source.GetColumns() >= TMatResult::GetColumns() + col);

    TMatResult res{};
    for(std::size_t i = 0; i < TMatResult::GetRows(); ++i)
        for(std::size_t j = 0; j < TMatResult::GetColumns(); ++j)
            res.At(i, j) = source.Get(i + row, j + col);

    return res;
}

template <typename TMatResult, typename TMatInput>
constexpr TMatResult Extract(const TMatInput& source, std::size_t row, std::size_t col)
{
    assert(source.GetRows() >= TMatResult::GetRows() + row);
    assert(source.GetColumns() >= TMatResult::GetColumns() + col);

    TMatResult res{};
    for(std::size_t i = 0; i < TMatResult::GetRows(); ++i)
        for(std::size_t j = 0; j < TMatResult::GetColumns(); ++j)
            res.At(i, j) = source.Get(i + row, j + col);

    return res; // todo
}

// mostly used for tests, but also useful outside of it
template <typename TMat1, typename TMat2> // requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto IsEqual(const TMat1& mat1, const TMat2& mat2, const double eps = 1e-10)
{
    if((mat1.GetRows() != mat2.GetRows()) || (mat1.GetColumns() != mat2.GetColumns()))
        return false;
    for(std::size_t i = 0; i < mat1.GetRows(); ++i)
        for(std::size_t j = 0; j < mat1.GetColumns(); ++j)
            if(std::fabs(mat1.Get(i, j) - mat2.Get(i, j)) > eps)
                return false;
    return true;
}

// Verify that a matrix is orthogonal (e.g. A.T*A == I)
template <typename TMat> // requires ConceptMatrix<TMat>
constexpr auto IsOrthogonal(const TMat& mat, const double eps = 1e-10)
{
    return IsEqual( //
        Matmul(Transpose(mat), mat), //
        Identity<typename TMat::DataType, TMat::GetRows()>{}, //
        eps);
}

template <typename TMat> // requires ConceptMatrix<TMat>
constexpr auto Negative(const TMat& mat)
{
    TMat res{};
    for(std::size_t i = 0; i < mat.GetRows(); ++i)
        for(std::size_t j = 0; j < mat.GetColumns(); ++j)
            res.At(i, j) = -mat.Get(i, j);
    return res;
}

// 3d cross product
template <typename TMat> // requires ConceptVector<TMat>
constexpr auto Subtract(const TMat& mat1, const TMat& mat2) // calculates mat1-mat2
{
    TMat res{};
    for(std::size_t i = 0; i < mat1.GetRows(); ++i)
        for(std::size_t j = 0; j < mat1.GetColumns(); ++j)
            res.At(i, j) = mat1.Get(i, j) - mat2.Get(i, j);
    return res;
}

} // namespace linalg
} // namespace vic