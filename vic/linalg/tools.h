#pragma once


#include "vic/linalg/matrices.h"


namespace vic
{
namespace linalg
{


template <typename T>
class Bracket3
{
public:
    constexpr Bracket3(const Vector3<T>& vec)
        : mVector(vec) {}

    constexpr static size_t Rows = 3;
    constexpr static size_t Columns = 3;
    constexpr static std::size_t GetRows() { return Rows; }
    constexpr static std::size_t GetColumns() { return Columns; }

    using DataType = T;

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < Rows) && (j < Columns)));
        if (i == j)
            return T{ 0 };

        // No need to manually optimize, the compiler is better at it
        if (i == 1 && j == 0)
            return mVector.mData.at(2);
        if (i == 2 && j == 0)
            return -mVector.mData.at(1);
        if (i == 2 && j == 1)
            return mVector.mData.at(0);

        if (i == 0 && j == 1)
            return -mVector.mData.at(2);
        if (i == 0 && j == 2)
            return mVector.mData.at(1);
        if (i == 1 && j == 2)
            return -mVector.mData.at(0);

        assert(false);
        return T{ 0 };
    }

    Vector3<T> mVector{};
};

// trace is sum of all diagonal elements
template <typename TMat>
    requires ConceptSquareMatrix<TMat>
constexpr auto Trace(const TMat& matrix)
{
    static_assert(IsSquare<TMat>::value);
    typename TMat::DataType val{ 0 };
    for (std::size_t i = 0; i < TMat::Rows; ++i)
        val += matrix.Get(i, i);
    return val;
}

// 2d cross product
template <typename T>
    requires ConceptVector<T> && (T::GetRows() == 2)
constexpr auto Cross(const T& vec1, const T& vec2)
{
    return (vec1.Get(0, 0) * vec2.Get(1, 0)) - (vec1.Get(1, 0) * vec2.Get(0, 0));
}

// 3d cross product
template <typename T>
    requires ConceptVector<T> && (T::GetRows() == 3)
constexpr auto Cross(const Vector3<T>& vec1, const Vector3<T>& vec2)
{
    const double ax = vec1.Get(0, 0), ay = vec1.Get(1, 0), az = vec1.Get(1, 0);
    const double bx = vec2.Get(0, 0), by = vec2.Get(1, 0), bz = vec2.Get(1, 0);
    return Vector3<double>({ (ay * bz) - (az * by),			//
                             (az * bx) - (ax * bz),			//
                             (ax * by) - (ay * bx) });
}

// dot product between two vectors
template <typename TMat1, typename TMat2>
    requires ConceptVector<TMat1> && ConceptVector<TMat2> && HasSameShape<TMat1, TMat2>::value
constexpr auto Dot(const TMat1& mat1, const TMat2& mat2)
{
    static_assert(TMat1::Columns == 1 && TMat2::Columns == 1 && TMat1::Rows == TMat2::Rows);
    using TRet =  decltype((typename TMat1::DatType{}) *(typename TMat2::DataType{}));
    TRet val{ 0 };
    for (std::size_t i = 0; i < TMat1::Rows; ++i)
        val += (mat1.Get(i, 0) * mat2.Get(i, 0));
    return val;
}

}
}