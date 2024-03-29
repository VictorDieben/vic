#pragma once
#pragma once

#include "vic/linalg/matmul.h"
#include "vic/linalg/matrices.h"
#include "vic/linalg/tools.h"
#include "vic/linalg/traits.h"
#include "vic/utils.h"

#include <algorithm>

namespace vic
{
namespace linalg
{
namespace algorithms
{
// TODO: Eigenvalues / eigenvectors
// http://mathreview.uwaterloo.ca/archive/voli/1/panju.pdf

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
constexpr auto PowerMethod(const TMatrix& matrix, const double eps = 1E-10)
{
    assert(matrix.GetRows() == matrix.GetColumns());

    using SquareShape = SquareShape<TMatrix>;
    using DataType = typename TMatrix::DataType;
    using EigenVectorShape = Shape<SquareShape::rows, 1>;
    using EigenVectorType = Matrix<DataType, EigenVectorShape>;
    constexpr Row n = TMatrix::GetRows();

    // todo: good initial guess
    EigenVectorType v{matrix.GetRows(), 1u};

    auto normPrev = 0.;

    for(std::size_t i = 0; i < 10000; ++i)
    {
        const auto w = Matmul(matrix, v);

        //const auto norm = Norm(w);
        //v = Matmul(w, 1. / norm);
        v = Normalize(w);

        if(std::fabs(normPrev - norm) < eps)
            break; // check if change in ev is smaller than a certain limit
        normPrev = norm;
    }
    return v;
}

//
//
//

template <typename TMatrix>
requires ConceptSquareMatrix<TMatrix>
constexpr auto QRMethod(const TMatrix& matrix, const double eps = 1E-10)
{
    using DataType = typename TMatrix::DataType;
    constexpr std::size_t n = TMatrix::GetRows();

    Matrix<double, n, n> eigenVectors{};
    Diagonal<double, n, n> eigenValues{};

    return std::pair(eigenVectors, eigenValues); // todo: pass by reference
}

} // namespace algorithms
} // namespace linalg
} // namespace vic