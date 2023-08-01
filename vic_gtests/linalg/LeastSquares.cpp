
#include "gtest/gtest.h"

#include "../test_base.h"

#include "vic/linalg/linalg.h"

#include <array>

#include <random>

using namespace vic::linalg;

// todo: move to util or something
template <typename T, std::size_t N>
struct Polynomial
{
    // static_assert(N > 0 && "need at least 1 input");
    Polynomial()
        : mCoefficients{{}}
    { }

    Polynomial(const std::array<T, N>& input)
        : mCoefficients(input)
    { }

    T Eval(const T x) const
    {
        T res{};
        // todo: replace with "that one" algorithm for evaluating nth order polynomials
        T pow_x_n = 1.;
        for(uint32_t i = 0; i < N; ++i)
            res += pow_x_n * mCoefficients.at(i);
        return res; //
    }

    template <std::size_t derivation>
    T Derivative() const
    {
        if constexpr(derivation > N)
            return 0.;
        return 0.; // todo
    }

    std::array<T, N> Derivatives() const
    {
        return {}; // todo
    }

private:
    std::array<T, N> mCoefficients;
};

TEST(Algorithms, LeastSquares)
{
    constexpr auto nPolynomials = 10;
    constexpr auto nMeasurements = 30;

    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(0., 1.);

    auto c = Vector<double>{nPolynomials, 1};
    for(Row i = 0; i < c.GetRows(); ++i)
        c.At(i, 0) = dist(g);

    auto A = Matrix<double>{nMeasurements, nPolynomials};

    auto b = Vector<double>{nMeasurements, 1};

    std::vector<double> mx{};
    for(uint32_t im = 0; im < nMeasurements; ++im)
        mx.push_back(dist(g));
    std::sort(mx.begin(), mx.end());

    for(uint32_t im = 0; im < nMeasurements; ++im)
    {
        const double mi = mx.at(im);

        for(uint32_t ip = 0; ip < nPolynomials; ++ip)
            A.At(im, ip) = Power<double>(mi, ip);

        double sum = 0;
        for(uint32_t ip = 0; ip < nPolynomials; ++ip)
            sum += A.Get(im, ip) * c.At(ip, 0);

        b.At(im, 0) = sum;
    }

    const auto xHat = LeastSquares(A, b);
    const auto bHat = Matmul(A, xHat);
    EXPECT_TRUE(IsEqual(b, bHat, 1e-6));
}

TEST(Algorithms, LeastSquaresSimple)
{
    auto b = Vector3<double>{{6., 0., 0.}};
    auto mat = Matrix<double, Shape<3, 2>>{{0., 1., 1., 1., 2., 1.}};

    const auto xHat = LeastSquares(mat, b);

    EXPECT_TRUE(IsEqual(xHat, Vector2<double>{{-3., 5.}}, 1e-10));
}