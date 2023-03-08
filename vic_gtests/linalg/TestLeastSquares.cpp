
#include "pch.h"

#include "../test_base.h"

#include "vic/linalg/linalg.h"

#include <array>

#include <random>

namespace vic
{
namespace linalg
{

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

TEST(TestAlgorithms, LeastSquares)
{
    constexpr auto nPolynomials = 3;
    constexpr auto nMeasurements = 5;

    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(0., 1.);

    const auto c = std::array{dist(g), dist(g), dist(g), dist(g), dist(g), dist(g), dist(g), dist(g), dist(g), dist(g)};

    auto b = Vector<double>{nMeasurements, 1};
    auto mat = Matrix<double>{nMeasurements, nPolynomials};

    std::vector<double> xs{};
    for(uint32_t im = 0; im < nMeasurements; ++im)
        xs.push_back(dist(g));
    std::sort(xs.begin(), xs.end());

    for(uint32_t im = 0; im < nMeasurements; ++im)
    {
        const double x = xs.at(im);

        for(uint32_t ip = 0; ip < nPolynomials; ++ip)
            mat.At(im, ip) = Power<double>(x, ip);

        double sum = 0;
        for(uint32_t ip = 0; ip < nPolynomials; ++ip)
            sum += mat.Get(im, ip) * c.at(im);
        b.At(im, 0) = sum;
    }

    const auto cHat = LeastSquares(mat, b);

    // EXPECT_TRUE(IsEqual(c, cHat, 1e-6));
}

} // namespace linalg
} // namespace vic