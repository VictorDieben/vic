
#include <benchmark/benchmark.h>

#include "vic/linalg/linalg.h"

static void LinalgMatmul2x2d(benchmark::State& state)
{
    using namespace vic::linalg;

    const Matrix2<double> mat1{};
    const Matrix2<double> mat2{};

    for(auto _ : state)
    {
        const auto res = Matmul(mat1, mat2);
        benchmark::DoNotOptimize(res);
    }
}

static void LinalgMatmul3x3d(benchmark::State& state)
{
    using namespace vic::linalg;

    const Matrix3<double> mat1{};
    const Matrix3<double> mat2{};

    for(auto _ : state)
    {
        const auto res = Matmul(mat1, mat2);
        benchmark::DoNotOptimize(res);
    }
}

static void LinalgMatmul4x4f(benchmark::State& state)
{
    using namespace vic::linalg;

    static float val = 0.;

    for(auto _ : state)
    {
        Matrix4<float> mat1{val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++};
        Matrix4<float> mat2{val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++};

        const auto res = Matmul(mat1, mat2);
        benchmark::DoNotOptimize(res);
    }
}

static void LinalgMatmul4x4d(benchmark::State& state)
{
    using namespace vic::linalg;

    static double val = 0.;

    for(auto _ : state)
    {
        Matrix4<float> mat1{val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++};
        Matrix4<float> mat2{val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++, val++};

        const auto res = Matmul(mat1, mat2);
        benchmark::DoNotOptimize(res);
    }
}

static void LinalgMatmul4nxn4d(benchmark::State& state)
{
    // benchmark a 4x4 matrix multiplication, but hide the in-between dimension from the compiler (also 4)
    using namespace vic::linalg;

    static double val = 0.;

    for(auto _ : state)
    {
        Matrix<double, Shape<4, UnknownSize>> mat1{4, 4};
        auto* data = mat1.data();
        for(std::size_t i = 0; i < 4 * 4; ++i)
            data[i] = val++;

        Matrix<double, Shape<UnknownSize, 4>> mat2{4, 4};
        data = mat1.data();
        for(std::size_t i = 0; i < 4 * 4; ++i)
            data[i] = val++;

        const auto res = Matmul(mat1, mat2);
        benchmark::DoNotOptimize(res);
    }
}

static void LinalgMatmulNxNd(benchmark::State& state)
{
    // benchmark matrix multipication for a wide range of matrix sizes
    using namespace vic::linalg;

    const std::size_t size = state.range(0);

    for(auto _ : state)
    {

        Matrix<double, UnknownShape> mat1{Row(size), Col(size)};
        Matrix<double, UnknownShape> mat2{Row(size), Col(size)};

        const auto res = Matmul(mat1, mat2);
        benchmark::DoNotOptimize(res);
    }
}

BENCHMARK(LinalgMatmul2x2d);
BENCHMARK(LinalgMatmul3x3d);
BENCHMARK(LinalgMatmul4x4f);
BENCHMARK(LinalgMatmul4x4d);
BENCHMARK(LinalgMatmul4nxn4d);

BENCHMARK(LinalgMatmulNxNd)->DenseRange(4, 16, 4);