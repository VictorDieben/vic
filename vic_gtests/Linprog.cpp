
#include "gtest/gtest.h"

#include "vic/linprog/seidel.h"

using namespace vic::linprog;

template <typename T1, typename T2>
bool IterablesNear(const T1& iterable1, const T2& iterable2, const double eps = 1e-10)
{
    if(iterable1.size() != iterable2.size())
        return false;
    auto it1 = iterable1.begin();
    auto it2 = iterable2.begin();
    bool succes = true;
    for(; it1 < iterable1.end() && it2 < iterable2.end(); ++it1, ++it2)
        if(std::fabs((*it1) - (*it2)) > eps)
            succes = false;
    return succes;
}

TEST(Linprog, MinlMaxL)
{
    const double eps = 1e-15;
    ASSERT_FALSE(MinL(1., 1., 1., 1.));
    ASSERT_TRUE(MinL(1., 1. - eps, 1., 1.));
    ASSERT_TRUE(MinL(1. - eps, 1., 1., 1.));

    ASSERT_FALSE(MaxL(1., 1., 1., 1.));
    ASSERT_TRUE(MaxL(1., 1. + eps, 1., 1.));
    ASSERT_TRUE(MaxL(1. + eps, 1., 1., 1.));
}

TEST(Linprog, 1d)
{
    Seidel<double, 1> seidel{};
    // Tests with 1 constraint: 1*x < 1

    seidel.AddConstraint(Constraint<double, 1>({1}, 1));

    // maximize x
    SeidelResult<double, 1> res = seidel.Calculate({1});
    ASSERT_EQ(res.mState, ESeidelState::OK);
    ASSERT_TRUE(IterablesNear(res.mU, std::array{1.}));
    ASSERT_TRUE(IterablesNear(res.mW, std::array{0.}));

    // minimize x
    res = seidel.Calculate({-1});
    ASSERT_EQ(res.mState, ESeidelState::OK);
    ASSERT_TRUE(IterablesNear(res.mW, std::array{-1.}));

    // no objective
    res = seidel.Calculate({0});
    ASSERT_EQ(res.mState, ESeidelState::OK);
    ASSERT_TRUE(IterablesNear(res.mW, std::array{0.}));

    // infeasible
    seidel.AddConstraint(Constraint<double, 1>({-1}, -2));
    res = seidel.Calculate({1});
    ASSERT_EQ(res.mState, ESeidelState::INFEASIBLE);
}

TEST(Linprog, 2d)
{
    Seidel<double, 2> seidel{};
    seidel.AddConstraint(Constraint<double, 2>({1, 0}, 1));
    seidel.AddConstraint(Constraint<double, 2>({0, 1}, 1));
    auto res = seidel.Calculate({1, 1});
    ASSERT_EQ(res.mState, ESeidelState::OK);
    ASSERT_TRUE(IterablesNear(res.mU, std::array{1., 1.}));
    ASSERT_TRUE(IterablesNear(res.mW, std::array{0., 0.}));
}

TEST(Linprog, 2dUnbounded)
{
    // Test unbounded case
    Seidel<double, 2> seidel{};
    auto res = seidel.Calculate({1, 1});
    ASSERT_EQ(res.mState, ESeidelState::OK);
    ASSERT_TRUE(IterablesNear(res.mU, std::array{0., 0.}));
    ASSERT_TRUE(IterablesNear(res.mW, std::array{1., 1.}));

    seidel.AddConstraint(Constraint<double, 2>({1, 0}, 1));
    res = seidel.Calculate({1, 1});
    ASSERT_EQ(res.mState, ESeidelState::OK);
    ASSERT_TRUE(IterablesNear(res.mU, std::array{1., 0.}));
    ASSERT_TRUE(IterablesNear(res.mW, std::array{0., 1.}));
}

TEST(Linprog, 3d)
{
    Seidel<double, 3> seidel{};
    seidel.AddConstraint(Constraint<double, 3>({1, 0, 0}, 1));
    seidel.AddConstraint(Constraint<double, 3>({0, 2, 0}, 2));
    seidel.AddConstraint(Constraint<double, 3>({0, 0, 3}, 3));

    auto res = seidel.Calculate({1, 1, 1});
    ASSERT_EQ(res.mState, ESeidelState::OK);
    ASSERT_TRUE(IterablesNear(res.mU, std::array{1., 1., 1.}));
    ASSERT_TRUE(IterablesNear(res.mW, std::array{0., 0., 0.}));
}

TEST(Linprog, 4d)
{
    Seidel<double, 4> seidel{};
    seidel.AddConstraint(Constraint<double, 4>({1, 0, 0, 0}, 1));
    seidel.AddConstraint(Constraint<double, 4>({0, 1, 0, 0}, 2));
    seidel.AddConstraint(Constraint<double, 4>({0, 0, 1, 0}, 3));
    seidel.AddConstraint(Constraint<double, 4>({0, 0, 0, 1}, 4));

    auto res = seidel.Calculate({1, 1, 1, 1});
    ASSERT_EQ(res.mState, ESeidelState::OK);
    ASSERT_TRUE(IterablesNear(res.mU, std::array{1., 2., 3., 4.}));
    ASSERT_TRUE(IterablesNear(res.mW, std::array{0., 0., 0., 0.}));
}

TEST(Linprog, ManyConstraints)
{
    // Test with many constraints, all going through the same point
}

TEST(Linprog, UnitCircle)
{
    // Test with many constraints on the unit circle.
    // This makes it a lot more difficult to optimize,
    // because the active constraints will switch multiple times

    // release:
    // iters: 1000000;    constraints: 1000:      time: 51.3s
    // iters: 100000;     constraints: 10000:     time: 48.7s
    // iters: 10000;      constraints: 100000:    time: 52.0s

    Seidel<double, 2> seidel{};
    constexpr std::size_t n = 1000;
    for(std::size_t i = 0; i < n; ++i)
        seidel.AddConstraint(Constraint<double, 2>({std::sin(3.1415 * (double(i) / double(n - 1))), //
                                                    std::cos(3.1415 * (double(i) / double(n - 1)))},
                                                   1));
    for(std::size_t i = 0; i < 4; ++i)
    {
        auto res = seidel.Calculate({1, 1});
        ASSERT_EQ(res.mState, ESeidelState::OK);
        ASSERT_TRUE(IterablesNear(res.mU, std::array{std::sqrt(.5), std::sqrt(.5)}, 0.01));
        ASSERT_TRUE(IterablesNear(res.mW, std::array{0., 0.}));
    }
}

TEST(Linprog, Narrow)
{
    // Test with 2 constraints, intersecting at a distance
}
