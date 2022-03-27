#include "pch.h"

#include "vic/linprog/seidel.h"

namespace vic
{
namespace linprog
{

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

TEST(TestLinprog, MinlMaxL)
{
    const double eps = 1e-15;
    ASSERT_FALSE(MinL(1., 1., 1., 1.));
    ASSERT_TRUE(MinL(1., 1. - eps, 1., 1.));
    ASSERT_TRUE(MinL(1. - eps, 1., 1., 1.));

    ASSERT_FALSE(MaxL(1., 1., 1., 1.));
    ASSERT_TRUE(MaxL(1., 1. + eps, 1., 1.));
    ASSERT_TRUE(MaxL(1. + eps, 1., 1., 1.));
}

TEST(TestLinprog, 1d)
{
    Seidel<double, 1> seidel{};
    // Tests with 1 constraint: 1*x < 1

    seidel.Reset();
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
}

TEST(TestLinprog, 2d)
{
    Seidel<double, 2> seidel{};
    seidel.AddConstraint(Constraint<double, 2>({1, 0}, 1));
    seidel.AddConstraint(Constraint<double, 2>({0, 1}, 1));
    auto res = seidel.Calculate({1, 1});
    //ASSERT_EQ(res.mState, ESeidelState::OK);
    //ASSERT_TRUE(IterablesNear(res.mU, std::array{1., 1.}));
    //ASSERT_TRUE(IterablesNear(res.mW, std::array{0., 0.}));
}

TEST(TestLinprog, 3d)
{
    Seidel<double, 3> seidel{};
    seidel.AddConstraint(Constraint<double, 3>({1, 0, 0}, 1));
    seidel.AddConstraint(Constraint<double, 3>({0, 1, 0}, 1));
    seidel.AddConstraint(Constraint<double, 3>({0, 0, 1}, 1));

    auto res = seidel.Calculate({1, 1, 1});
    //ASSERT_EQ(res.mState, ESeidelState::OK);
    //ASSERT_TRUE(IterablesNear(res.mU, std::array{1., 1., 1.}));
    //ASSERT_TRUE(IterablesNear(res.mW, std::array{0., 0., 0.}));
}

TEST(TestLinprog, 4d)
{
    Seidel<double, 4> seidel{};
    seidel.AddConstraint(Constraint<double, 4>({1, 0, 0, 0}, 1));
    seidel.AddConstraint(Constraint<double, 4>({0, 1, 0, 0}, 1));
    seidel.AddConstraint(Constraint<double, 4>({0, 0, 1, 0}, 1));
    seidel.AddConstraint(Constraint<double, 4>({0, 0, 0, 1}, 1));

    auto res = seidel.Calculate({1, 1, 1, 1});
    //ASSERT_EQ(res.mState, ESeidelState::OK);
    //ASSERT_TRUE(IterablesNear(res.mU, std::array{1., 1., 1., 1.}));
    //ASSERT_TRUE(IterablesNear(res.mW, std::array{0., 0., 0., 0.}));
}

} // namespace linprog
} // namespace vic