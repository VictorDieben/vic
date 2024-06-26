
#include "gtest/gtest.h"

#include "test_base.h"

#include "vic/symbolic/symbolic.h"

using namespace vic::symbolic;

TEST(Symbolic, Initialize)
{
    auto var1 = Variable("var1");
    auto var2 = Variable("var2");

    auto add = Add(var1, var2);

    //var1.SetName("x");
    //EXPECT_EQ(var1.GetName(), "x");
    //EXPECT_EQ(var1.GetType(), EType::Var);

    //auto const2 = Constant(2);
    //var1.SetName("const2");
    //EXPECT_EQ(const2.GetName(), "const2");
    //EXPECT_EQ(const2.GetType(), EType::Const);

    //auto varSquared = Power(var1, const2);
    //auto varCubed = Power(var1, Constant(3));

    //auto varFrac = Fraction(varCubed, varSquared);

    //std::cout << varFrac << std::endl;

    //EXPECT_TRUE(false);
}
//
//TEST(Symbolic, Equivalent)
//{
//    Symbol statement1 = Power{Variable{"x"}, Constant{2}}; // x^2
//    Symbol statement2 = Multiply{Variable{"x"}, Variable{"x"}}; // x*x
//    EXPECT_TRUE(Equivalent(statement1, statement2));
//
//    statement1 = Multiply{Variable{"x"}, Constant{2}}; // x*2
//    statement2 = Sum{Variable{"x"}, Variable{"x"}}; // x+x
//    EXPECT_TRUE(Equivalent(statement1, statement2));
//}
//
//TEST(Symbolic, Simplify)
//{
//    Symbol x = Variable{"x"};
//    Symbol oneOverX = Power{x, Constant{-1}};
//    Symbol symbol = Multiply{{Constant{2}, x, oneOverX}}; // 2*x/x
//
//    Symbol simplified = Simplify(symbol);
//    EXPECT_TRUE(Equivalent(simplified, Constant{2}));
//
//    Symbol FourX = Sum{Sum{x, x}, Sum{x, x}};
//    Symbol simplifiedFourX = Simplify(FourX);
//    EXPECT_TRUE(Equivalent(simplifiedFourX, Multiply{{Constant{4}, x}}));
//}
