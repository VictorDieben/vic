#pragma once

#include <string>

namespace vic
{
namespace symbolic
{

// This file is a first attempt at creating a symbolic logic solver in c++.

struct Symbol
{
public:
    Symbol() = default;
    Symbol(std::string name)
        : mName(name)
    { }
    virtual ~Symbol() = default;

private:
    std::string mName{};
};

// Constants are building blocks for values.
// Rationals can be constructed as Multiply(nom, Power(denom, -1)
// Irrational numbers will need a special object
struct Constant : public Symbol
{
    Constant() = default;
    Constant(int val)
        : mValue(val)
    { }

private:
    int mValue{}; // todo: some kind of infinite number container
};

struct Variable : public Symbol
{
    Variable() = default;
    Variable(std::string name)
        : Symbol(name)
    { }
};

struct Sum : public Symbol
{
    Sum(Symbol s1, Symbol s2)
        : mTerms({s1, s2})
    { }
    Sum(std::vector<Symbol> symbols)
        : mTerms(symbols)
    { }

private:
    std::vector<Symbol> mTerms{};
};

struct Multiply : public Symbol
{
    Multiply(Symbol s1, Symbol s2)
        : mTerms({s1, s2})
    { }
    Multiply(std::vector<Symbol> symbols)
        : mTerms(symbols)
    {
        assert(symbols.size() > 1);
    }

private:
    std::vector<Symbol> mTerms{};
};

struct Power : public Symbol
{
    Power(Symbol term, Symbol exponent)
        : mTerm(term)
        , mExp(exponent)
    { }

private:
    Symbol mTerm;
    Symbol mExp;
};

Symbol Rational(Symbol nominator, Symbol denominator)
{
    return Multiply{nominator, Power{denominator, Constant{-1}}}; //
}

Symbol Simplify(Symbol symbol)
{
    return symbol; // todo
}

bool Equivalent(Symbol symbol1, Symbol simbol2)
{
    return false; // todo: verify that two statements are equivalent
}

} // namespace symbolic
} // namespace vic