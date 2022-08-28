#pragma once

#include <string>
#include <string_view>
#include <variant>

namespace vic
{
namespace symbolic
{

// This file is a first attempt at creating a symbolic logic solver in c++.

enum class EType
{
    Const,
    Var,
    Plus,
    Multiply,
    Power,
    Equal
};

namespace Symbols
{
struct Symbol;

struct SymConst
{
    int val;
};
struct SymVar
{
    //
};
struct SymPlus
{
    Symbol left;
    Symbol right;
};
struct SymMult
{
    Symbol left;
    Symbol right;
};
struct SymPow
{
    Symbol base;
    Symbol exp;
};
struct SymEq
{
    Symbol left;
    Symbol right;
};
} // namespace Symbols

using SymbolVariant = std::variant<Symbols::SymConst, Symbols::SymVar, Symbols::SymPlus, Symbols::SymMult, Symbols::SymPow, Symbols::SymEq>;

// wrapper for Symbol variants
struct Symbol
{
public:
    Symbol() = default;
    Symbol(EType type) { }
    Symbol(EType type, Symbol left, Symbol Right)
        : mType(type)
        , mData()
    { }

    void SetName(std::string_view name) { mName = std::string(name); }
    const std::string& GetName() const { return mName; }

    EType GetType() const { return mType; }

private:
    SymbolVariant mData;
    EType mType{};
    std::string mName = "<Symbol>";
};

Symbol Constant(int val) { return Symbol{EType::Const}; }

Symbol Variable() { return Symbol{EType::Var}; }

Symbol Multiply(Symbol left, Symbol right) { return Symbol{}; }

Symbol Power(Symbol base, Symbol exp)
{
    return Symbol{EType::Power, base, exp}; //
}

Symbol Fraction(Symbol nominator, Symbol denominator)
{
    return Multiply(nominator, Power(denominator, Constant(-1))); //
}

Symbol Simplify(Symbol symbol)
{
    return symbol; // todo
}

bool Equivalent(Symbol symbol1, Symbol simbol2)
{
    return false; // todo: verify that two statements are equivalent
}

constexpr auto ToString(const EType type)
{
    if(type == EType::Const)
        return "Const";
    if(type == EType::Var)
        return "Var";
    if(type == EType::Plus)
        return "Plus";
    if(type == EType::Multiply)
        return "Multiply";
    if(type == EType::Power)
        return "Power";
    if(type == EType::Equal)
        return "Equal";
    return "Error";
}

std::ostream& operator<<(std::ostream& os, const EType& type)
{
    os << ToString(type);
    return os;
}

std::ostream& operator<<(std::ostream& os, const Symbol& symbol) { os << symbol.GetType() << "<" << symbol return os; }

} // namespace symbolic
} // namespace vic