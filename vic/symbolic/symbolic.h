#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <variant>

#include <iostream>
namespace vic
{
namespace symbolic
{

enum class EType
{
    Const,
    Var,
    Add,
    Multiply,
    Power,
    Equal
};

class Symbol;

class BaseSymbol final
{
public:
    void SetName(std::string_view name) { mName = name; }

    EType GetType() const { return mType; }
    const std::string& GetName() const { return mName; }

    //protected:
    //    friend class Symbol;
    BaseSymbol(EType type)
        : mType(type)
    { }
    BaseSymbol(EType type, std::shared_ptr<BaseSymbol> arg)
        : mType(type)
        , mArguments{arg}
    { }
    BaseSymbol(EType type,
               std::shared_ptr<BaseSymbol> arg1, //
               std::shared_ptr<BaseSymbol> arg2)
        : mType(type)
        , mArguments{{arg1, arg2}}
    { }

    const BaseSymbol& FirstArg() const { return *mArguments.at(0); }
    const BaseSymbol& SecondArg() const { return *mArguments.at(1); }
    const BaseSymbol& NthArg(const std::size_t i) const { return *mArguments.at(i); }

private:
    EType mType;
    std::string mName{"<empty name>"};

    std::vector<std::shared_ptr<BaseSymbol>> mArguments;
};

// symbol is effectively a wrapper around shared_ptr to BaseSymbol
class Symbol
{
public:
    Symbol(EType type)
        : mData(std::make_shared<BaseSymbol>(type))
    {
        assert(type == EType::Var);
    }
    Symbol(EType type, const Symbol& arg)
        : mData(std::make_shared<BaseSymbol>(type, arg.Get()))
    {
        assert(type == EType::Const);
    }
    Symbol(EType type, const Symbol& arg1, const Symbol& arg2)
        : mData(std::make_shared<BaseSymbol>(type, arg1.Get(), arg2.Get()))
    {
        assert(type != EType::Const && type != EType::Var);
    }
    // todo: more than 2 arguments?

    void SetName(std::string_view name) { mData->SetName(name); }

    EType GetType() const { return mData->GetType(); }
    const std::string& GetName() const { return mData->GetName(); }

    std::shared_ptr<BaseSymbol> Get() const { return mData; }

private:
    std::shared_ptr<BaseSymbol> mData;
};

Symbol Variable(std::string_view name)
{
    Symbol symbol{EType::Var};
    symbol.SetName(name);
    return symbol;
}

Symbol Add(const Symbol& symbol1, const Symbol& symbol2)
{
    Symbol symbol{EType::Add, symbol1, symbol2};

    return symbol;
}

std::ostream& operator<<(std::ostream& os, const BaseSymbol& symbol)
{
    switch(symbol.GetType())
    {
    case EType::Add:
        os << "(" << symbol.FirstArg() << " + " << symbol.SecondArg();
    case EType::Const:
        os << symbol.FirstArg();
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const Symbol& symbol)
{
    os << symbol.Get();
    return os;
}

std::string Represent(const Symbol& symbol)
{
    return ""; //
}

} // namespace symbolic
} // namespace vic

//} // namespace symbolic
//} // namespace vic