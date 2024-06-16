#pragma once

#include <array>

template <uint64_t Tsize>
class Memory
{
public:
    Memory() = default;

    using TStack = uint8_t;

private:
    std::array<void, Tsize> mData;
};

template <typename Tcode = uint8_t, typename Targ = uint8_t>
struct Instruction
{
    Tcode mCode;
    Targ mArg1;
    Targ mArg2;

    //
    using code_t = Tcode;
    using arg_t = Targ;
};

template <typename Tinstruction>
class Program
{
public:
    Program() = default;
    std::vector<Tinstruction> mInstructions{};

    using instr_t = Tinstruction;
    using instr_ptr_t = size_t;
};

template <typename Treg = uint8_t, uint64_t regsize = 8>
class Register
{
public:
    Register() = default;
    std::array<Treg, regsize> mReg;
};

template <typename TMem, typename TRegister, typename TProg>
class Emulator
{
public:
    Emulator(TMem& mem)
        : mMemory(mem)
    { }

    void Run(const TProg& prog) { }

private:
    TMem& mMemory;
    TRegister mReg;

    // TMem::TStack mStackPtr;
    // TProg::instr_ptr_t mInstrPtr;
};