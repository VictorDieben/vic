#pragma once

#include <array>
#include <cstddef>
#include <tuple>

namespace vic
{

// statemachine that checks state change at compile time,
// only needs to check if fromState is correct at runtime
template <typename TEnum, std::size_t size, std::array<std::pair<TEnum, TEnum>, size> validChanges>
class StateMachine
{
public:
    constexpr StateMachine() = default;
    constexpr StateMachine(const TEnum initialState)
        : mState(initialState)
    { }

    template <TEnum val>
    bool HasState() const
    {
        return mState == val;
    }

    TEnum GetState() const { return mState; }

    template <TEnum fromState, TEnum toState>
    constexpr static bool IsValid()
    {
        for(const auto& pair : validChanges)
            if(pair.first == fromState && pair.second == toState)
                return true;
        return false;
    }

    template <TEnum fromState, TEnum toState>
    void SetState()
    {
        static_assert(this->IsValid<fromState, toState>());
        assert(mState == fromState); // todo: assert or throw?
        //if(mState != fromState)
        //    throw std::runtime_error("");
        mState = toState;
    }

private:
    TEnum mState{};
};
} // namespace vic