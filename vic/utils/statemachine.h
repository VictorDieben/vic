#pragma once

#include <array>
#include <cstddef>
#include <tuple>

#include "vic/utils/templates.h"

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

template <auto stateValue>
struct State
{
    using type = decltype(stateValue);
    static constexpr type value = stateValue;
};

template <typename fromState, typename toState>
struct Transition
{
    //static_assert(std::is_same_v<typename fromState::type, typename toState::type>);
    //using type = typename fromState::type;
    using from = fromState;
    using to = toState;
};

// type that contains the list of valid states
template <typename... States>
struct StateContainer
{
    constexpr static std::tuple<States...> types{};

    template <typename T>
    constexpr static bool Contains()
    {
        return templates::Contains<T, States...>();
    }
    // todo: make sure all states are compatible with each other
};

// type that contains the list of valid transitions
template <typename... Transitions>
struct TransitionContainer
{
    using TransitionsTuple = std::tuple<Transitions...>;
    constexpr static TransitionsTuple types{};

    template <typename T>
    constexpr static bool Contains()
    {
        return templates::Contains<T, Transitions...>();
    }
};

template <typename TStateContainer, typename TTransition>
constexpr bool ValidateTransition()
{
    constexpr static bool containsFrom = TStateContainer::template Contains<typename TTransition::from>();
    constexpr static bool containsTo = TStateContainer::template Contains<typename TTransition::to>();
    return containsFrom && containsTo;
}

template <typename TStateContainer, typename TTransitionContainer>
constexpr bool ValidateContainer()
{
    bool allTrue = true;
    using transitions = TTransitionContainer::TransitionsTuple;

    //auto res = std::apply(
    //    [](auto... x) {
    //        return std::make_tuple( //
    //            ValidateTransition<TStateContainer, decltype(x)>()...);
    //    },
    //    TTransitionContainer::types);
    return allTrue;
};

// Type that only contains the compile time info of a state machine
template <typename TStates, typename TTransitions>
struct StateMachineDescription
{
    // Make sure the states and transitions are compatible
    static_assert(ValidateContainer<TStates, TTransitions>() && "Invalid description; not all transitions are composed of existing states");

    using StatesContainer = TStates;
    using TransitionsContainer = TTransitions;
};

//

template <typename TStateMachine>
struct StateInstance
{
    using StateMachineType = TStateMachine;

    StateInstance(TStateMachine& machine)
        : mStateMachine(machine)
    { }

    template <typename TNewState>
    auto GoToState()
    {
        static_assert(true); // todo: static assert that the transition is valid
        assert(mValid); // make sure we didn't already move out of this state
        mValid = false;
        return StateInstance<TNewState>(mStateMachine);
    }

private:
    StateMachineType& mStateMachine;
    bool mValid{true};
};

//
template <typename TDescription>
struct NewStateMachine
{
public:
private:
};

} // namespace vic