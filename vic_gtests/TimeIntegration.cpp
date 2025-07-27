
#include <gtest/gtest.h>

#include "vic/linalg/linalg.h"

#include "vic/math/time_integration.h"

#include "vic/physics/equations_of_motion.h"

using namespace vic;
using namespace vic::linalg;

TEST(TimeIntegration, Setup)
{
    eom::Properties properties{};
    eom::State state0{};

    const auto derivativeFunctor = [&properties](const eom::State& state) -> eom::StateDot {
        eom::StateDot dot{};
        dot.vel = state.vel;
        dot.acc = properties.gravity;
        return dot;
    };

    const auto integrateFunctor = [&properties](const eom::State& state0, //
                                                const eom::StateDot& dot0,
                                                const double dt) -> eom::State {
        return state0 + (dt * dot0); //
    };

    eom::State state1 = ForwardEuler(state0, 0.1, derivativeFunctor, integrateFunctor);

    const auto state1_tr = Trapezoidal(state0, 0.1, derivativeFunctor, integrateFunctor);

    const auto state1_rk4 = RungeKutta4(state0, 0.1, derivativeFunctor, integrateFunctor);

    int bla = 1;
}
