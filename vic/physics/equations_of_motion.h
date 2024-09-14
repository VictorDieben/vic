#pragma once

#include "vic/math/quaternion.h"

#include "vic/linalg/linalg.h"

#include "vic/physics/moment_of_inertia.h"

namespace vic
{
namespace eom
{

using Pos = vic::linalg::Vector3d;
using Vel = vic::linalg::Vector3d;
using Acc = vic::linalg::Vector3d;

using Rot = vic::Quaternion<double>;
using RotVel = vic::linalg::Vector3d;
using RotAcc = vic::linalg::Vector3d;

using Mass = double;

struct Properties
{
    Acc gravity{0., 0., -9.81};

    // todo: make part of properties or state? the values should not change
    Mass mass;
    MoIDiag<double> moi = MoISphere(0.05, 2.); // metal ball
    MoIDiag<double> moiInverse = vic::linalg::Inverse(moi);
};

struct State
{
    Pos pos;
    Vel vel;

    Rot rot;
    RotVel rotVel;
};

struct StateDot
{
    Vel vel;
    Acc acc;

    RotVel rotVel;
    RotAcc rotAcc;
};

// This is the (scalar * Matrix) operator
StateDot operator*(const double f, const StateDot& dot)
{
    StateDot dot2;
    dot2.vel = linalg::Matmul(dot.vel, f);
    dot2.acc = linalg::Matmul(dot.acc, f);
    dot2.rotVel = linalg::Matmul(dot.rotVel, f);
    dot2.rotAcc = linalg::Matmul(dot.rotAcc, f);
    return dot2;
}

StateDot operator*(const StateDot& dot, const double f) { return f * dot; }

StateDot operator+(const StateDot& dot1, const StateDot& dot2)
{
    StateDot res;
    res.vel = linalg::Add(dot1.vel, dot2.vel);
    res.acc = linalg::Add(dot1.acc, dot2.acc);
    res.rotVel = linalg::Add(dot1.rotVel, dot2.rotVel);
    res.rotAcc = linalg::Add(dot1.rotAcc, dot2.rotAcc);

    return res;
}

State operator+(const State& state, const StateDot& dot)
{
    // dot should be pre-multiplied with timestep
    State res;
    res.pos = linalg::Add(state.pos, dot.vel);
    res.vel = linalg::Add(state.vel, dot.acc);
    res.rotVel = linalg::Add(state.rotVel, dot.rotAcc);
    res.rot = state.rot; // todo: quaternion time integration
    return res;
}

} // namespace eom
} // namespace vic