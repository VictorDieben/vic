
#include "gtest/gtest.h"

#include "test_base.h"

#include <vic/physics/moment_of_inertia.h>

using namespace vic;

TEST(Physics, MoI)
{
    const auto cube = MoICuboid<double>(1, 1, 1, 1);

    const auto sphere = MoISphere<double>(1, 1);
    const auto thinWalledSphere = MoISphereThinWalled<double>(1, 1);
    const auto thickWalledSphere = MoISphereThickWalled<double>(1, 1, 1);

    const auto cylinder = MoICylinder<double>(1, 1, 1);
    // const auto cylinderThinWalled = MoIThinWalledCylinder<double>(1, 1, 1);
    const auto cylinderThickWalled = MoIThickWalledCylinder<double>(1, 1, 1, 1);
}

TEST(Physics, MoICombine)
{
    const double mass = 1.;
    const auto cubeMoi = MoICuboid<double>(1, 1, 1, mass);
    const linalg::Vector3d r{1., 0., 0.};
    const auto displacedCube = ParallelAxisTheorem(mass, cubeMoi, r);
}

struct Properties
{
    double gravity = 9.81;
};
struct State
{
    double x;
    double y;
};
struct StateDot
{
    double dxdt;
    double dydt;
};

TEST(Physics, TimeIntegration) { }