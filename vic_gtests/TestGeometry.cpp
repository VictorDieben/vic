#include "pch.h"

#include "vic/geometry/algorithms/algorithms.h"
#include "vic/geometry/geometry.h"
#include "vic/linalg/add.h"
#include "vic/linalg/tools.h"

using namespace vic;

namespace vic
{
namespace geom
{

TEST(TestGeom, Initialization)
{
    using namespace vic::linalg;

    Point2d p1{{0.1, 10.}};
    ASSERT_TRUE(IsEqual(p1, p1));
    ASSERT_NEAR(p1.Get(0), 0.1, 1E-10);
    ASSERT_NEAR(p1.Get(1), 10., 1E-10);

    Direction<double, 2> d1{{0.1, 10.}};
    ASSERT_NEAR(d1.Get(0), 0.1, 1E-10);
    ASSERT_NEAR(d1.Get(1), 10., 1E-10);

    Line<int, 2> l1{Point2i{{0, 1}}, Point2i{{2, 3}}};
    ASSERT_EQ(l1.pos.Get(0), 0);
    ASSERT_EQ(l1.pos.Get(1), 1);
    ASSERT_EQ(l1.dir.Get(0), 2);
    ASSERT_EQ(l1.dir.Get(1), 3);

    LineSegment<int, 2> seg1{Point2i{{0, 1}}, Point2i{{1, 0}}};
    ASSERT_EQ(seg1.p1.Get(0), 0);
    ASSERT_EQ(seg1.p1.Get(1), 1);
    ASSERT_EQ(seg1.p2.Get(0), 1);
    ASSERT_EQ(seg1.p2.Get(1), 0);

    ASSERT_TRUE(IsEqual(Point2i{{0, 0}}, Point2i{{0, 0}}));

    Triangle<int, 2> tri1{Point2i{{0, 0}}, Point2i{{1, 0}}, Point2i{{0, 1}}};
    ASSERT_TRUE(IsEqual(tri1.p1, Point2i{{0, 0}}));
    ASSERT_TRUE(IsEqual(tri1.p2, Point2i{{1, 0}}));
    ASSERT_TRUE(IsEqual(tri1.p3, Point2i{{0, 1}}));

    Interval<int> interval1{-1, 1};
    ASSERT_EQ(interval1.min, -1);
    ASSERT_EQ(interval1.max, 1);

    CubeAxisAligned<int, 3> cube1{Interval<int>{-1, 1}, //
                                  Interval<int>{-2, 2},
                                  Interval<int>{-3, 3}};

    Cylinder<double, 3> cylinder{Point3d{{-1, 0, 0}}, //
                                 Point3d{{1, 0, 0}},
                                 1.};
}

TEST(TestGeom, TriLineIntersection)
{
    using namespace vic::linalg;

    constexpr Triangle<double, 3> tri{Point3d{{0, 0, 0}}, //
                                      Point3d{{1, 0, 0}},
                                      Point3d{{0, 1, 0}}};

    constexpr LineSegment<double, 3> segment{Point3d{{0, 0, 1}}, //
                                             Point3d{{0, 0, -1}}};

    constexpr auto res = TriLineIntersection(tri, segment);
    EXPECT_NEAR(res.u, 0., 1e-10);
    EXPECT_NEAR(res.v, 0., 1e-10);
    EXPECT_NEAR(res.t, .5, 1e-10);
}

TEST(TestGeom, SphereLineIntersection)
{
    //constexpr LineSegment<double, 3> segment{Point3d{{0, 0, 2}}, //
    //                                         Point3d{{0, 0, -2}}};
    //constexpr Sphere<double, 3> sphere{Point3d{{0, 0, 0}}, 1.};

    //auto res = SphereLineIntersection(sphere, segment);
    //EXPECT_NEAR(res.u1, 0.25, 1e-10);
    //EXPECT_NEAR(res.u2, 0.75, 1e-10);
    //EXPECT_EQ(res.nIntersections, 2);

    std::default_random_engine g;
    std::uniform_real_distribution<double> xyz(-10., 10);
    std::uniform_real_distribution<double> dxyz(3, 10);
    std::uniform_real_distribution<double> offset(-0.1, 0.01);

    // ~52.3m intersections per second (random values seem expensive)
    for(const auto i : Range(1000))
    {
        const auto x = xyz(g);
        const auto y = xyz(g);
        const auto z = xyz(g);

        const double r = 1.;

        const Sphere<double, 3> sphere{Point3d{{x + offset(g), y + offset(g), z + offset(g)}}, r};

        const auto dx = dxyz(g);
        const auto dy = dxyz(g);
        const auto dz = dxyz(g);

        const auto p1 = Point3d{{x + dx, y + dy, z + dz}};
        const auto p2 = Point3d{{x - dx, y - dy, z - dz}};
        const LineSegment<double, 3> segment{p1, p2};

        // use this to get a rough performance estimate
        //for(const auto j : Range(1000))
        //{
        //    const auto ans = SphereLineIntersection(sphere, segment);
        //}

        const auto ans = SphereLineIntersection(sphere, segment);
        ASSERT_EQ(ans.nIntersections, 2);

        const auto dir = Subtract(p2, p1);
        const auto int1 = Add(p1, Matmul(ans.u1, dir));
        const auto int2 = Add(p1, Matmul(ans.u2, dir));

        // make sure the two intersections are at distance r from the circle
        ASSERT_NEAR(Norm(Subtract(sphere.pos, int1)), r, 1E-10);
        ASSERT_NEAR(Norm(Subtract(sphere.pos, int2)), r, 1E-10);
    }
}

} // namespace geom
} // namespace vic
