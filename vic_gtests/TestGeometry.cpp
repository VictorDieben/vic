#include "pch.h"

#include "vic/geometry/geometry.h"

using namespace vic;

namespace vic
{
namespace geom
{

TEST(TestGeom, Initialization)
{
    Point<double, 2> p1{0.1, 10.};
    ASSERT_EQ(p1, p1);
    ASSERT_NEAR(p1[0], 0.1, 1E-10);
    ASSERT_NEAR(p1[1], 10., 1E-10);

    Direction<double, 2> d1{0.1, 10.};
    ASSERT_NEAR(d1[0], 0.1, 1E-10);
    ASSERT_NEAR(d1[1], 10., 1E-10);

    Line<int, 2> l1{{0, 1}, {2, 3}};
    ASSERT_EQ(l1.pos[0], 0);
    ASSERT_EQ(l1.pos[1], 1);
    ASSERT_EQ(l1.dir[0], 2);
    ASSERT_EQ(l1.dir[1], 3);

    LineSegment<int, 2> seg1{{0, 1}, {1, 0}};
    ASSERT_EQ(seg1.p1[0], 0);
    ASSERT_EQ(seg1.p1[1], 1);
    ASSERT_EQ(seg1.p2[0], 1);
    ASSERT_EQ(seg1.p2[1], 0);

    Triangle<int, 2> tri1{{0, 0}, {1, 0}, {0, 1}};
    ASSERT_EQ(tri1.p1, (Point<int, 2>{0, 0}));
    ASSERT_EQ(tri1.p2, (Point<int, 2>{1, 0}));
    ASSERT_EQ(tri1.p3, (Point<int, 2>{0, 1}));

    Interval<int> interval1{-1, 1};
    ASSERT_EQ(interval1.min, -1);
    ASSERT_EQ(interval1.max, 1);

    CubeAxisAligned<int, 3> cube1{Interval<int>{-1, 1}, //
                                  Interval<int>{-2, 2},
                                  Interval<int>{-3, 3}};

    Cylinder<double, 3> cylinder{Point<double, 3>{-1, 0, 0}, //
                                 Point<double, 3>{1, 0, 0},
                                 1.};
}

} // namespace geom
} // namespace vic
