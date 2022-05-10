#include "pch.h"

#include "vic/geometry/algorithms/algorithms.h"
#include "vic/geometry/algorithms/bbox_tree.h"
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
    using Result = TriLineIntersectionResult<double>;
    const auto ResultEqual = [](const auto& s1, const auto& s2) {
        return std::fabs(s1.u - s2.u) < 1E-10 && //
               std::fabs(s1.v - s2.v) < 1E-10 && //
               std::fabs(s1.t - s2.t) < 1E-10;
    };

    constexpr Triangle<double, 3> tri{Point3d{{0, 0, 0}}, //
                                      Point3d{{1, 0, 0}},
                                      Point3d{{0, 1, 0}}};

    constexpr LineSegment<double, 3> seg{Point3d{{0, 0, 1}}, //
                                         Point3d{{0, 0, -1}}};

    constexpr auto res = TriLineIntersection(tri, seg);
    EXPECT_TRUE(ResultEqual(res, Result{0, 0, .5}));

    auto ans = TriLineIntersection(tri, LineSegment<double, 3>{Point3d{{1, 0, 1}}, Point3d{{1, 0, -1}}});
    EXPECT_TRUE(ResultEqual(Result{1., 0, .5}, ans));

    ans = TriLineIntersection(tri, LineSegment<double, 3>{Point3d{{0, 1, 1}}, Point3d{{0, 1, -1}}});
    EXPECT_TRUE(ResultEqual(Result{0., 1., .5}, ans));

    ans = TriLineIntersection(tri, LineSegment<double, 3>{Point3d{{1, 1, 1}}, Point3d{{1, 1, -1}}});
    EXPECT_TRUE(ResultEqual(Result{1., 1., .5}, ans));

    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(-1., 1.);

    // ~45.5m iterations/s
    for(const auto i : Range(1000))
    {
        const auto t1 = Point3d{{dist(g), dist(g), dist(g)}};
        const auto t2 = Point3d{{dist(g) + 1., dist(g), dist(g)}};
        const auto t3 = Point3d{{dist(g), dist(g) + 1., dist(g)}};

        const auto e12 = Subtract(t2, t1);
        const auto e13 = Subtract(t3, t1);
        const auto n = Cross(e12, e13);

        // const auto o1 = dist(g), o2 = dist(g), o3 = dist(g);
        const auto o1 = 0., o2 = 0., o3 = 0.;
        const auto d1 = dist(g), d2 = dist(g), d3 = dist(g);

        const auto s1 = Point3d{{o1 + d1, o2 + d2, o3 + d3}};
        const auto s2 = Point3d{{o1 - d1, o2 - d2, o3 - d3}};

        const auto dir = Subtract(s2, s1); // line direction

        const Triangle<double, 3> triangle{t1, t2, t3};
        const LineSegment<double, 3> segment{s1, s2};

        const auto result = TriLineIntersection(triangle, segment);

        // verify that calculated intersection point is on surface spanned by e12xe13
        const auto inter1 = Add(s1, Matmul(result.t, dir));
        const auto inter2 = Add(t1, Matmul(result.u, e12), Matmul(result.v, e13));

        ASSERT_LT(Norm(Subtract(inter1, inter2)), 1E-7);
    }
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
    std::uniform_real_distribution<double> offset(-0.1, 0.1);

    // ~52.3m intersections per second (random values seem expensive)
    for(const auto i : Range(1000))
    {
        const double x = xyz(g);
        const double y = xyz(g);
        const double z = xyz(g);

        const double r = 1. + offset(g);

        const Sphere<double, 3> sphere{Point3d{{x + offset(g), y + offset(g), z + offset(g)}}, r};

        const double dx = dxyz(g);
        const double dy = dxyz(g);
        const double dz = dxyz(g);

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

constexpr bool IntervalEqual(const Interval<double>& int1, const Interval<double>& int2, const double eps = 1E-10)
{
    return Abs(int1.min - int2.min) < eps && Abs(int1.max - int2.max) < eps; //
};

TEST(TestGeom, Interval)
{
    // Includes
    ASSERT_TRUE(Includes(Interval<double>{-1., 1.}, Interval<double>{-.99, .99}));
    ASSERT_TRUE(Includes(Interval<double>{-1., 1.}, Interval<double>{-1., 1.}));
    ASSERT_FALSE(Includes(Interval<double>{-1., 1.}, Interval<double>{-1.01, .99}));
    ASSERT_FALSE(Includes(Interval<double>{-1., 1.}, Interval<double>{-.99, 1.01}));

    // Overlaps
    ASSERT_TRUE(Overlaps(Interval<double>{-1., 1.}, Interval<double>{-1.01, -0.99}));
    ASSERT_TRUE(Overlaps(Interval<double>{-1., 1.}, Interval<double>{.99, 1.01}));
    ASSERT_FALSE(Overlaps(Interval<double>{-1., 1.}, Interval<double>{1.01, 1.02}));
    ASSERT_FALSE(Overlaps(Interval<double>{-1., 1.}, Interval<double>{-1.02, -1.01}));

    // Combine
    ASSERT_TRUE(IntervalEqual(Combine(Interval<double>{-1, 0}, Interval<double>{0, 1}), Interval<double>{-1, 1}));
    ASSERT_TRUE(IntervalEqual(Combine(Interval<double>{-1, 1}, Interval<double>{-2, 2}), Interval<double>{-2, 2}));
}

constexpr bool BBoxEqual(const BBox<double, 2>& bbox1, const BBox<double, 2>& bbox2)
{
    return IntervalEqual(bbox1.intervals.at(0), bbox2.intervals.at(0)) && //
           IntervalEqual(bbox1.intervals.at(1), bbox2.intervals.at(1)); //
};

TEST(TestGeom, BBox)
{
    using Inter = Interval<double>;
    const BBox<double, 2> bbox1{{Inter{1., 2.}, Inter{1., 2.}}};

    // Includes
    ASSERT_TRUE(Includes(bbox1, BBox<double, 2>{{Inter{1.01, 1.99}, Inter{1.01, 1.99}}}));
    ASSERT_FALSE(Includes(bbox1, BBox<double, 2>{{Inter{1.01, 1.99}, Inter{1.01, 2.01}}}));
    ASSERT_FALSE(Includes(bbox1, BBox<double, 2>{{Inter{1.01, 2.01}, Inter{1.01, 1.99}}}));
    ASSERT_FALSE(Includes(bbox1, BBox<double, 2>{{Inter{1.01, 2.01}, Inter{1.01, 2.01}}}));

    // Overlaps
    ASSERT_TRUE(Overlaps(bbox1, BBox<double, 2>{{Inter{1.99, 3.}, Inter{1.99, 3.}}}));
    ASSERT_TRUE(Overlaps(bbox1, BBox<double, 2>{{Inter{0, 3.}, Inter{0, 3.}}}));
    ASSERT_TRUE(Overlaps(bbox1, BBox<double, 2>{{Inter{0, 3.}, Inter{0.99, 1.01}}}));

    ASSERT_FALSE(Overlaps(bbox1, BBox<double, 2>{{Inter{3., 4}, Inter{3., 4}}}));
    ASSERT_FALSE(Overlaps(bbox1, BBox<double, 2>{{Inter{3., 4}, Inter{1., 2.}}}));
    ASSERT_FALSE(Overlaps(bbox1, BBox<double, 2>{{Inter{1., 2.}, Inter{3., 4}}}));

    // Combine
    ASSERT_TRUE(BBoxEqual(Combine(bbox1, BBox<double, 2>{{Inter{1., 2.}, Inter{3., 4}}}), //
                          BBox<double, 2>{{Inter{1., 2.}, Inter{1., 4}}}));

    ASSERT_TRUE(BBoxEqual(Combine(bbox1, BBox<double, 2>{{Inter{3., 4.}, Inter{3., 4}}}), //
                          BBox<double, 2>{{Inter{1., 4.}, Inter{1., 4}}}));

    ASSERT_TRUE(BBoxEqual(Combine(bbox1, BBox<double, 2>{{Inter{1., 2.}, Inter{0., 3}}}), //
                          BBox<double, 2>{{Inter{1., 2.}, Inter{0., 3}}}));
}

TEST(TestGeom, BoxTree)
{
    const auto lambda = [&](const std::size_t key) {
        return BBox<double, 3>{}; //
    };
    BBoxTree<std::size_t, 3, decltype(lambda)> boxtree{lambda};

    boxtree.Insert(1u);
}

} // namespace geom
} // namespace vic
