
#include "gtest/gtest.h"

#include "vic/geometry/algorithms/algorithms.h"
#include "vic/geometry/algorithms/assignment_problem.h"
#include "vic/geometry/algorithms/bbox_tree.h"
#include "vic/geometry/algorithms/intersections.h"
#include "vic/geometry/algorithms/interval_heap.h"
#include "vic/geometry/geometry.h"

#include "vic/linalg/algorithms/add.h"
#include "vic/linalg/algorithms/matmul.h"
#include "vic/linalg/tools.h"

#include "vic/geometry/algorithms/balanced_aabb_tree.h"
#include "vic/geometry/mesh.h"

#include "vic/utils/timing.h"
#include <format>
#include <random>

using namespace vic;

namespace vic
{
namespace geom
{

using namespace vic::linalg;
using namespace vic::mesh;

TEST(TestGeom, Initialization)
{

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
    ASSERT_TRUE(IsEqual(tri1.points[0], Point2i{{0, 0}}));
    ASSERT_TRUE(IsEqual(tri1.points[1], Point2i{{1, 0}}));
    ASSERT_TRUE(IsEqual(tri1.points[2], Point2i{{0, 1}}));

    Interval<int> interval1{-1, 1};
    ASSERT_EQ(interval1.min, -1);
    ASSERT_EQ(interval1.max, 1);

    AABB<int, 3> cube1{Interval<int>{-1, 1}, //
                       Interval<int>{-2, 2},
                       Interval<int>{-3, 3}};

    Cylinder<double, 3> cylinder{Point3d{{-1, 0, 0}}, //
                                 Point3d{{1, 0, 0}},
                                 1.};
}

TEST(TestGeom, TriLineIntersection)
{
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

TEST(TestGeom, AABBLineIntersection)
{
    AABB<double, 3> bbox{Interval<double>{1, 2}, Interval<double>{1, 2}, Interval<double>{1, 2}};

    // simple intersection
    auto res = AABBLineIntersection(bbox,
                                    Line<double, 3>{Point<double, 3>{{0, 1.5, 1.5}}, //
                                                    Direction<double, 3>{{1, 0, 0}}});
    EXPECT_DOUBLE_EQ(res.interval.min, 1.);
    EXPECT_DOUBLE_EQ(res.interval.max, 2.);

    // intersection starting inside bbox
    auto res2 = AABBLineIntersection(bbox,
                                     Line<double, 3>{Point<double, 3>{{1.5, 1.5, 1.5}}, //
                                                     Direction<double, 3>{{0, 1, 0}}});
    EXPECT_DOUBLE_EQ(res2.interval.min, -0.5);
    EXPECT_DOUBLE_EQ(res2.interval.max, 0.5);

    // intersection cutting through a corner
    auto res3 = AABBLineIntersection(bbox,
                                     Line<double, 3>{Point<double, 3>{{0, 0, 1.5}}, //
                                                     Direction<double, 3>{{1, 1, 0}}});
    EXPECT_DOUBLE_EQ(res3.interval.min, 1.);
    EXPECT_DOUBLE_EQ(res3.interval.max, 2.);

    // zero direction
    auto res4 = AABBLineIntersection(bbox,
                                     Line<double, 3>{Point<double, 3>{{1.5, 1.5, 1.5}}, //
                                                     Direction<double, 3>{{0, 0, 0}}});
    EXPECT_DOUBLE_EQ(res4.interval.min, -std::numeric_limits<double>::infinity());
    EXPECT_DOUBLE_EQ(res4.interval.max, std::numeric_limits<double>::infinity());

    // intersection in negative direction
    auto res5 = AABBLineIntersection(bbox,
                                     Line<double, 3>{Point<double, 3>{{3, 3, 3}}, //
                                                     Direction<double, 3>{{-1, -1, -1}}});
    EXPECT_DOUBLE_EQ(res5.interval.min, 1.);
    EXPECT_DOUBLE_EQ(res5.interval.max, 2.);
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
    std::default_random_engine g;
    std::uniform_real_distribution<double> r(-1., 1.);

    using Inter = Interval<double>;
    const BBox<double, 2> bbox1{{Inter{1., 2.}, Inter{1., 2.}}};

    // Includes
    ASSERT_TRUE(Includes(bbox1, BBox<double, 2>{{Inter{1.01, 1.99}, Inter{1.01, 1.99}}}));
    ASSERT_FALSE(Includes(bbox1, BBox<double, 2>{{Inter{1.01, 1.99}, Inter{1.01, 2.01}}}));
    ASSERT_FALSE(Includes(bbox1, BBox<double, 2>{{Inter{1.01, 2.01}, Inter{1.01, 1.99}}}));
    ASSERT_FALSE(Includes(bbox1, BBox<double, 2>{{Inter{1.01, 2.01}, Inter{1.01, 2.01}}}));

    // make sure boxes always include themselves
    for(const std::size_t i : Range(1000))
    {
        const double pos = r(g), eps = r(g);
        BBox<double, 1> box{{Inter{pos - eps, pos + eps}}};
        ASSERT_TRUE(Includes(box, box));
    }

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
    std::default_random_engine g;
    std::uniform_real_distribution<double> pos(-1., 1.);
    std::uniform_real_distribution<double> eps(-0.001, 0.001);

    using TestObject = std::vector<Point<double, 3>>;

    const auto toBBox = [](const TestObject& object) {
        BBox<double, 3> bbox{};
        for(const auto& point : object)
            for(vic::linalg::MatrixSize i = 0; i < 3; ++i)
                bbox.intervals[i] = Interval<double>{Min(bbox.intervals[i].min, point.Get(i)), //
                                                     Max(bbox.intervals[i].max, point.Get(i))};
        return bbox;
    };

    BBoxTree<TestObject, 3, decltype(toBBox)> boxtree{toBBox};

    std::vector<TestObject> objects;

    const auto makeObject = [&]() {
        TestObject object{};
        const double px = pos(g), py = pos(g), pz = pos(g);
        for(const auto i : Range(10))
            object.push_back(Point<double, 3>{{px + eps(g), py + eps(g), pz + eps(g)}});
        return object;
    };

    for(const auto i : Range(1000))
        objects.push_back(makeObject());

    {
        CTimer timer{};
        for(auto& object : objects)
            boxtree.Insert(object);
        const auto totalTime = timer.GetTime();

        int bla = 1;
    }
}

//TEST(TestGeom, TriTri)
//{
//    std::default_random_engine g;
//    std::uniform_real_distribution<double> pos(-1., 1.);
//
//    // ~13m iters per second
//    for(const auto i : Range(1000))
//    {
//        const Triangle<double, 3> tri1{Point3d{{pos(g), pos(g), pos(g)}}, //
//                                       Point3d{{pos(g), pos(g), pos(g)}},
//                                       Point3d{{pos(g), pos(g), pos(g)}}};
//        const Triangle<double, 3> tri2{Point3d{{pos(g), pos(g), pos(g)}}, //
//                                       Point3d{{pos(g), pos(g), pos(g)}},
//                                       Point3d{{pos(g), pos(g), pos(g)}}};
//
//        const auto res = TriTriIntersection(tri1, tri2);
//
//        if(res.interval.min >= res.interval.max)
//            continue; // not intersecting
//
//        const auto tarray = Linspace(res.interval.min, res.interval.max, 5);
//        for(const auto t : tarray | std::views::drop(1) | vic::drop_last)
//        {
//            const auto pos = Add(res.pos, Matmul(res.dir, t));
//
//            const double u1 = Project(tri1.points[0], Subtract(tri1.points[1], tri1.points[0]), pos);
//            const double v1 = Project(tri1.points[0], Subtract(tri1.points[2], tri1.points[0]), pos);
//            // ASSERT_TRUE(u1 >= 0. && v1 >= 0. && (u1 + v1) <= 1.);
//
//            const double u2 = Project(tri2.points[0], Subtract(tri2.points[1], tri2.points[0]), pos);
//            const double v2 = Project(tri2.points[0], Subtract(tri2.points[2], tri2.points[0]), pos);
//            // ASSERT_TRUE(u2 >= 0. && v2 >= 0. && (u2 + v2) <= 1.);
//        }
//    }
//}

TEST(TestGeom, IntervalHeap)
{
    using Box = BBox<double, 2>;
    using Key = uint32_t;
    std::default_random_engine g;
    std::uniform_real_distribution<double> pos(-1., 1.);
    std::uniform_real_distribution<double> size(0.00001, 0.1);
    constexpr std::size_t nodes = 10000000;

    const auto randomInterval = [&]() {
        double p = pos(g), d = size(g);
        return Interval<double>{p - d, p + d};
    };

    const auto randomBBox = [&]() { return Box{{randomInterval(), randomInterval()}}; };

    // make a list of random bboxes
    std::vector<Box> boxes{};
    for(const auto i : Range(nodes))
        boxes.push_back(randomBBox());

    const auto lambdax = [&](Key key) {
        const auto bbox = boxes.at(key).intervals[0];
        return bbox;
    };
    IntervalHeap<Key, decltype(lambdax)> heapX{lambdax};

    for(Key i = 0; i < boxes.size(); ++i)
        heapX.Add(i);
    heapX.Resort();

    const auto overlapInterval = Interval<double>{-0.5, -0.49};

    // find overlaps using algorithm
    std::chrono::duration<double> time, bruteForceTime;
    std::vector<Key> overlap;
    CTimer timer{};
    heapX.Overlap(overlapInterval, overlap);
    time = timer.GetTime();

    // find overlaps using brute force
    std::vector<Key> bruteForce;
    CTimer bruteForceTimer{};
    for(Key i = 0; i < boxes.size(); ++i)
        if(Overlaps(overlapInterval, lambdax(i)))
            bruteForce.push_back(i);
    bruteForceTime = bruteForceTimer.GetTime();

    std::cout << std::format("algorithm: {}[s]; brute force: {}[s]", time.count(), bruteForceTime.count()) << std::endl;

    // check that all overlaps do indeed overlap
    for(const auto& item : overlap)
        ASSERT_TRUE(Overlaps(boxes.at(item).intervals[0], overlapInterval));

    // Check that all items that are not in overlap do indeed not overlap
    std::sort(overlap.begin(), overlap.end());
    for(const auto i : Range(nodes))
    {
        if(!std::binary_search(overlap.begin(), overlap.end(), i))
        {
            const auto interval = boxes.at(i).intervals[0];
            ASSERT_FALSE(Overlaps(interval, overlapInterval));
        }
    }

    ASSERT_TRUE(false);
}

TEST(TestGeom, IntervalSorted)
{
    using Inter = Interval<double>;
    using Key = uint32_t;

    std::default_random_engine g;
    std::uniform_real_distribution<double> pos(-1., 1.);
    std::uniform_real_distribution<double> size(0.00001, 0.1);

    const auto randomInterval = [&]() {
        const double p = pos(g), d = size(g);
        return Inter{p - d, p + d};
    };

    constexpr std::size_t nodes = 100;

    std::vector<Inter> intervals{};
    for(const auto i : Range(nodes))
        intervals.push_back(randomInterval());

    const auto getInterval = [&](const Key key) { return intervals.at(key); };

    IntervalSorted<Key, decltype(getInterval)> collection{getInterval};

    for(auto i : Range<Key>(nodes))
        collection.Add(i);
    collection.Resort();

    //

    int bla = 1;
    (void)bla;
}

TEST(TestGeom, HeapVector)
{
    ASSERT_EQ(FromBinaryIndex(0, 4), 0);
    ASSERT_EQ(FromBinaryIndex(1, 4), 2);
    ASSERT_EQ(FromBinaryIndex(2, 4), 1);
    ASSERT_EQ(FromBinaryIndex(3, 4), 3);

    const auto lambda = [&](const double d1, const double d2) { return d1 < d2; };
    BinaryTreeVector<double, decltype(lambda)> heapvec{lambda};

    //ASSERT_EQ(heapvec.size(), 0);
    //ASSERT_EQ(heapvec.capacity(), 0);

    //heapvec.push_back(1.);
    //ASSERT_EQ(heapvec.size(), 1);
    //ASSERT_EQ(heapvec.capacity(), 1);
    //ASSERT_TRUE(heapvec.IsFilled(0));
}

TEST(TestGeom, PyramidVector)
{
    EXPECT_EQ(PyramidSize(0), 0);
    EXPECT_EQ(PyramidSize(1), 1);
    EXPECT_EQ(PyramidSize(2), 3);
    EXPECT_EQ(PyramidSize(3), 7);

    // EXPECT_EQ(PyramidFirstIndex(0), 0); // what would the correct value be here?
    EXPECT_EQ(PyramidFirstIndex(1), 0);
    EXPECT_EQ(PyramidFirstIndex(2), 1);
    EXPECT_EQ(PyramidFirstIndex(3), 3);
    EXPECT_EQ(PyramidFirstIndex(4), 7);
    EXPECT_EQ(PyramidFirstIndex(5), 15);

    // EXPECT_EQ(PyramidIndex(0, 0), 0);  // what would the correct value be here?
    EXPECT_EQ(PyramidIndex(1, 0), 0);

    EXPECT_EQ(PyramidIndex(2, 0), 1);
    EXPECT_EQ(PyramidIndex(2, 1), 2);

    EXPECT_EQ(PyramidIndex(3, 0), 3);
    EXPECT_EQ(PyramidIndex(3, 1), 4);
    EXPECT_EQ(PyramidIndex(3, 2), 5);
    EXPECT_EQ(PyramidIndex(3, 3), 6);

    //
    EXPECT_EQ(PyramidLevelSize(0), 0);
    EXPECT_EQ(PyramidLevelSize(1), 1);
    EXPECT_EQ(PyramidLevelSize(2), 2);
    EXPECT_EQ(PyramidLevelSize(3), 4);
    EXPECT_EQ(PyramidLevelSize(4), 8);
    EXPECT_EQ(PyramidLevelSize(5), 16);

    PyramidVector<double> pyramid{};

    pyramid.SetLevel(0);
    EXPECT_EQ(pyramid.GetSize(), 0);

    pyramid.SetLevel(1);
    EXPECT_EQ(pyramid.GetSize(), 1);

    pyramid.SetLevel(2);
    EXPECT_EQ(pyramid.GetSize(), 1 + 2);

    pyramid.SetLevel(3);
    EXPECT_EQ(pyramid.GetVector().size(), 1 + 2 + 4);

    // test LevelIterator
    const auto& vec = pyramid.GetVector();

    const auto emptyLevel = pyramid.LevelIterator(0);
    ASSERT_EQ(std::distance(emptyLevel.begin(), emptyLevel.end()), 0);

    const auto firstLevel = pyramid.LevelIterator(1);
    ASSERT_EQ(std::distance(firstLevel.begin(), firstLevel.end()), 1);

    const auto secondLevel = pyramid.LevelIterator(2);
    ASSERT_EQ(std::distance(secondLevel.begin(), secondLevel.end()), 2);
    ASSERT_EQ(firstLevel.end(), secondLevel.begin());

    const auto thirdLevel = pyramid.LevelIterator(3);
    ASSERT_EQ(std::distance(thirdLevel.begin(), thirdLevel.end()), 4);
    ASSERT_EQ(secondLevel.end(), thirdLevel.begin());
}

//TEST(TestGeom, BTreeVector)
//{
//    EXPECT_EQ(NextPowerOf2(0u), 1); // todo: probably not what we want
//    EXPECT_EQ(NextPowerOf2(1u), 1);
//    EXPECT_EQ(NextPowerOf2(2u), 2);
//    EXPECT_EQ(NextPowerOf2(3u), 4);
//    EXPECT_EQ(NextPowerOf2(4u), 4);
//    EXPECT_EQ(NextPowerOf2(7u), 8);
//    EXPECT_EQ(NextPowerOf2(8u), 8);
//
//    BTreeVector<double> vec{};
//    EXPECT_EQ(vec.GetLevel(), 0);
//    EXPECT_EQ(vec.GetCapacity(), 1);
//
//    vec.push_back({});
//    EXPECT_EQ(vec.GetLevel(), 0);
//    EXPECT_EQ(vec.GetCapacity(), 1);
//
//    vec.push_back({});
//    EXPECT_EQ(vec.GetLevel(), 1);
//    EXPECT_EQ(vec.GetCapacity(), 2);
//
//    vec.push_back({});
//    EXPECT_EQ(vec.GetLevel(), 2);
//    EXPECT_EQ(vec.GetSize(), 3);
//    EXPECT_EQ(vec.GetCapacity(), 4);
//
//    vec.push_back({});
//    EXPECT_EQ(vec.GetLevel(), 2);
//    EXPECT_EQ(vec.GetSize(), 4);
//    EXPECT_EQ(vec.GetCapacity(), 4);
//
//    vec.push_back({});
//    EXPECT_EQ(vec.GetLevel(), 3);
//    EXPECT_EQ(vec.GetSize(), 5);
//    EXPECT_EQ(vec.GetCapacity(), 8);
//
//    // now pop the back items
//    vec.pop_back();
//    EXPECT_EQ(vec.GetLevel(), 3);
//    EXPECT_EQ(vec.GetSize(), 4);
//    EXPECT_EQ(vec.GetCapacity(), 8);
//
//    vec.pop_back();
//    EXPECT_EQ(vec.GetLevel(), 2);
//    EXPECT_EQ(vec.GetSize(), 3);
//    EXPECT_EQ(vec.GetCapacity(), 4);
//}

TEST(TestGeom, BalancedAABBTree)
{
    using Key = std::size_t;
    using Inter = Interval<double>;
    using BBoxTree = BalancedAABBTree<Key, double, 2>;
    using BBox = BBoxTree::BBox;
    using Leaf = BBoxTree::LeafType;

    std::default_random_engine g;
    std::uniform_real_distribution<double> pos(-1., 1.);
    std::uniform_real_distribution<double> size(0.00001, 0.1);

    std::vector<BBox> boxes{};
    const std::size_t nItems = vic::Pow<12>(2);
    for(auto i = 0; i < nItems; ++i)
    {
        const auto p1 = pos(g), s1 = size(g);
        const auto p2 = pos(g), s2 = size(g);
        boxes.push_back(BBox{Inter{p1 - s1, p1 + s1}, Inter{p2 - s2, p2 + s2}});
    }

    BBoxTree tree{}; //

    for(auto i = 0; i < nItems; ++i)
    {
        tree.Insert(Leaf{boxes.at(i), {0}});
    }

    tree.Update();
}

TEST(TestGeom, GroupPairsOfTwo)
{
    using Inter = Interval<double>;
    std::default_random_engine g;
    std::uniform_real_distribution<double> pos(-1., 1.);
    std::uniform_real_distribution<double> size(0.01, 0.1);

    std::vector<Inter> intervals{};
    const std::size_t nItems = 1000;
    for(auto i = 0; i < nItems; ++i)
    {
        const auto p = pos(g), s = size(g);
        intervals.push_back(Inter{p - s, p + s});
    }

    const auto volumeLambda = [](const Inter& left, const Inter& right) {
        const auto combined = Combine(left, right);
        return combined.max - combined.min;
    }; // sum of volumes: 55.9504

    // filled fraction is a slightly better heuristic.
    // but maybe the extra computational effort is not worth it.
    const auto filledFractionLambda = [](const Inter& left, const Inter& right) {
        const auto combined = Combine(left, right);
        return Volume(combined) / (Volume(left) + Volume(right));
    };

    const auto res = GroupPairsOfTwo(intervals, volumeLambda);

    double sum = 0.;
    for(const auto& pair : res)
    {
        const auto lambdaVal = volumeLambda(intervals.at(pair.first), intervals.at(pair.second));
        // std::cout << pair.first << "; " << pair.second << "; " << lambda(intervals.at(pair.first), intervals.at(pair.second)) << std::endl;
        sum += lambdaVal;
    }

    std::cout << "sum of volumes: " << sum << std::endl;

    ASSERT_TRUE(false);
}

TEST(TestGeom, MeshUvSphere)
{
    const double radius = .5;

    const MeshIndex nu = 4; // horizontal number of surfaces
    const MeshIndex nv = 4; // verticl number of surfaces
    TriMesh<double> uvMesh = vic::mesh::GenerateUVSphere<double>(radius, nu, nv);

    EXPECT_EQ(uvMesh.vertices.size(), (4 * 4) + 2);
    for(const auto& vertex : uvMesh.vertices)
        EXPECT_NEAR(vic::linalg::Norm(vertex), radius, 1e-14);

    // todo:
    // EXPECT_TRUE(IsClosed(mesh));
}

TEST(TestGeom, MeshCubeSphere)
{
    const double radius = .5;

    auto mesh = GenerateCubeSphere(radius, 2);
    EXPECT_TRUE(IsClosed(mesh));

    for(const auto& vertex : mesh.vertices)
        EXPECT_NEAR(vic::linalg::Norm(vertex), radius, 1e-14);

    const auto triNormals = GenerateTriNormals(mesh);
    for(const auto& normal : triNormals)
        EXPECT_NEAR(vic::linalg::Norm(normal), 1., 1e-14);

    const auto vertexNormals = GenerateVertexNormals(mesh, triNormals);
    for(const auto& normal : vertexNormals)
        EXPECT_NEAR(vic::linalg::Norm(normal), 1., 1e-14);

    const auto uvs = GenerateVertexUVsPolar<double>(mesh, Vertex<double>{{0., 0., 0.}});
    for(const auto& uv : uvs)
    {
        // todo: check that uv is within [0; 1]
    }
}

TEST(TestGeom, MeshCube)
{
    const AABB<double, 3> bbox{Interval<double>{1, 2}, Interval<double>{1, 2}, Interval<double>{1, 2}};

    const auto cubeMesh = GenerateCube(bbox);
    EXPECT_TRUE(IsClosed(cubeMesh));

    const auto subdividedCube = Subdivide(cubeMesh);
    EXPECT_TRUE(IsClosed(subdividedCube));
}

TEST(TestGeom, MeshCone)
{
    const auto mesh = GenerateCone(1., 2., 8);
    EXPECT_TRUE(IsClosed(mesh));

    const auto subdividedMesh = Subdivide(mesh);
    EXPECT_TRUE(IsClosed(subdividedMesh));
}

TEST(TestGeom, MeshTorus)
{
    const auto mesh = GenerateTorus(1., .25, 16, 8);
    EXPECT_TRUE(IsClosed(mesh));

    const auto subdividedMesh = Subdivide(mesh);
    EXPECT_TRUE(IsClosed(subdividedMesh));
}

TEST(TestGeom, Subdivide)
{
    TriMesh<double> triMesh;
    triMesh.vertices = {ToVertex(0., 0., 0.), //
                        ToVertex(1., 0., 0.),
                        ToVertex(0., 1., 0.)};
    triMesh.tris = {Tri{0, 1, 2}};

    auto subdivided = Subdivide(triMesh);

    EXPECT_EQ(subdivided.vertices.size(), 6);
    EXPECT_EQ(subdivided.tris.size(), 4);
}

TEST(TestGeom, MeshCircle)
{
    const double radius = 1.5;
    const EdgeMesh edges = GenerateCircle(radius, 16u);

    EXPECT_TRUE(IsClosed(edges));

    for(const auto& vertex : edges.vertices)
        EXPECT_NEAR(vic::linalg::Norm(vertex), radius, 1e-14);
}

TEST(TestGeom, Revolve)
{
    const std::size_t n = 16;

    EdgeMesh<double> mesh;
    mesh.vertices = {Vertex<double>{{1., 0., -.1}}, //
                     Vertex<double>{{1., 0., .1}}};
    mesh.edges = {{0, 1}};

    const TriMesh revolvedMesh = Revolve(mesh, n, false);

    // EXPECT_EQ(revolvedMesh.vertices.size(), 2 * n);
    EXPECT_EQ(revolvedMesh.tris.size(), 2 * n);
}

TEST(TestGeom, EulerPoincare)
{
    //
}

} // namespace geom
} // namespace vic
