#pragma once

#include "vic/geometry/geometry.h"
#include "vic/utils.h"

#include <optional>
#include <unordered_map>
#include <variant>

namespace vic
{
namespace geom
{

template <typename T, std::size_t dims>
using BBox = CubeAxisAligned<T, dims>;

// calculate the overlap of two intervals.
// min > max, if the two intervals do not overlap
template <typename T>
constexpr Interval<T> Overlap(const Interval<T>& interval1, const Interval<T>& interval2)
{
    return Interval<T>{Max(interval1.min, interval2.min), Min(interval1.max, interval2.max)};
}

template <typename T>
constexpr bool Overlaps(const Interval<T>& interval1, const Interval<T>& interval2)
{
    return (interval1.min <= interval2.max) && (interval2.min <= interval1.max);
}

template <typename T, std::size_t dims>
constexpr bool Overlaps(const BBox<T, dims>& bbox1, const BBox<T, dims>& bbox2)
{
    for(std::size_t i = 0; i < dims; ++i)
        if(!Overlaps(bbox1.intervals.at(i), bbox2.intervals.at(i)))
            return false;
    return true;
}

// return if interval 2 is completely enveloped by interval 1
template <typename T>
constexpr bool Includes(const Interval<T>& interval1, const Interval<T>& interval2)
{

    return (interval1.min <= interval2.min) && (interval1.max >= interval2.max);
}

// return if BBox 2 is completely enveloped by BBox 1
template <typename T, std::size_t dims>
constexpr bool Includes(const BBox<T, dims>& bbox1, const BBox<T, dims>& bbox2)
{
    for(std::size_t i = 0; i < dims; ++i)
        if(!Includes(bbox1.intervals.at(i), bbox2.intervals.at(i)))
            return false;
    return true;
}

template <typename T>
constexpr Interval<T> Combine(const Interval<T>& interval1, const Interval<T>& interval2)
{
    return Interval<T>{Min(interval1.min, interval2.min), Max(interval1.max, interval2.max)};
}

template <typename T, std::size_t dims>
constexpr BBox<T, dims> Combine(const BBox<T, dims>& bbox1, const BBox<T, dims>& bbox2)
{
    BBox<T, dims> bbox{};
    for(std::size_t i = 0; i < dims; ++i)
        bbox.intervals[i] = Combine(bbox1.intervals.at(i), bbox2.intervals.at(i));
    return bbox;
}

template <typename T, std::size_t dims>
constexpr T Volume(const BBox<T, dims>& bbox)
{
    T volume{1.};
    for(std::size_t i = 0; i < dims; ++i)
        volume *= (bbox.intervals[i].max - bbox.intervals[i].min);
    return volume;
}

// http://delab.csd.auth.gr/papers/TRSurveyRtree03_mnpt.pdf
// http://www-db.deis.unibo.it/courses/SI-LS/papers/Gut84.pdf

template <typename TObject, std::size_t dims, typename TLambda>
class BBoxTree
{
    constexpr static std::size_t Dims = dims;
    using Box = BBox<double, dims>;
    using Key = std::size_t;

    Key mKeyCounter{0};
    Key NewKey()
    {
        auto key = mKeyCounter;
        mKeyCounter++;
        return key;
    }

    // A node can either be a leaf or a branch.
    // a leaf has an object, a branch has children.
    struct Branch
    {
        std::array<Key, 2> children{};
    };
    struct Leaf
    {
        TObject* object{nullptr};
    };
    struct Node
    {
        Node() = default;
        Node(const Box& box, const Leaf& leaf)
            : box(box)
            , data{leaf}
        { }
        Node(const Box& box, const Branch& branch)
            : box(box)
            , data{branch}
        { }
        bool IsLeaf() const { return std::holds_alternative<Leaf>(data); }
        Key parent{};
        Box box{};
        std::variant<Branch, Leaf> data{};
    };

    TLambda mBoxLambda; // todo: check that it takes in a TObject& and returns a Box
    std::unordered_map<Key, Node> mData{};

    Key PlaceInbetween(const Box& box, const Key parentKey, const Key childKey, TObject& object)
    {
        // child is one of the children of parent.
        // Let parent point to a new in-between branch node,
        // this branch node points to both the old child, and a newly created one

        const auto branchkey = NewKey();
        const auto leafkey = NewKey();

        const auto combinedBBox = Combine(mData[childKey].box, box);

        mData[branchkey] = Node{combinedBBox, Branch{leafkey, childKey}};
        mData[leafkey] = Node{box, Leaf{&object}};

        if(parentKey != childKey)
        {
            auto& branch = std::get<Branch>(mData[parentKey].data);
            if(branch.children[0] == childKey)
                branch.children[0] = branchkey;
            else
                branch.children[1] = branchkey;
        }
        return leafkey;
    }

    Key InsertRecursive(const Box& box, const Key currentKey, TObject& object)
    {
        const auto& node = mData.at(currentKey);

        if(node.IsLeaf())
        {
            // make a new leaf node, combine the two in one branch
            const auto parentKey = node.parent;
            return PlaceInbetween(box, node.parent, currentKey, object);
        }
        else
        {
            // check if box is inside either of the children.
            // if not, pick the closest child, combine them in 1 branch
            const auto& branch = std::get<Branch>(node.data);
            const auto& bbox1 = mData[branch.children[0]].box;
            const auto& bbox2 = mData[branch.children[1]].box;
            const bool included1 = Includes(bbox1, box);
            const bool included2 = Includes(bbox2, box);
            if(included1 && !included2)
                return InsertRecursive(box, branch.children[0], object);
            else if(!included1 && included2)
                return InsertRecursive(box, branch.children[1], object);
            else if(!included1 && !included2)
            {
                // Not inside either. Put a new branch in between, add new leaf
                const auto volume1 = Volume(Combine(box, bbox1));
                const auto volume2 = Volume(Combine(box, bbox2));
                if(volume1 < volume2)
                    return PlaceInbetween(box, currentKey, branch.children[0], object); // combine with 1
                else
                    return PlaceInbetween(box, currentKey, branch.children[1], object);
            }
            else
            {
                // todo: box is inside both children, choose best
                const auto combined1 = Combine(box, bbox1);
                const auto combined2 = Combine(box, bbox2);
                const auto volume1 = Volume(combined1);
                const auto volume2 = Volume(combined2);
                if(volume1 < volume2)
                    return {};
                else
                    return {};
            }
        }
    }

public:
    BBoxTree(TLambda lambda)
        : mBoxLambda(lambda)
    { }

    Key Insert(TObject& object)
    {
        const auto box = mBoxLambda(object); //

        if(mData.empty())
        {
            const auto key = NewKey();
            Leaf leaf{&object};
            Node node{box, leaf};
            mData[key] = node;
            return key;
        }
        else
        {
            return InsertRecursive(box, mData.begin()->first, object);
        }
    }

    bool Remove(const TObject& object)
    {
        return false; //
    }

    // todo: make a Collisions(Box& box) function that returns a view of all leafs that might collide
};

//template <typename TKey, std::size_t dims, typename TLambda>
//auto GetTree(TLambda lambda)
//{
//    return BBoxTree{lambda};
//}

} // namespace geom
} // namespace vic