#pragma once

#include "vic/geometry/geometry.h"
#include "vic/utils.h"

#include <map>
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
        Node(const Key parent, const Box& box, const Leaf& leaf)
            : parent(parent)
            , box(box)
            , data{leaf}
        { }
        Node(const Key parent, const Box& box, const Branch& branch)
            : parent(parent)
            , box(box)
            , data{branch}
        { }
        bool IsLeaf() const { return std::holds_alternative<Leaf>(data); }
        Key parent{};
        Box box{};
        std::variant<Branch, Leaf> data{};
    };

    // todo: check that it takes in a TObject& and returns a Box
    // todo: decide if we want to pass a lambda for the bbox,
    // i did it so that we can easily update the tree when objects move,
    // but maybe i should just keep it out of scope for now.
    TLambda mBoxLambda;

    std::unordered_map<Key, Node> mData{};
    Key mRoot{};

    void UpdateBBoxes(const Key key)
    {
        // iterate upwards from the leaf node. bboxes are re-calculated.
        // key should point to the first non-leaf node, so we don't have to check it in the loop
        Key iterKey = key;
        while(true)
        {
            auto& node = mData[iterKey];

            assert(!node.IsLeaf());

            const auto& branch = std::get<Branch>(node.data);
            const auto& box1 = mData[branch.children[0]].box;
            const auto& box2 = mData[branch.children[1]].box;

            const auto newbox = Combine(box1, box2);
            if(Includes(node.box, newbox))
                break; // old box contains new box, we are done
            node.box = newbox;

            Key parentKey = node.parent;
            if(parentKey == iterKey)
                break; // root node, we are done

            iterKey = parentKey;
        }
    }

    Key PlaceInbetween(const Box& box, const Key parentKey, const Key childKey, TObject& object)
    {
        // child is one of the children of parent.
        // Let parent point to a new in-between branch node,
        // this branch node points to both the old child, and a newly created one

        const auto branchkey = NewKey();
        const auto leafkey = NewKey();

        const auto combinedBBox = Combine(mData[childKey].box, box);

        const Branch newbranch{leafkey, childKey};
        if(parentKey != childKey) // normal case: put new node in between parent and child
        {
            mData[branchkey] = Node{parentKey, combinedBBox, newbranch};

            auto& branch = std::get<Branch>(mData[parentKey].data);

            // find index that should be replaced without branching
            const std::size_t idx = std::size_t{branch.children[0] != childKey};

            branch.children[idx] = branchkey;
        }
        else // special case: putting leaf right next to the (previous) root
        {
            mRoot = branchkey;
            mData[branchkey] = Node{branchkey, combinedBBox, newbranch};
        }
        mData[childKey].parent = branchkey;
        mData[leafkey] = Node{branchkey, box, Leaf{&object}};

        return leafkey;
    }

    Key InsertRecursive(const Box& box, const Key currentKey, TObject& object)
    {
        const auto& node = mData.at(currentKey);

        if(node.IsLeaf())
            return PlaceInbetween(box, node.parent, currentKey, object);

        // check if box is inside either of the children.
        // if not, pick the closest child, combine them in 1 branch
        const auto& branch = std::get<Branch>(node.data);
        const auto& bbox1 = mData[branch.children[0]].box;
        const auto& bbox2 = mData[branch.children[1]].box;
        const bool included1 = Includes(bbox1, box);
        const bool included2 = Includes(bbox2, box);
        //if(included1 && !included2)
        //    return InsertRecursive(box, branch.children[0], object);
        //else if(!included1 && included2)
        //    return InsertRecursive(box, branch.children[1], object);
        if(included1 != included2)
        {
            const std::size_t idx = included2; // if 2 is included, cast true to index 1
            return InsertRecursive(box, branch.children[idx], object);
        }
        else if(included1 && included2)
        {
            // todo: box is inside both children, choose best,
            // Right now we choose the smallest box
            const std::size_t idx = Volume(bbox1) > Volume(bbox2);
            return InsertRecursive(box, branch.children[idx], object);
        }
        else //  if(!included1 && !included2)
        {
            // Not inside either. Choose box that increases the least

            const std::size_t idx = mKeyCounter % 2;
            return PlaceInbetween(box, currentKey, branch.children[idx], object);

            //const auto change1 = Volume(Combine(box, bbox1)) - Volume(bbox1);
            //const auto change2 = Volume(Combine(box, bbox2)) - Volume(bbox2);
            //// const std::size_t idx = change1 < change2 ? 0 : 1;
            //const std::size_t idx = change1 > change2;
            // return PlaceInbetween(box, currentKey, branch.children[idx], object);

            //const auto change1 = Volume(Combine(box, bbox1));
            //const auto change2 = Volume(Combine(box, bbox2));
            //// const std::size_t idx = change1 < change2 ? 0 : 1;
            //const std::size_t idx = change1 > change2;
            // return PlaceInbetween(box, currentKey, branch.children[idx], object);
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
            Node node{key, box, leaf};
            mData[key] = node;
            mRoot = key;
            return key;
        }
        else
        {
            const auto key = InsertRecursive(box, mRoot, object);
            UpdateBBoxes(mData[key].parent);
            return key;
        }
    }

    bool Remove(const TObject& object)
    {
        return false; //
    }

    // todo: make a Collisions(Box& box) function that returns a view of all leafs that might collide

    void Collisions(const Box& box) { }
};

//template <typename TKey, std::size_t dims, typename TLambda>
//auto GetTree(TLambda lambda)
//{
//    return BBoxTree{lambda};
//}

} // namespace geom
} // namespace vic