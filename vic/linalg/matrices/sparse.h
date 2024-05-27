#pragma once

#include "vic/linalg/matrices/base.h"
#include "vic/linalg/traits.h"
#include "vic/memory/flat_map.h"

#include <map>
#include <unordered_map>
#include <vector>

namespace vic
{
namespace linalg
{
namespace detail
{

template <typename T, typename TShape>
class MatrixSparse : public MatrixBaseSelector<TShape>
{
public:
    using DataType = T;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Sparse;

    using KeyType = std::pair<Row, Col>;
    using MatrixBaseType = MatrixBaseSelector<TShape>;

    constexpr MatrixSparse(const Row rows, const Col cols)
        : MatrixBaseType(rows, cols)
    { }

    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));

        const auto it = mData.find(KeyType{i, j});
        if(it == mData.end())
            return 0.;
        return it->second;
    }

    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return mData[KeyType{i, j}]; // NOTE: creates a new entry if it didn't exist
    }

    void Prune()
    {
        // todo: iterate over mData, remove zeros
    }

    auto begin() { return mData.begin(); }
    auto end() { return mData.end(); }

    auto begin() const { return mData.begin(); }
    auto end() const { return mData.end(); }

    auto Reserve(const std::size_t size) { mData.Reserve(size); }

private:
    std::map<KeyType, T> mData{};
};

} // namespace detail

template <typename T, typename TShape = Shape<UnknownSize, UnknownSize>>
using Sparse = TypeSelector<TShape, // todo: specific types for different shapes
                            detail::MatrixSparse<T, TShape>,
                            detail::MatrixSparse<T, TShape>,
                            detail::MatrixSparse<T, TShape>,
                            detail::MatrixSparse<T, TShape>>;

template <typename T, Row rows, Col cols>
using SparseMxN = Sparse<T, Shape<rows, cols>>;

template <typename TMat>
    requires ConceptMatrix<TMat>
constexpr auto ToSparse(const TMat& mat)
{
    Sparse<typename TMat::DataType, typename TMat::ShapeType> res{mat.GetRows(), mat.GetColumns()};

    // note: add in row mayor order
    for(Row i = 0; i < mat.GetRows(); ++i)
        for(Col j = 0; j < mat.GetColumns(); ++j)
            if(const auto val = mat.Get(i, j); val != 0.)
                res.At(i, j) = val;

    return res;
}

} // namespace linalg
} // namespace vic