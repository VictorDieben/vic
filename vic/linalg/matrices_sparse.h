#pragma once

#include "vic/linalg/matrices.h"
#include "vic/linalg/traits.h"
#include "vic/memory/flat_map.h"

#include <vector>

namespace vic
{
namespace linalg
{

// sparse matrix (row mayor)
// TODO: verify that it is _actually_ row mayor, right now the index pair is automatically sorted
template <typename T>
class MatrixSparse
{
public:
    using DataType = T;
    constexpr static auto Size = ESizeType::Dynamic;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Sparse;

    using KeyType = std::pair<std::size_t, std::size_t>;

    MatrixSparse() = default;
    MatrixSparse(const std::size_t rows, const std::size_t cols)
        : mRows(rows)
        , mColumns(cols)
    { }

    template <typename TMat>
    requires ConceptMatrix<TMat> //
    MatrixSparse(const TMat& matrix)
    {
        mRows = matrix.GetRows();
        mColumns = matrix.GetColumns();
        for(std::size_t j = 0; j < mColumns; ++j)
            for(std::size_t i = 0; i < mRows; ++i)
                if(const auto val = matrix.Get(i, j); val != 0.) // TODO: comparisson to 0? maybe define delta
                    At(i, j) = val;
    }

    std::size_t GetRows() const { return mRows; }
    std::size_t GetColumns() const { return mColumns; }

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));

        const auto it = mData.find(KeyType{i, j});
        if(it == mData.end())
            return 0.;
        return it->second;
    }

    constexpr T& At(const std::size_t i, const std::size_t j)
    {
        assert(((i < GetRows()) && (j < GetColumns())));

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

private:
    // vic::memory::FlatMap<KeyType, DataType> mData{};
    std::map<KeyType, DataType> mData{};

    std::size_t mRows{0};
    std::size_t mColumns{0};
};

template <typename T>
struct is_sparse<MatrixSparse<T>>
{
    static constexpr bool value = true;
};

} // namespace linalg
} // namespace vic