#pragma once

#include <cmath>
#include <cassert>

#include "vic/linalg/matrices_dynamic.h"
#include "vic/machine_learning/definitions.h"

namespace vic
{
namespace ml
{

// base class for machine learning layers
template <typename T>
class BaseLayer
{
public:
    BaseLayer(std::size_t size) { }

    virtual std::size_t Size() const = 0;

    // todo: based on type of layer and mapping, we might not want to use a ToVector().
    // for instance, if the data is located on the gpu
    virtual vic::linalg::VectorDynamic<T> ToVector() const = 0;
    virtual void SetVector(const vic::linalg::VectorDynamic<T>& vec) = 0;

private:
};

// simple implementation of a machine learning layer
template <typename T>
class Layer : public BaseLayer<T>
{
public:
    Layer(std::size_t size)
        : BaseLayer<T>(size)
    {
        mData = vic::linalg::VectorDynamic<T>(size); //
    }

    std::size_t Size() const override { return mData.GetRows(); }

    vic::linalg::VectorDynamic<T> ToVector() const override { return mData; }

    void SetVector(const vic::linalg::VectorDynamic<T>& vec) override
    {
        assert(vec.GetRows() == mData.GetRows());
        mData = vec;
    }

private:
    vic::linalg::VectorDynamic<T> mData;
};
} // namespace ml
} // namespace vic