#pragma once

#include <cmath>

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

private:
    vic::linalg::VectorDynamic<T> mData;
};
} // namespace ml
} // namespace vic