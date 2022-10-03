#pragma once
#pragma once

#include <cmath>

#include "vic/machine_learning/layer.h"

#include "vic/linalg/matrices_dynamic.h"

namespace vic
{
namespace ml
{

// base for mapping class. This way, we can put the mapping and hidden layers on the gpu if we want
template <typename T>
class BaseMapping
{
public:
    BaseMapping(const BaseLayer<T>& from, const BaseLayer<T>& to)
        : mFrom(from)
        , mTo(to)
    { }

private:
    const BaseLayer<T>& mFrom;
    const BaseLayer<T>& mTo;
};

// simple mapping. just a matrix.
template <typename T>
class Mapping : public BaseMapping<T>
{
public:
    Mapping(const BaseLayer<T>& from, const BaseLayer<T>& to)
        : BaseMapping<T>(from, to)
    {
        mMatrix = vic::linalg::MatrixDynamic<T>(from.Size(), to.Size());
    }

private:
    vic::linalg::MatrixDynamic<T> mMatrix;
};

} // namespace ml
} // namespace vic