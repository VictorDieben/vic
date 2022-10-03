#pragma once
#pragma once

#include <cmath>

#include "vic/machine_learning/layer.h"

#include "vic/linalg/add.h"
#include "vic/linalg/matmul.h"
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
    BaseMapping(BaseLayer<T>& from, BaseLayer<T>& to)
        : mFrom(from)
        , mTo(to)
    { }
    
    virtual void Initialize() = 0;
    virtual void Calculate() = 0;

protected:
    BaseLayer<T>& mFrom;
    BaseLayer<T>& mTo;
};

// simple mapping. just a matrix.
template <typename T>
class Mapping : public BaseMapping<T>
{
public:
    using BaseMapping<T>::mFrom;
    using BaseMapping<T>::mTo;

    Mapping(BaseLayer<T>& from, BaseLayer<T>& to)
        : BaseMapping<T>(from, to)
    {
        Initialize();
    }

    void Initialize() override
    { 
        mMatrix = vic::linalg::MatrixDynamic<T>(mFrom.Size(), mTo.Size());
        mBiases = vic::linalg::VectorDynamic<T>(mTo.Size());
    }

    void Calculate() override
    {
        const auto result = vic::linalg::Add(vic::linalg::Matmul(mMatrix, mFrom.ToVector()), mBiases);

        // todo: workaround to copy matrix with 1 column to a vector
        vic::linalg::VectorDynamic<T> vec{result};

        mTo.SetVector(vec);
    }

private:
    vic::linalg::MatrixDynamic<T> mMatrix;

    vic::linalg::VectorDynamic<T> mBiases;
};

} // namespace ml
} // namespace vic