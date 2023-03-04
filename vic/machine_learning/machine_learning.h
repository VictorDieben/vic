#pragma once

#include <memory>
#include <vector>

#include "vic/machine_learning/definitions.h"
#include "vic/machine_learning/layer.h"
#include "vic/machine_learning/mapping.h"

#include "vic/linalg/linalg.h"

namespace vic
{
namespace ml
{

//
//template <typename T>
//class BaseSystem
//{
//public:
//    BaseSystem() = default;
//    virtual void Initialize(const std::vector<std::size_t>& layers) = 0;
//
//private:
//};
//
//template <typename T>
//class System : public BaseSystem<T>
//{
//public:
//    System() = default;
//    System(std::vector<std::size_t> layers)
//        : BaseSystem<T>()
//    {
//        Initialize(layers);
//    }
//
//    void Initialize(const std::vector<std::size_t>& layers) override
//    {
//        mMappings.clear(); // <-- should be done first
//        mLayers.clear();
//        for(const auto& size : layers)
//            mLayers.push_back(std::make_unique<Layer<T>>(size));
//
//        for(std::size_t i = 0; i < mLayers.size() - 1; ++i)
//            mMappings.push_back(std::make_unique<Mapping<T>>(*(mLayers.at(i)), *(mLayers.at(i + 1))));
//    }
//
//private:
//    // list of layers, each layer contains a list of coefficients
//    std::vector<std::unique_ptr<BaseLayer<T>>> mLayers{};
//
//    // list of mappings between layers. each mapping contains weights and biasses
//    std::vector<std::unique_ptr<BaseMapping<T>>> mMappings{};
//};

} // namespace ml
} // namespace vic