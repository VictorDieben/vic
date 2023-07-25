#pragma once

#include <vector>

namespace vic
{

template <typename T, typename TFunctor>
void ConstrainedPermutationsRecursive(const std::vector<T>& objects, //
                                      const std::vector<std::pair<T, T>>& constriants,
                                      TFunctor functor) // first < second
{
    //
}

template <typename T, typename TFunctor>
void ConstrainedPermutations(const std::vector<T>& objects, //
                             const std::vector<std::pair<T, T>>& constriants,
                             TFunctor functor) // first < second
{
    //
}
} // namespace vic