#pragma once

namespace vic
{

template <typename T>
struct Result : public T
{
    // UniqueLambda does nothing. But every instance of a lambda will get a unique type,
    // and therefore every definition of a UniqueType<T> will be its own type.
    // Useful for making unique mutex types.
};

} // namespace vic