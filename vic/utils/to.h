#pragma once

namespace vic
{

// default template for a To(...) cast. Overload in other files
template <class TResult, class TInput>
TResult To(const TInput& input) = delete;

} // namespace vic