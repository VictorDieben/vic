#pragma once

#include <string>

namespace vic
{

// error type, for use with std::expected
template <typename TEnum>
    requires std::is_enum_v<TEnum>
struct StaticError
{
    TEnum code;
    std::string message;
};

} // namespace vic