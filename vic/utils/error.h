#pragma once

#include <ostream>
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

    TEnum Code() const noexcept { return code; }
    operator TEnum() const noexcept { return code; }

    std::ostream& operator<<(std::ostream& stream)
    {
        stream << "error<" << code << "; " << message << ">" << std::endl;
        return stream;
    }
};

} // namespace vic