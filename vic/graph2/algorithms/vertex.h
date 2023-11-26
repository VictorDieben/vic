#pragma once

#include "vic/memory/refcounter.h"

namespace vic
{
namespace graph2
{
struct Vertex : public IntrusiveRef<Vertex>
{ };
} // namespace graph2
} // namespace vic