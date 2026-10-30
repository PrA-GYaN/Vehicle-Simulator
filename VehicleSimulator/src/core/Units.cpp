#include "core/Units.h"

// Units.h is header-only by design. This translation unit exists so the
// header is compiled at least once on its own, catching syntax errors that
// would otherwise only surface at the first include site.
namespace vsim {
namespace {
static_assert(degToRad(180.0) > 3.14 && degToRad(180.0) < 3.15,
              "degree/radian conversion is broken");
static_assert(clampd(5.0, 0.0, 1.0) == 1.0, "clampd is broken");
static_assert(clampAbs(-9.0, 2.0) == -2.0, "clampAbs is broken");
} // namespace
} // namespace vsim
