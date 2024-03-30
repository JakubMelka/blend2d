// This file is part of Blend2D project <https://blend2d.com>
//
// See blend2d.h or LICENSE.md for license and copyright information
// SPDX-License-Identifier: Zlib

#include "api-build_test_p.h"
#if defined(BL_TEST)

#include "api-impl.h"
#include "blend2d/polygonclipper.h"

// PolygonClipper - Tests
// ======================

namespace bl {
namespace Tests {

static void testPolygonClipperBasic() noexcept {
    INFO("Testing Polygon Clipper - basic");

    BLPolygonClipper polygonClipper;
    polygonClipper;
}

UNIT(polygon_clipper, BL_TEST_GROUP_POLYGON_CLIPPER) {
    testPolygonClipperBasic();
}


} // {Tests}
} // {bl}

#endif // BL_TEST
