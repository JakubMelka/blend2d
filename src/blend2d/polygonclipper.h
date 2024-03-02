// This file is part of Blend2D project <https://blend2d.com>
//
// See blend2d.h or LICENSE.md for license and copyright information
// SPDX-License-Identifier: Zlib

#ifndef BLEND2D_POLYGONCLIPPER_H_INCLUDED
#define BLEND2D_POLYGONCLIPPER_H_INCLUDED

#include "api.h"

//! Defines boolean operators.
BL_DEFINE_ENUM(BLBooleanOperator) {
    //! Creates the union of defined areas.
    BL_BOOLEAN_OPERATOR_UNION = 0,
    //! Creates the intersection of defined areas.
    BL_BOOLEAN_OPERATOR_INTERSECTION = 1,
    //! Creates the difference between defined areas.
    BL_BOOLEAN_OPERATOR_DIFFERENCE = 2,
    //! Creates the symmetric difference of defined areas.
    BL_BOOLEAN_OPERATOR_SYMMETRIC_DIFFERENCE = 3,

    //! Maximum value for BLBooleanOperator.
    BL_BOOLEAN_OPERATOR_MAX_VALUE = 3,
    //! Forces this enumeration to be a 32-bit unsigned integer.
    BL_FORCE_ENUM_UINT32(BL_BOOLEAN_OPERATOR)
};

namespace bl {
class PolygonClipperImpl;
}

class BLPolygonClipper {
public:
    BLPolygonClipper();
    ~BLPolygonClipper();

private:
    bl::PolygonClipperImpl* _impl;
};

#endif // BLEND2D_POLYGONCLIPPER_H_INCLUDED
