// This file is part of Blend2D project <https://blend2d.com>
//
// See blend2d.h or LICENSE.md for license and copyright information
// SPDX-License-Identifier: Zlib

#include "api-build_p.h"
#include "api-impl.h"
#include "polygonclipper.h"
#include "polygonclipper_p.h"

BLPolygonClipper::BLPolygonClipper() :
    _impl(new bl::PolygonClipperImpl()) {

}

BLPolygonClipper::~BLPolygonClipper() {
    delete _impl;
}

namespace bl {

static BL_INLINE double getSignedArea(const BLPoint& p1, const BLPoint& p2, const BLPoint& p3) {
    return (p1.x * p2.y - p1.x * p3.y + p2.x * p3.y - p2.x * p1.y + p3.x * p1.y - p3.x * p2.y) * 0.5;
}

static BL_INLINE bool isClockwise(const BLPoint& p1, const BLPoint& p2, const BLPoint& p3) {
    return getSignedArea(p1, p2, p3) < 0.0;
}

static BL_INLINE bool isCounterClockwise(const BLPoint& p1, const BLPoint& p2, const BLPoint& p3) {
    return getSignedArea(p1, p2, p3) > 0.0;
}

static BL_INLINE bool isCollinear(const SweepEvent* a, const SweepEvent* b, double epsilon) {
    const double area1 = getSignedArea(a->_pt, a->_opposite->_pt, b->_pt);
    const double area2 = getSignedArea(a->_pt, a->_opposite->_pt, b->_opposite->_pt);
    const double totalArea = blAbs(area1) + blAbs(area2);
    return totalArea < epsilon;
}

struct SweepEventComparator {
    int operator()(const SweepEvent* a, const SweepEvent* b) const noexcept {
        if (a == b)
            return 0;

        // Compare x-coordinates first as we are sweeping from left to right.
        if (a->_pt.x < b->_pt.x)
            return -1;
        if (a->_pt.x > b->_pt.x)
            return 1;

        // If we have the same coordinate, we process the point with the lower y-coordinate first.
        if (a->_pt.y < b->_pt.y)
            return -1;
        if (a->_pt.y > b->_pt.y)
            return 1;

        // The point is the same in both events. We process right endpoint events first.
        if (a->isLeft() != b->isLeft())
            return a->isLeft() ? 1 : -1;

        // Otherwise, the order should remain as it is in the status line.
        return a->isPointAbove(b->_pt) ? -1 : 1;
    }
};

struct StatusLineComparator {
    BL_INLINE BL_CONSTEXPR StatusLineComparator(double epsilon) : _epsilon(epsilon) { }

    int operator()(const SweepEvent* a, const SweepEvent* b) const noexcept {
        if (a == b)
            return 0;

        SweepEventComparator sweepEventComparator;

        if (!isCollinear(a, b, _epsilon)) {
            // Segments are not collinear

            // Both segments start at the same point. Determine which line is above the other
            // in the sweep line by considering the second point of each segment. If b's right
            // point is above the line segment defined by points (a->_pt, a->_opposite->_pt),
            // it indicates that segment a precedes segment b in the status line.
            if (a->_pt == b->_pt)
                return a->isPointAbove(b->_opposite->_pt) ? -1 : 1;

            // Test whether event a is inserted before event b in the priority queue.
            // If a < b, then test whether b's start point is above the line segment
            // defined by points (a->_pt, a->_opposite->_pt). If so, a must precede
            // b in the status line. Otherwise, b must precede a in the status line.
            if (sweepEventComparator(a, b) < 0) {
                return a->isPointAbove(b->_pt) ? -1 : 1;
            } else {
                return b->isPointAbove(a->_pt) ? 1 : -1;
            }
        } else {
            // Segments are collinear
            if (a->_pt == b->_pt) {
                if (a->_opposite->_pt == b->_opposite->_pt) {
                    return a->isSubject() ? -1 : 1;
                } else {
                    return sweepEventComparator(a->_opposite, b->_opposite);
                }
            } else {
                return sweepEventComparator(a, b);
            }
        }

        return 0;
    }

    double _epsilon = 0.0;
};

PolygonClipperImpl::PolygonClipperImpl() noexcept :
    _memoryAllocator(MEMORY_BLOCK_SIZE) {

}

BLBooleanOperator PolygonClipperImpl::getOperator() const noexcept {
    return _operator;
}

void PolygonClipperImpl::setOperator(BLBooleanOperator newOperator) noexcept {
    _operator = newOperator;
}

SweepEvent* PolygonClipperImpl::allocSweepEvent() noexcept {
    SweepEvent* event = _sweepEventPool.alloc(_memoryAllocator);
    new (event) SweepEvent();
    return event;
}

void PolygonClipperImpl::freeSweepEvent(SweepEvent* event) noexcept {
    event->~SweepEvent();
    _sweepEventPool.free(event);
}

SweepEventNode* PolygonClipperImpl::allocSweepEventNode(SweepEvent* event) noexcept {
    SweepEventNode* node = _sweepEventNodePool.alloc(_memoryAllocator);
    new (node) SweepEventNode(event);
    return node;
}

void PolygonClipperImpl::freeSweepEventNode(SweepEventNode* node) noexcept {
    node->~SweepEventNode();
    _sweepEventNodePool.free(node);
}

Segment SweepEvent::getSegment() const noexcept {
    if (isLeft()) {
        return Segment(_pt, _opposite->_pt);
    } else {
        return Segment(_opposite->_pt, _pt);
    }
}

bool SweepEvent::isPointAbove(const BLPoint& pt) const noexcept
{
    if (isLeft())
        return isCounterClockwise(_pt, _opposite->_pt, pt);
    else
        return isCounterClockwise(_opposite->_pt, _pt, pt);
}

bool SweepEvent::isPointBelow(const BLPoint& pt) const noexcept
{
    if (isLeft())
        return isClockwise(_pt, _opposite->_pt, pt);
    else
        return isClockwise(_opposite->_pt, _pt, pt);
}

} // {bl}
