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
        return a->isPointAbove(b->_opposite->_pt) ? -1 : 1;
    }
};

struct SweepEventNodeComparator {
    int operator()(const SweepEventNode& a, const SweepEventNode& b) const noexcept {
        return SweepEventComparator()(a._event, b._event);
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

void PolygonClipperImpl::addPolygonSegment(const Segment& segment, bool isSubject) noexcept {
    SweepEvent* ev1 = allocSweepEvent();
    SweepEvent* ev2 = allocSweepEvent();

    ev1->_pt = segment._p1;
    ev2->_pt = segment._p2;

    ev1->_opposite = ev2;
    ev2->_opposite = ev1;

    if (isSubject) {
        ev1->_flags |= SweepEventFlags::kIsSubject;
        ev2->_flags |= SweepEventFlags::kIsSubject;
    }

    if (SweepEventComparator()(ev1, ev2) < 0)
        ev1->_flags |= SweepEventFlags::kIsLeft;
    else
        ev2->_flags |= SweepEventFlags::kIsLeft;

    SweepEventNode* node1 = allocSweepEventNode(ev1);
    SweepEventNode* node2 = allocSweepEventNode(ev2);

    _q.insert(node1, SweepEventNodeComparator());
    _q.insert(node2, SweepEventNodeComparator());

    if (isSubject)
        ++_subjectEdgeCount;
    else
        ++_clippingEdgeCount;
}

BLResult PolygonClipperImpl::perform() noexcept
{
    size_t maxSweepEvents = calculateMaximumNumberOfSweepEvents();
    size_t currentSweepEventIndex = 0;

    struct ClearGuard {
        BL_INLINE ClearGuard(PolygonClipperImpl* impl) : _impl(impl) { }
        BL_INLINE ~ClearGuard() { _impl->reset(); }

        PolygonClipperImpl* _impl = nullptr;
    } guard(this);

    while (!_q.empty()) {
        if (++currentSweepEventIndex > maxSweepEvents) {
            return blTraceError(BL_ERROR_POLYGON_CLIPPER_MAX_SWEEP_EVENTS);
        }

        // Pop current event from the queue
        SweepEventNode* node = _q.pop();
        SweepEvent* event = node->_event;
        freeSweepEventNode(node);

        if (event->isLeft()) {
            StatusLineComparator comparator(_epsilon);

            // Start point of the segment. We must insert current
            // event into _s. Only sweep events of left (starting)
            // points of the segments are in _s.
            SweepEventNode* sNode = allocSweepEventNode(event);
            _s.insert(sNode, comparator);

            SweepEventNode* sPrev = _s.prev(sNode, comparator);
            SweepEventNode* sNext = _s.next(sNode, comparator);

            BLResult updateResult = updateFlags(sPrev, sNode);
            if (updateResult != BL_SUCCESS)
                return updateResult;

            findIntersections(sPrev, sNode);
            findIntersections(sNode, sNext);
        } else {
            // End point of the segment
        }
    }

    return BL_SUCCESS;
}

bool PolygonClipperImpl::isSelfOverlapping(SweepEventNode* sNode1, SweepEventNode* sNode2) const noexcept {
    return sNode1->_event->isSubject() == sNode2->_event->isSubject();
}

BLResult PolygonClipperImpl::updateFlags(SweepEventNode* sPrev, SweepEventNode* sNode) noexcept {
    if (!sPrev) {
        // This is an outer edge; simply clear the flags.
        sNode->_event->_flags &= ~(SweepEventFlags::kIsInOut | SweepEventFlags::kIsInside);
    } else if (!sPrev->_event->isSegmentNormal()) {
        if (isSelfOverlapping(sPrev, sNode)) {
            return blTraceError(BL_ERROR_POLYGON_POLYGON_SELF_OVERLAPS);
        }

        // At this point, things get more complex. The segments overlap with another segment.
        // Two edges of the same polygon overlapping implies an error; hence, overlapping
        // edges must belong to different polygons (subject or clipping). If there's no edge
        // before the previous one, set the flag kIsInside (since the previous edge belongs to another
        // polygon, indicating we are inside a polygon), and unset the flag kIsInOut (because the current polygon
        // is above the current edge represented by sNode).
        SweepEventNode* sPrevPrev = _s.prev(sPrev, StatusLineComparator(_epsilon));
        if (!sPrevPrev) {
            blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInside, true);
            blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInOut, false);
        }

        if (isSelfOverlapping(sPrev, sPrevPrev)) {
            return blTraceError(BL_ERROR_POLYGON_POLYGON_SELF_OVERLAPS);
        }

        if (sPrev->_event->isSubject() == sNode->_event->isSubject()) {
            blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInside, !sPrevPrev->_event->isInOut());
            blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInOut, !sPrev->_event->isInOut());
        } else {
            blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInside, !sPrev->_event->isInOut());
            blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInOut, !sPrevPrev->_event->isInOut());
        }
    } else if (sPrev->_event->isSubject() == sNode->_event->isSubject()) {
        // Both edges belong to the same polygon. Thus, if the previous edge is inside
        // another polygon, then the current edge is also inside that polygon.
        blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInside, sPrev->_event->isInside());

        // Since the edges belong to the same polygon, one edge implies the polygon
        // is below, and the other implies the polygon is above. This can be determined
        // using the isInOut flag.
        blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInOut, !sPrev->_event->isInOut());
    } else {
        // The previous edge is below the current edge. This means if the polygon is above the previous edge,
        // the current edge is inside the other polygon, necessitating setting the flag kIsInside.
        blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInside, !sPrev->_event->isInOut());

        // If the current polygon is below the current edge and the previous edge is inside the current polygon,
        // it signifies our polygon ends at the current edge.
        blSetFlag(sNode->_event->_flags, SweepEventFlags::kIsInOut, sPrev->_event->isInside());
    }

    return BL_SUCCESS;
}

size_t PolygonClipperImpl::calculateMaximumNumberOfSweepEvents() const noexcept {
    size_t result = 0;
    size_t totalEdgeCount = _subjectEdgeCount + _clippingEdgeCount;

    if (totalEdgeCount == 0)
        return result;

    // We need two sweep events for each polygon edge, considering both the start
    // and end points of the edge.
    result += 2 * totalEdgeCount;

    // Each segment intersection may create up to four new edges if the intersection
    // occurs in the middle of each segment, thereby splitting each segment into two.
    // It's necessary to account for the possibility of self-intersecting subject and
    // clipping polygons. Therefore, we will conservatively estimate that each edge
    // could intersect with all other edges.
    size_t numberOfPossibleIntersectionPoints = (totalEdgeCount * (totalEdgeCount - 1)) / 2;
    size_t numberOfSweepEventsFromIntersections = 4 * 2 * numberOfPossibleIntersectionPoints;

    result += numberOfSweepEventsFromIntersections;

    return result;
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
