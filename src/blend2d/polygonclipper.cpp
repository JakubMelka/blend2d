// This file is part of Blend2D project <https://blend2d.com>
//
// See blend2d.h or LICENSE.md for license and copyright information
// SPDX-License-Identifier: Zlib

#include "api-build_p.h"
#include "api-impl.h"
#include "polygonclipper.h"
#include "polygonclipper_p.h"
#include "geometry_p.h"
#include "matrix.h"

BLPolygonClipper::BLPolygonClipper() noexcept :
    _impl(nullptr) {
    _impl = reinterpret_cast<bl::PolygonClipperImpl*>(_buffer);

    void* staticData = _buffer + sizeof(bl::PolygonClipperImpl);
    constexpr size_t staticSize = MEMORY_BLOCK_SIZE - sizeof(bl::PolygonClipperImpl);
    blCallCtor(*_impl, MEMORY_BLOCK_SIZE, staticData, staticSize);
}

BLPolygonClipper::~BLPolygonClipper() noexcept {
    blCallDtor(*_impl);
}

void BLPolygonClipper::setOperator(BLBooleanOperator booleanOperator) noexcept {
    _impl->setOperator(booleanOperator);
}

const BLPath& BLPolygonClipper::getPath() const noexcept {
    return _impl->getPath();
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
    }

    inline int operator()(const SweepEventNode& a, const SweepEventNode& b) const noexcept {
        return operator()(a._event, b._event);
    }

    double _epsilon = 0.0;
};

PolygonClipperImpl::PolygonClipperImpl(size_t blockSize, void* staticData, size_t staticSize) noexcept :
    _memoryAllocator(blockSize, 1, staticData, staticSize),
    _connector(_memoryAllocator) {

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
    BLResult result = BL_SUCCESS;
    size_t maxSweepEvents = calculateMaximumNumberOfSweepEvents();
    size_t currentSweepEventIndex = 0;

    struct ClearGuard {
        BL_INLINE ClearGuard(PolygonClipperImpl* impl) : _impl(impl) { }
        BL_INLINE ~ClearGuard() { _impl->reset(); }

        PolygonClipperImpl* _impl = nullptr;
    } guard(this);

    while (!_q.empty() && result == BL_SUCCESS) {
        if (++currentSweepEventIndex > maxSweepEvents)
            return blTraceError(BL_ERROR_POLYGON_CLIPPER_MAX_SWEEP_EVENTS);

        // Pop current event from the queue
        SweepEventNode* node = _q.pop(SweepEventNodeComparator());
        SweepEvent* event = node->_event;
        freeSweepEventNode(node);

        if (event->isLeft()) {
            StatusLineComparator comparator(_epsilon);

            // Start point of the segment. We must insert current
            // event into _s. Only sweep events of left (starting)
            // points of the segments are in _s.
            SweepEventNode* sNode = allocSweepEventNode(event);
            _s.insert(sNode, comparator);
            sNode->_event->_nodeS = sNode;
            sNode->_event->_opposite->_nodeS = sNode;

            SweepEventNode* sPrev = _s.prev(sNode, comparator);
            SweepEventNode* sNext = _s.next(sNode, comparator);

            updateResult(result, updateFlags(sPrev, sNode));

            updateResult(result, findIntersections(sPrev, sNode));
            updateResult(result, findIntersections(sNode, sNext));
        } else {
            // End point of the segment
            SweepEventNode* sNode = event->_nodeS;

            StatusLineComparator comparator(_epsilon);
            SweepEventNode* sPrev = _s.prev(sNode, comparator);
            SweepEventNode* sNext = _s.next(sNode, comparator);

            // Add edge to polygon connector
            addResultEdge(event->_opposite);

            // Clear sweep events
            _s.remove(sNode, comparator);
            freeSweepEventNode(sNode);
            freeSweepEvent(event->_opposite);
            freeSweepEvent(event);

            updateResult(result, findIntersections(sPrev, sNext));
        }
    }

    return result;
}

void PolygonClipperImpl::reset() noexcept
{
    StatusLineComparator comparator(_epsilon);

    while (!_s.empty()) {
        SweepEventNode* node = _s.root();
        _s.remove(node, comparator);
        freeSweepEventNode(node);
    }

    while (!_q.empty()) {
        SweepEventNode* node = _q.pop(SweepEventNodeComparator());
        freeSweepEvent(node->_event);
        freeSweepEventNode(node);
    }

    _connector.reset();
}

bool PolygonClipperImpl::isSelfOverlapping(SweepEventNode* sNode1, SweepEventNode* sNode2) const noexcept {
    return sNode1->_event->isSubject() == sNode2->_event->isSubject();
}

BLResult PolygonClipperImpl::updateFlags(SweepEventNode* sPrev, SweepEventNode* sNode) noexcept {
    if (!sPrev) {
        // This is an outer edge; simply clear the flags.
        sNode->_event->_flags &= ~(SweepEventFlags::kIsInOut | SweepEventFlags::kIsInside);
    } else if (!sPrev->_event->isSegmentNormal()) {
        if (isSelfOverlapping(sPrev, sNode))
            return blTraceError(BL_ERROR_POLYGON_POLYGON_SELF_OVERLAPS);

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

        if (isSelfOverlapping(sPrev, sPrevPrev))
            return blTraceError(BL_ERROR_POLYGON_POLYGON_SELF_OVERLAPS);

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

BLResult PolygonClipperImpl::findIntersections(SweepEventNode* sPrev, SweepEventNode* sNode) noexcept
{
    if (!sPrev || !sNode)
        return BL_SUCCESS;

    BL_ASSERT(sPrev->_event->isLeft());
    BL_ASSERT(sNode->_event->isLeft());

    Segment s1(sPrev->_event->_pt, sPrev->_event->_opposite->_pt);
    Segment s2(sNode->_event->_pt, sNode->_event->_opposite->_pt);

    SegmentIntersection intersections;
    BLResult intersectionResult = calculateSegmentIntersections(intersections, s1, s2);

    if (intersectionResult != BL_SUCCESS)
        return intersectionResult;
    if (intersections.isNoIntersection())
        return BL_SUCCESS;

    if (intersections.isOverlapped()) {
        // Segments overlap. We must verify that they originate from
        // different polygons. We do not support overlapping segments from
        // the same polygon.
        if (sPrev->_event->isSubject() == sNode->_event->isSubject())
            return BL_ERROR_POLYGON_POLYGON_SELF_OVERLAPS;

        SweepEventFlags line1Flags = SweepEventFlags::kSegmentNonContributing;
        SweepEventFlags line2Flags = (sPrev->_event->isInOut() == sNode->_event->isInOut()) ? SweepEventFlags::kSegmentSameTransition : SweepEventFlags::kSegmentDifferentTransition;

        // Is one segment contained within the other?
        if (intersections.isLine1ContainedWithinLine2()) {
            sPrev->_event->_flags |= line1Flags;
            sPrev->_event->_opposite->_flags |= line1Flags;
        } else {
            // The line must be split. If the line needs to be divided into three segments,
            // it will be split into only two, with the second split deferred until the next
            // sweep event.

            if (blTestFlag(intersections.flags, SegmentIntersectionFlags::kIntersectionLine1Start)) {
                sPrev->_event->_flags |= line1Flags;
                sPrev->_event->_opposite->_flags |= line1Flags;
            }

            const bool useRight = !blTestFlag(intersections.flags, SegmentIntersectionFlags::kIntersectionLine1End);
            divideSegment(sPrev, useRight ? intersections.ptLeft1 : intersections.ptRight1);
        }

        // Is one segment contained within the other?
        if (intersections.isLine2ContainedWithinLine1()) {
            sNode->_event->_flags |= line2Flags;
            sNode->_event->_opposite->_flags |= line2Flags;
        } else {
            // The line must be split. If the line needs to be divided into three segments,
            // it will be split into only two, with the second split deferred until the next
            // sweep event.

            if (blTestFlag(intersections.flags, SegmentIntersectionFlags::kIntersectionLine2Start)) {
                sPrev->_event->_flags |= line2Flags;
                sPrev->_event->_opposite->_flags |= line2Flags;
            }

            const bool useRight = !blTestFlag(intersections.flags, SegmentIntersectionFlags::kIntersectionLine1End);
            divideSegment(sPrev, useRight ? intersections.ptLeft2 : intersections.ptRight2);
        }
    } else {
        if (blTestFlag(intersections.flags, SegmentIntersectionFlags::kIntersectionLine1Interior))
            divideSegment(sPrev, intersections.ptLeft1);
        if (blTestFlag(intersections.flags, SegmentIntersectionFlags::kIntersectionLine2Interior))
            divideSegment(sNode, intersections.ptLeft2);
    }

    return BL_SUCCESS;
}

void PolygonClipperImpl::divideSegment(SweepEventNode* sNode, const BLPoint& point) noexcept
{
    SweepEvent* e = sNode->_event;
    SweepEvent* l = allocSweepEvent();
    SweepEvent* r = allocSweepEvent();

    l->_opposite = e->_opposite;
    l->_pt = point;
    l->_flags = e->_flags & (SweepEventFlags::kIsSubject | SweepEventFlags::kIsInOut | SweepEventFlags::kIsInside);
    l->_flags |= SweepEventFlags::kIsLeft;

    r->_opposite = e;
    r->_pt = point;
    r->_flags = e->_flags & (SweepEventFlags::kIsSubject | SweepEventFlags::kIsInOut | SweepEventFlags::kIsInside);
    r->_nodeS = e->_nodeS;

    e->_opposite->_opposite = l;
    e->_opposite = r;

    _q.insert(allocSweepEventNode(l), SweepEventNodeComparator());
    _q.insert(allocSweepEventNode(r), SweepEventNodeComparator());
}

BLResult PolygonClipperImpl::calculateSegmentIntersections(SegmentIntersection& intersections, const Segment& s1, const Segment& s2) const noexcept
{
    double vx1, vy1, d1;
    double vx2, vy2, d2;
    bl::Geometry::lineEquation(s1._p1, s1._p2, vx1, vy1, d1);
    bl::Geometry::lineEquation(s2._p1, s2._p2, vx2, vy2, d2);

    BLMatrix2D equationMatrix(vx1, vy1, vx2, vy2, d1, d2);
    double determinant = equationMatrix.determinant();

    auto getLineParameter = [this](const Segment& segment, const BLPoint& p) -> double {
        BLPoint v = segment._p2 - segment._p1;
        if (blAbs(v.x) > blAbs(v.y))
            return (p.x - segment._p1.x) / (segment._p2.x - segment._p1.x);
        else
            return (p.y - segment._p1.y) / (segment._p2.y - segment._p1.y);
    };

    if (blAbs(determinant) < _epsilon) {
        // The segments are collinear. However, are they overlapping?
        // They are considered overlapping if their distance is less than epsilon.
        if (blAbs(d1 - d2) < _epsilon) {
            BLPoint v = s1._p2 - s1._p1;
            double angle = bl::Math::atan2(v.y, v.x);
            BLMatrix2D rotationMatrix;
            rotationMatrix.resetToRotation(-angle, s1._p1);

            Segment rs1(rotationMatrix.mapPoint(s1._p1), rotationMatrix.mapPoint(s1._p2));
            Segment rs2(rotationMatrix.mapPoint(s2._p1), rotationMatrix.mapPoint(s2._p2));

            double xMin1 = blMin(rs1._p1.x, rs1._p2.x);
            double xMax1 = blMax(rs1._p1.x, rs1._p2.x);
            double xMin2 = blMin(rs2._p1.x, rs2._p2.x);
            double xMax2 = blMax(rs2._p1.x, rs2._p2.x);

            double overlapStart = blMax(xMin1, xMin2);
            double overlapEnd = blMin(xMax1, xMax2);

            // Intervals can also overlap at a single point. However, since this is not significant
            // for our algorithm, we disregard such cases and focus only on non-empty intervals.
            if (overlapStart < overlapEnd - _epsilon) {
                BLMatrix2D rotationMatrixInverted = rotationMatrix;
                BLResult invertResult = rotationMatrixInverted.invert();
                if (invertResult != BL_SUCCESS)
                    return invertResult;

                BLPoint pt1 = rotationMatrixInverted.mapPoint(BLPoint(overlapStart, 0));
                BLPoint pt2 = rotationMatrixInverted.mapPoint(BLPoint(overlapEnd, 0));

                auto processIntersections = [&, this](const Segment& s, BLPoint& intPt1, BLPoint& intPt2,
                                                      SegmentIntersectionFlags flagStart,
                                                      SegmentIntersectionFlags flagInterior,
                                                      SegmentIntersectionFlags flagEnd) {
                    double lineParameter1 = getLineParameter(s, pt1);
                    double lineParameter2 = getLineParameter(s, pt2);

                    if (lineParameter1 > lineParameter2)
                        std::swap(lineParameter1, lineParameter2);

                    const double segmentLength = s.getLength();

                    const double lineOrdinate1 = segmentLength * lineParameter1;
                    const double lineOrdinate2 = segmentLength * lineParameter2;

                    // Clamp ordinates
                    lineParameter1 = clampParameter(lineParameter1, lineOrdinate1, segmentLength);
                    lineParameter2 = clampParameter(lineParameter2, lineOrdinate2, segmentLength);

                    if (lineParameter1 == 0.0) {
                        intersections.flags |= flagStart;
                        intPt1 = s._p1;
                    } else {
                        intersections.flags |= flagInterior;
                        intPt1 = pt1;
                    }

                    if (lineParameter2 == 1.0) {
                        intersections.flags |= flagEnd;
                        intPt2 = s._p2;
                    } else {
                        intersections.flags |= flagInterior;
                        intPt2 = s._p2;
                    }
                };

                processIntersections(s1, intersections.ptLeft1, intersections.ptLeft2,
                                     SegmentIntersectionFlags::kIntersectionLine1Start,
                                     SegmentIntersectionFlags::kIntersectionLine1Interior,
                                     SegmentIntersectionFlags::kIntersectionLine1End);
                processIntersections(s2, intersections.ptRight1, intersections.ptRight2,
                                     SegmentIntersectionFlags::kIntersectionLine2Start,
                                     SegmentIntersectionFlags::kIntersectionLine2Interior,
                                     SegmentIntersectionFlags::kIntersectionLine2End);

                intersections.flags |= SegmentIntersectionFlags::kOverlapped;
            }
        }
    } else {
        BLResult invertResult = equationMatrix.invert();
        if (invertResult != BL_SUCCESS)
            return invertResult;

        BLPoint pt(equationMatrix.m20, equationMatrix.m21);

        double lineParameter1 = getLineParameter(s1, pt);
        double lineParameter2 = getLineParameter(s2, pt);

        const double segment1Length = s1.getLength();
        const double segment2Length = s2.getLength();

        const double lineOrdinate1 = segment1Length * lineParameter1;
        const double lineOrdinate2 = segment2Length * lineParameter2;

        // Intersection point must lie on both lines.
        if (lineOrdinate1 < -_epsilon || lineOrdinate1 > segment1Length + _epsilon ||
            lineOrdinate2 < -_epsilon || lineOrdinate2 > segment2Length + _epsilon)
            return BL_SUCCESS;

        // Clamp ordinates
        lineParameter1 = clampParameter(lineParameter1, lineOrdinate1, segment1Length);
        lineParameter2 = clampParameter(lineParameter2, lineOrdinate2, segment2Length);

        if (lineParameter1 == 0.0) {
            intersections.flags |= SegmentIntersectionFlags::kIntersectionLine1Start;
            intersections.ptLeft1 = s1._p1;
            intersections.ptLeft2 = s1._p1;
        } else if (lineParameter1 == 1.0) {
            intersections.flags |= SegmentIntersectionFlags::kIntersectionLine1End;
            intersections.ptLeft1 = s1._p2;
            intersections.ptLeft2 = s1._p2;
        } else {
            intersections.flags |= SegmentIntersectionFlags::kIntersectionLine1Interior;
            intersections.ptLeft1 = pt;
            intersections.ptLeft2 = pt;
        }

        if (lineParameter2 == 0.0) {
            intersections.flags |= SegmentIntersectionFlags::kIntersectionLine2Start;
            intersections.ptRight1 = s2._p1;
            intersections.ptRight2 = s2._p1;
        } else if (lineParameter2 == 1.0) {
            intersections.flags |= SegmentIntersectionFlags::kIntersectionLine2End;
            intersections.ptRight1 = s2._p2;
            intersections.ptRight2 = s2._p2;
        } else {
            intersections.flags |= SegmentIntersectionFlags::kIntersectionLine2Interior;
            intersections.ptRight1 = pt;
            intersections.ptRight2 = pt;
        }
    }

    if (intersections.flags == SegmentIntersectionFlags::kNoFlags)
        intersections.flags |= SegmentIntersectionFlags::kNoIntersection;

    return BL_SUCCESS;
}

void PolygonClipperImpl::updateResult(BLResult& oldResult, BLResult newResult) const noexcept
{
    if (oldResult == BL_SUCCESS)
        oldResult = newResult;
}

double PolygonClipperImpl::clampParameter(double parameter, double ordinate, double segmentLength) const noexcept
{
    if (ordinate < _epsilon)
        parameter = 0.0;
    if (ordinate > segmentLength - _epsilon)
        parameter = 1.0;

    return parameter;
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
    blCallCtor(*event);
    return event;
}

void PolygonClipperImpl::freeSweepEvent(SweepEvent* event) noexcept {
    blCallDtor(*event);
    _sweepEventPool.free(event);
}

SweepEventNode* PolygonClipperImpl::allocSweepEventNode(SweepEvent* event) noexcept {
    SweepEventNode* node = _sweepEventNodePool.alloc(_memoryAllocator);
    blCallCtor(*node, event);
    return node;
}

void PolygonClipperImpl::freeSweepEventNode(SweepEventNode* node) noexcept {
    blCallDtor(*node);
    _sweepEventNodePool.free(node);
}

void PolygonClipperImpl::addResultEdge(SweepEvent* edge)
{
    BL_ASSERT(edge->isLeft());

    if (edge->isSegmentNormal()) {
        switch (_operator) {
        case BL_BOOLEAN_OPERATOR_UNION:
            if (!edge->isInside())
                _connector.addEdge(edge->_pt, edge->_opposite->_pt);
            break;
        case BL_BOOLEAN_OPERATOR_INTERSECTION:
            if (edge->isInside())
                _connector.addEdge(edge->_pt, edge->_opposite->_pt);
            break;
        case BL_BOOLEAN_OPERATOR_DIFFERENCE:
            if ((edge->isSubject() && !edge->isInside()) || (edge->isClipping() && edge->isInside()))
                _connector.addEdge(edge->_pt, edge->_opposite->_pt);
            break;
        case BL_BOOLEAN_OPERATOR_SYMMETRIC_DIFFERENCE:
            _connector.addEdge(edge->_pt, edge->_opposite->_pt);
            break;
        }
    } else if (edge->isSegmentSameTransition()) {
        if (_operator == BL_BOOLEAN_OPERATOR_UNION || _operator == BL_BOOLEAN_OPERATOR_INTERSECTION)
            _connector.addEdge(edge->_pt, edge->_opposite->_pt);
    } else if (edge->isSegmentDifferentTransition()) {
        if (_operator == BL_BOOLEAN_OPERATOR_DIFFERENCE)
            _connector.addEdge(edge->_pt, edge->_opposite->_pt);
    }
}

Segment SweepEvent::getSegment() const noexcept {
    if (isLeft()) {
        return Segment(_pt, _opposite->_pt);
    } else {
        return Segment(_opposite->_pt, _pt);
    }
}

bool SweepEvent::isPointAbove(const BLPoint& pt) const noexcept {
    if (isLeft())
        return isCounterClockwise(_pt, _opposite->_pt, pt);
    else
        return isCounterClockwise(_opposite->_pt, _pt, pt);
}

bool SweepEvent::isPointBelow(const BLPoint& pt) const noexcept {
    if (isLeft())
        return isClockwise(_pt, _opposite->_pt, pt);
    else
        return isClockwise(_opposite->_pt, _pt, pt);
}

constexpr bool SegmentIntersection::isLine1ContainedWithinLine2() const {
    return blTestFlag(flags, SegmentIntersectionFlags::kIntersectionLine1Start) &&
           blTestFlag(flags, SegmentIntersectionFlags::kIntersectionLine1End);
}

constexpr bool SegmentIntersection::isLine2ContainedWithinLine1() const {
    return blTestFlag(flags, SegmentIntersectionFlags::kIntersectionLine2Start) &&
           blTestFlag(flags, SegmentIntersectionFlags::kIntersectionLine2End);
}

constexpr bool SegmentIntersection::isNoIntersection() const {
    return blTestFlag(flags, SegmentIntersectionFlags::kNoIntersection);
}

constexpr bool SegmentIntersection::isOverlapped() const {
    return blTestFlag(flags, SegmentIntersectionFlags::kOverlapped);
}

double Segment::getLength() const noexcept {
    return bl::Geometry::length(_p1, _p1);
}

PolygonConnector::~PolygonConnector() noexcept {
    reset();
}

void PolygonConnector::reset() noexcept {
    while (!_openPolygons.empty()) {
        removePolygonPath(_openPolygons.first());
    }
}

void PolygonConnector::addEdge(const BLPoint& p1, const BLPoint& p2) noexcept {
    if (p1 == p2)
        return;

    FindPathResult f1 = find(p1);
    FindPathResult f2 = find(p2);

    if (!f1._node && !f2._node) {
        // It is a completely new edge, we will create a new polygon and
        // add edge points to the polygon.
        PolygonConnectorPathNode* pathNode = _poolPaths.alloc(_allocator);
        blCallCtor(*pathNode);

        PolygonConnectorPathItemNode* startItemNode = _poolItemNodes.alloc(_allocator);
        PolygonConnectorPathItemNode* endItemNode = _poolItemNodes.alloc(_allocator);

        blCallCtor(*startItemNode, p1);
        blCallCtor(*endItemNode, p2);

        pathNode->append(startItemNode);
        pathNode->append(endItemNode);

        _openPolygons.append(pathNode);
    } else if (f1._node && f1._node == f2._node) {
        // We have closed the path. Now, just add polygon
        // to the final path. Then, remove polygon.
        PolygonConnectorPathNode* pathNode = f1._node;

        _path.moveTo(pathNode->front()->_pt);
        for (PolygonConnectorPathItemNode* itemNode = pathNode->front()->next(); itemNode; itemNode = itemNode->next()) {
            _path.lineTo(itemNode->_pt);
        }
        _path.lineTo(pathNode->front()->_pt);

        removePolygonPath(pathNode);
    } else if (!f1._node) {
        PolygonConnectorPathItemNode* newItemNode = _poolItemNodes.alloc(_allocator);
        blCallCtor(*newItemNode, p1);

        if (f2._isFront)
            f2._node->prepend(newItemNode);
        else
            f2._node->append(newItemNode);
    } else if (!f2._node) {
        PolygonConnectorPathItemNode* newItemNode = _poolItemNodes.alloc(_allocator);
        blCallCtor(*newItemNode, p2);

        if (f1._isFront)
            f1._node->prepend(newItemNode);
        else
            f1._node->append(newItemNode);
    } else {
        BL_ASSERT(f1._node && f2._node && f1._node != f2._node);

        // Connect two paths
        while (!f2._node->empty()) {
            PolygonConnectorPathItemNode* itemNode = f2._isFront ? f2._node->popFront() : f2._node->popBack();

            if (f1._isFront)
                f1._node->prepend(itemNode);
            else
                f1._node->append(itemNode);
        }

        removePolygonPath(f2._node);
    }
}

void PolygonConnector::removePolygonPath(PolygonConnectorPathNode* node) noexcept {
    _openPolygons.unlink(node);

    while (!node->empty()) {
        PolygonConnectorPathItemNode* itemNode = node->popBack();
        blCallDtor(*itemNode);
        _poolItemNodes.free(itemNode);
    }

    blCallDtor(*node);
    _poolPaths.free(node);
}

PolygonConnector::FindPathResult PolygonConnector::find(const BLPoint& point) const noexcept {
    for (PolygonConnectorPathNode* pathNode = _openPolygons.first(); pathNode; pathNode = pathNode->next()) {
        BL_ASSERT(!pathNode->empty());

        if (pathNode->front()->_pt == point)
            return FindPathResult{pathNode, true};

        if (pathNode->back()->_pt == point)
            return FindPathResult{pathNode, false};
    }

    return FindPathResult{};
}

} // {bl}
