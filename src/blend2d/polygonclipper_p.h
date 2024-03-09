// This file is part of Blend2D project <https://blend2d.com>
//
// See blend2d.h or LICENSE.md for license and copyright information
// SPDX-License-Identifier: Zlib

#ifndef BLEND2D_POLYGONCLIPPER_P_H_INCLUDED
#define BLEND2D_POLYGONCLIPPER_P_H_INCLUDED

#include "api.h"
#include "api-internal_p.h"
#include "support/arenaallocator_p.h"
#include "support/arenapriorityqueue_p.h"
#include "support/arenatree_p.h"
#include "polygonclipper.h"
#include "geometry.h"

//! \cond INTERNAL
//! \addtogroup blend2d_internal
//! \{

namespace bl {

//! Status flags for sweep events in a polygon clipper.
enum class SweepEventFlags : uint32_t {
    kNoFlags                        = 0u,

    /// Indicates whether the sweep event is the start of a line segment.
    /// If set, the event marks the start; if not, it is the end of the segment.
    kIsLeft                         = 0x00000001u,

    /// Specifies the polygon to which the sweep event belongs. If set,
    /// the event is part of the "subject" polygon; otherwise, it's part of
    /// the "clipping" polygon.
    kIsSubject                      = 0x00000002u,

    /// Determines the relative vertical position of the polygon containing
    /// the edge. If set, the polygon is "below" the edge; otherwise, it is
    /// "above" the edge.
    kIsInOut                        = 0x00000004u,

    /// Indicates that the current edge is inside the other polygon.
    /// For an edge belonging to the "subject" polygon, if set, it signifies
    /// containment within the "clipping" polygon.
    kIsInside                       = 0x00000008u,

    kSegmentNonContributing         = 0x00000010u,
    kSegmentSameTransition          = 0x00000020u,
    kSegmentDifferentTransition     = 0x00000040u,
};

BL_DEFINE_ENUM_FLAGS(SweepEventFlags)

struct Segment {
    inline constexpr Segment() noexcept = default;
    inline constexpr Segment(BLPoint p1, BLPoint p2) noexcept : _p1(p1), _p2(p2) { }

    BLPoint _p1;
    BLPoint _p2;
};

struct SweepEvent {
    BLPoint _pt;
    SweepEvent* _opposite{};
    SweepEventFlags _flags{};

    BL_INLINE BL_CONSTEXPR bool isLeft() const noexcept { return blTestFlag(_flags, SweepEventFlags::kIsLeft); }
    BL_INLINE BL_CONSTEXPR bool isRight() const noexcept { return !isRight(); }
    BL_INLINE BL_CONSTEXPR bool isSubject() const noexcept { return blTestFlag(_flags, SweepEventFlags::kIsSubject); }
    BL_INLINE BL_CONSTEXPR bool isClipping() const noexcept { return !isSubject(); }
    BL_INLINE BL_CONSTEXPR bool isInOut() const noexcept { return blTestFlag(_flags, SweepEventFlags::kIsInOut); }
    BL_INLINE BL_CONSTEXPR bool isInside() const noexcept { return blTestFlag(_flags, SweepEventFlags::kIsInside); }

    BL_INLINE BL_CONSTEXPR bool isSegmentNonContributing() const noexcept { return blTestFlag(_flags, SweepEventFlags::kSegmentNonContributing); }
    BL_INLINE BL_CONSTEXPR bool isSegmentSameTransition() const noexcept { return blTestFlag(_flags, SweepEventFlags::kSegmentSameTransition); }
    BL_INLINE BL_CONSTEXPR bool isSegmentDifferentTransition() const noexcept { return blTestFlag(_flags, SweepEventFlags::kSegmentDifferentTransition); }
    BL_INLINE BL_CONSTEXPR bool isSegmentNormal() const noexcept { return !blTestFlag(_flags, SweepEventFlags::kSegmentNonContributing | SweepEventFlags::kSegmentSameTransition | SweepEventFlags::kSegmentDifferentTransition); }

    Segment getSegment() const noexcept;

    /// Determines whether a point (pt) is located above a line segment. The function
    /// yields consistent results regardless of whether it is invoked from the left or
    /// the right endpoint of the segment. If the point is above the segment (considering
    /// the y-axis), it returns true; otherwise, it returns false.
    /// \param pt The point to be tested.
    /// \return A boolean value indicating whether the point is above the segment.
    bool isPointAbove(const BLPoint& pt) const noexcept;

    /// Determines whether a point (pt) is located below a line segment. The function
    /// yields consistent results regardless of whether it is invoked from the left or
    /// the right endpoint of the segment. If the point is below the segment (considering
    /// the y-axis), it returns true; otherwise, it returns false.
    /// \param pt The point to be tested.
    /// \return A boolean value indicating whether the point is below the segment.
    bool isPointBelow(const BLPoint& pt) const noexcept;
};

class SweepEventNode : public ArenaTreeNode<SweepEventNode> {
public:
    BL_INLINE SweepEventNode(SweepEvent* event) : _event(event) { }

    SweepEvent* _event;
};

class PolygonClipperImpl {
public:
    PolygonClipperImpl() noexcept;

    BLBooleanOperator getOperator() const noexcept;
    void setOperator(BLBooleanOperator newOperator) noexcept;

    void addPolygonSegment(const Segment& segment, bool isSubject) noexcept;

    /// Performs polygon clipping
    BLResult perform() noexcept;

private:
    static constexpr size_t MEMORY_BLOCK_SIZE = 4096;

    bool isSelfOverlapping(SweepEventNode* sNode1, SweepEventNode* sNode2) const noexcept;
    BLResult updateFlags(SweepEventNode* sPrev, SweepEventNode* sNode) noexcept;

    BLBooleanOperator _operator = BL_BOOLEAN_OPERATOR_UNION;

    size_t calculateMaximumNumberOfSweepEvents() const noexcept;

    SweepEvent* allocSweepEvent() noexcept;
    void freeSweepEvent(SweepEvent* event) noexcept;

    SweepEventNode* allocSweepEventNode(SweepEvent* event) noexcept;
    void freeSweepEventNode(SweepEventNode* node) noexcept;

    ArenaAllocatorTmp<MEMORY_BLOCK_SIZE> _memoryAllocator;
    ArenaPool<SweepEvent> _sweepEventPool;
    ArenaPool<SweepEventNode> _sweepEventNodePool;

    double _epsilon = 0.001;

    size_t _subjectEdgeCount = 0;
    size_t _clippingEdgeCount = 0;

    ArenaPriorityQueue<SweepEventNode> _q;
    ArenaTree<SweepEventNode> _s;
};

} // {bl}

//! \}
//! \endcond

#endif // BLEND2D_POLYGONCLIPPER_P_H_INCLUDED
