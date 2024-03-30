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
#include "support/arenalist_p.h"
#include "polygonclipper.h"
#include "geometry.h"
#include "path.h"

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

    double getLength() const noexcept;

    BLPoint _p1;
    BLPoint _p2;
};

class SweepEventNode;

struct SweepEvent {
    BLPoint _pt;
    SweepEvent* _opposite{};
    SweepEventNode* _nodeS{};
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

enum class SegmentIntersectionFlags : uint32_t {
    kNoFlags                    = 0x0000,
    kNoIntersection             = 0x0001,
    kIntersectionLine1Start     = 0x0002,
    kIntersectionLine2Start     = 0x0004,
    kIntersectionLine1End       = 0x0008,
    kIntersectionLine2End       = 0x0010,
    kIntersectionLine1Interior  = 0x0020,
    kIntersectionLine2Interior  = 0x0040,
    kOverlapped                 = 0x0080,
};

BL_DEFINE_ENUM_FLAGS(SegmentIntersectionFlags)

struct SegmentIntersection
{
    BLPoint ptLeft1;
    BLPoint ptLeft2;

    BLPoint ptRight1;
    BLPoint ptRight2;

    SegmentIntersectionFlags flags = SegmentIntersectionFlags();

    constexpr bool isLine1ContainedWithinLine2() const;
    constexpr bool isLine2ContainedWithinLine1() const;
    constexpr bool isNoIntersection() const;
    constexpr bool isOverlapped() const;
};

class SweepEventNode : public ArenaTreeNode<SweepEventNode> {
public:
    BL_INLINE SweepEventNode(SweepEvent* event) : _event(event) { }

    SweepEvent* _event;
};

class PolygonConnectorPathItemNode : public ArenaListNode<PolygonConnectorPathItemNode> {
public:
    inline constexpr PolygonConnectorPathItemNode() = default;
    inline constexpr PolygonConnectorPathItemNode(const BLPoint& pt) : _pt(pt) { }

    BLPoint _pt;
};

class PolygonConnectorPathNode : public ArenaListNode<PolygonConnectorPathNode> {
public:

    bool empty() const noexcept { return _path.empty(); }
    PolygonConnectorPathItemNode* front() const noexcept { return _path.first(); }
    PolygonConnectorPathItemNode* back() const noexcept { return _path.last(); }

    void append(PolygonConnectorPathItemNode* node) noexcept { _path.append(node); }
    void prepend(PolygonConnectorPathItemNode* node) noexcept { _path.prepend(node); }

    PolygonConnectorPathItemNode* popFront() noexcept { return _path.popFirst(); }
    PolygonConnectorPathItemNode* popBack() noexcept { return _path.pop(); }

private:
    ArenaList<PolygonConnectorPathItemNode> _path;
};

class PolygonConnector {
public:
    inline PolygonConnector(ArenaAllocator& allocator) noexcept : _allocator(allocator) {}
    ~PolygonConnector() noexcept;

    void reset() noexcept;
    void addEdge(const BLPoint& p1, const BLPoint& p2) noexcept;
    const BLPath& getPath() const noexcept { return _path; }

private:
    struct FindPathResult {
        PolygonConnectorPathNode* _node{};
        bool _isFront{};
    };

    void removePolygonPath(PolygonConnectorPathNode* node) noexcept;

    FindPathResult find(const BLPoint& point) const noexcept;

    ArenaAllocator& _allocator;
    ArenaList<PolygonConnectorPathNode> _openPolygons;
    ArenaPool<PolygonConnectorPathNode> _poolPaths;
    ArenaPool<PolygonConnectorPathItemNode> _poolItemNodes;
    BLPath _path;
};

class PolygonClipperImpl {
public:
    PolygonClipperImpl(size_t blockSize, void* staticData, size_t staticSize) noexcept;

    BLBooleanOperator getOperator() const noexcept;
    void setOperator(BLBooleanOperator newOperator) noexcept;

    void addPolygonSegment(const Segment& segment, bool isSubject) noexcept;

    /// Performs polygon clipping
    BLResult perform() noexcept;

    void reset() noexcept;

    const BLPath& getPath() const noexcept { return _connector.getPath(); }

private:
    bool isSelfOverlapping(SweepEventNode* sNode1, SweepEventNode* sNode2) const noexcept;
    BLResult updateFlags(SweepEventNode* sPrev, SweepEventNode* sNode) noexcept;

    /// Finds intersections in the events corresponding to sweep event nodes.
    /// Nodes must always contain pointers to left sweep events.
    /// \param sPrev Previous node containing a pointer to a sweep event on the sweep line.
    /// \param sNode Next node containing a pointer to a sweep event on the sweep line.
    /// \return BLResult indicating the result of the intersection finding process.
    BLResult findIntersections(SweepEventNode* sPrev, SweepEventNode* sNode) noexcept;

    void divideSegment(SweepEventNode* sNode, const BLPoint& point) noexcept;

    BLResult calculateSegmentIntersections(SegmentIntersection& intersections, const Segment& s1, const Segment& s2) const noexcept;
    void updateResult(BLResult& oldResult, BLResult newResult) const noexcept;
    double clampParameter(double parameter, double ordinate, double segmentLength) const noexcept;

    size_t calculateMaximumNumberOfSweepEvents() const noexcept;

    SweepEvent* allocSweepEvent() noexcept;
    void freeSweepEvent(SweepEvent* event) noexcept;

    SweepEventNode* allocSweepEventNode(SweepEvent* event) noexcept;
    void freeSweepEventNode(SweepEventNode* node) noexcept;

    /// Based on the operator (intersection, union, difference,
    /// symmetric difference), it is decided whether to add the edge to the resulting
    /// polygon. Event must represent the starting point.
    /// \param edge The edge we want to insert
    void addResultEdge(SweepEvent* edge);

    BLBooleanOperator _operator = BL_BOOLEAN_OPERATOR_UNION;

    ArenaAllocator _memoryAllocator;
    ArenaPool<SweepEvent> _sweepEventPool;
    ArenaPool<SweepEventNode> _sweepEventNodePool;

    double _epsilon = 0.001;
    double _epsilonLineParameter = 0.0001;

    size_t _subjectEdgeCount = 0;
    size_t _clippingEdgeCount = 0;

    ArenaPriorityQueue<SweepEventNode> _q;
    ArenaTree<SweepEventNode> _s;
    PolygonConnector _connector;
};

} // {bl}

//! \}
//! \endcond

#endif // BLEND2D_POLYGONCLIPPER_P_H_INCLUDED
