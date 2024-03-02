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

} // {bl}
