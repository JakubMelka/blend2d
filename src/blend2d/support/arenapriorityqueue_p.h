// This file is part of Blend2D project <https://blend2d.com>
//
// See blend2d.h or LICENSE.md for license and copyright information
// SPDX-License-Identifier: Zlib

#ifndef BLEND2D_SUPPORT_ARENAPRIORITYQUEUE_P_H_INCLUDED
#define BLEND2D_SUPPORT_ARENAPRIORITYQUEUE_P_H_INCLUDED

#include "../api-internal_p.h"
#include "../support/arenatree_p.h"

//! \cond INTERNAL
//! \addtogroup blend2d_internal
//! \{

namespace bl {

//! \name Arena Allocated Priority Queue
//! \{

//! A priority queue that uses nodes allocated by `ArenaAllocator`.
template<typename NodeT>
class ArenaPriorityQueue {
public:
    BL_NONCOPYABLE(ArenaPriorityQueue)

    BL_INLINE ArenaPriorityQueue() noexcept = default;

    BL_INLINE bool empty() const noexcept { return _tree.empty(); }

    template<typename KeyT, typename CompareT = CompareOp<SortOrder::kAscending>>
    BL_INLINE NodeT* find(const KeyT& key, const CompareT& cmp = CompareT()) const noexcept {
        return _tree.get(key, cmp);
    }

    template<typename CompareT = CompareOp<SortOrder::kAscending>>
    void remove(ArenaTreeNodeBase* node, const CompareT& cmp = CompareT()) noexcept {
        if (_min == node) {
            _min = _tree.next(_min, cmp);
        }
        _tree.remove(node, cmp);
    }

    template<typename CompareT = CompareOp<SortOrder::kAscending>>
    void insert(NodeT* node, const CompareT& cmp = CompareT()) noexcept {
        _tree.insert(node, cmp);

        if (!_min) {
            _min = node;
        } else if (NodeT* newMinNode = _tree.prev(_min, cmp)) {
            _min = newMinNode;
        }
    }

    NodeT* top() const noexcept { return _min; }

    template<typename CompareT = CompareOp<SortOrder::kAscending>>
    NodeT* pop(const CompareT& cmp = CompareT()) noexcept {
        NodeT* node = _min;
        remove(node, cmp);
        return node;
    }

private:
    ArenaTree<NodeT> _tree;
    NodeT* _min = nullptr;
};

//! \}

} // {bl}

//! \}
//! \endcond

#endif // BLEND2D_SUPPORT_ARENAPRIORITYQUEUE_P_H_INCLUDED
