package pathfind

// heapNode represents a node in the intrusive binary heap.
// Index tracks the node's position in the heap slice, enabling O(1) contains
// and O(log n) decrease-key without container/heap overhead.
type heapNode struct {
	fScore int32
	idx    int32 // flat grid index
	heapIdx int32 // position in heap; -1 if not in heap
}

// Intrusive min-heap keyed by fScore. Stores flat grid indices.
// Avoids container/heap's interface overhead and allocations.
type minHeap struct {
	nodes []heapNode
	// lookup maps grid index -> *heapNode for O(1) access.
	lookup []heapNode
}

func newMinHeap(size int32) *minHeap {
	h := &minHeap{
		nodes:  make([]heapNode, 0, 1024),
		lookup: make([]heapNode, size),
	}
	for i := range h.lookup {
		h.lookup[i].heapIdx = -1
		h.lookup[i].idx = int32(i)
	}
	return h
}

func (h *minHeap) clear() {
	// Reset heapIdx for all nodes currently in heap.
	for i := range h.nodes {
		h.lookup[h.nodes[i].idx].heapIdx = -1
	}
	h.nodes = h.nodes[:0]
}

// pushOrUpdate inserts or decreases key of node at flat index idx.
func (h *minHeap) pushOrUpdate(idx int32, fScore int32) {
	n := &h.lookup[idx]
	if n.heapIdx >= 0 {
		// Already in heap — decrease key.
		if fScore < n.fScore {
			n.fScore = fScore
			h.siftUp(int32(n.heapIdx))
		}
		return
	}
	n.fScore = fScore
	h.nodes = append(h.nodes, *n)
	ni := int32(len(h.nodes) - 1)
	h.nodes[ni].heapIdx = ni
	h.lookup[idx].heapIdx = ni
	h.siftUp(ni)
}

func (h *minHeap) pop() (int32, int32) {
	top := h.nodes[0]
	h.lookup[top.idx].heapIdx = -1
	last := len(h.nodes) - 1
	if last > 0 {
		h.nodes[0] = h.nodes[last]
		h.nodes[0].heapIdx = 0
		h.lookup[h.nodes[0].idx].heapIdx = 0
		h.siftDown(0)
	}
	h.nodes = h.nodes[:last]
	return top.idx, top.fScore
}

func (h *minHeap) empty() bool {
	return len(h.nodes) == 0
}

func (h *minHeap) siftUp(i int32) {
	nodes := h.nodes
	for i > 0 {
		p := (i - 1) / 2
		if nodes[p].fScore <= nodes[i].fScore {
			break
		}
		nodes[p], nodes[i] = nodes[i], nodes[p]
		nodes[p].heapIdx = p
		nodes[i].heapIdx = i
		h.lookup[nodes[p].idx].heapIdx = p
		h.lookup[nodes[i].idx].heapIdx = i
		i = p
	}
}

func (h *minHeap) siftDown(i int32) {
	nodes := h.nodes
	n := int32(len(nodes))
	for {
		smallest := i
		l := 2*i + 1
		r := 2*i + 2
		if l < n && nodes[l].fScore < nodes[smallest].fScore {
			smallest = l
		}
		if r < n && nodes[r].fScore < nodes[smallest].fScore {
			smallest = r
		}
		if smallest == i {
			break
		}
		nodes[i], nodes[smallest] = nodes[smallest], nodes[i]
		nodes[i].heapIdx = i
		nodes[smallest].heapIdx = smallest
		h.lookup[nodes[i].idx].heapIdx = i
		h.lookup[nodes[smallest].idx].heapIdx = smallest
		i = smallest
	}
}
