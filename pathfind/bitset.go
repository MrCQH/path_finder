package pathfind

// BitSet is a compact boolean array for closed set tracking.
// Uses []uint64 for cache-friendly access without map overhead.
type BitSet struct {
	bits []uint64
	size int32
}

// NewBitSet creates bitset for n elements.
func NewBitSet(n int32) *BitSet {
	return &BitSet{
		bits: make([]uint64, (int(n)+63)/64),
		size: n,
	}
}

// Set marks bit at index i.
func (b *BitSet) Set(i int32) {
	b.bits[i/64] |= 1 << (i % 64)
}

// Test returns true if bit at index i is set.
func (b *BitSet) Test(i int32) bool {
	return b.bits[i/64]&(1<<(i%64)) != 0
}

// Clear resets all bits.
func (b *BitSet) Clear() {
	for i := range b.bits {
		b.bits[i] = 0
	}
}

// visitMark is a generation-based closed set — faster than clearing bits
// every search. Increment gen each search; Test checks if mark == gen.
type visitMark struct {
	marks []int32
	gen   int32
}

func newVisitMark(size int32) *visitMark {
	return &visitMark{
		marks: make([]int32, size),
		gen:   1,
	}
}

func (v *visitMark) nextGen() {
	v.gen++
}

func (v *visitMark) visit(idx int32) {
	v.marks[idx] = v.gen
}

func (v *visitMark) visited(idx int32) bool {
	return v.marks[idx] == v.gen
}
