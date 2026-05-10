package pathfind

import "sync"

// PathCache caches paths between chunks for hot routes.
// SLG maps have heavy hotspot patterns (main city ↔ resource points).
// Caching (startChunk, endChunk) paths eliminates redundant computation.
type PathCache struct {
	mu       sync.RWMutex
	entries  map[cacheKey]cacheEntry
	maxSize  int
}

type cacheKey struct {
	sx, sy int32 // start chunk coordinates
	gx, gy int32 // goal chunk coordinates
}

type cacheEntry struct {
	path []Point
	cost int32
}

// NewPathCache creates a path cache with given max entries.
// Typical size: 10000 for SLG.
func NewPathCache(maxSize int) *PathCache {
	return &PathCache{
		entries: make(map[cacheKey]cacheEntry, maxSize),
		maxSize: maxSize,
	}
}

// Get returns cached path if present.
func (pc *PathCache) Get(sx, sy, gx, gy int32) ([]Point, bool) {
	pc.mu.RLock()
	defer pc.mu.RUnlock()
	entry, ok := pc.entries[cacheKey{sx, sy, gx, gy}]
	if !ok {
		return nil, false
	}
	// Return copy to prevent mutation.
	cp := make([]Point, len(entry.path))
	copy(cp, entry.path)
	return cp, true
}

// Set stores a path in the cache.
func (pc *PathCache) Set(sx, sy, gx, gy int32, path []Point) {
	pc.mu.Lock()
	defer pc.mu.Unlock()

	key := cacheKey{sx, sy, gx, gy}
	if len(pc.entries) >= pc.maxSize {
		// Simple eviction: delete a random entry.
		for k := range pc.entries {
			delete(pc.entries, k)
			break
		}
	}

	cp := make([]Point, len(path))
	copy(cp, path)
	pc.entries[key] = cacheEntry{
		path: cp,
		cost: int32(len(path) - 1),
	}
}

// Invalidate clears all cached paths. Call after map changes.
func (pc *PathCache) Invalidate() {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pc.entries = make(map[cacheKey]cacheEntry, pc.maxSize)
}

// InvalidateChunk removes cached paths involving a specific chunk.
func (pc *PathCache) InvalidateChunk(cx, cy int32) {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	for k := range pc.entries {
		if (k.sx == cx && k.sy == cy) || (k.gx == cx && k.gy == cy) {
			delete(pc.entries, k)
		}
	}
}

// Size returns current cache entry count.
func (pc *PathCache) Size() int {
	pc.mu.RLock()
	defer pc.mu.RUnlock()
	return len(pc.entries)
}
