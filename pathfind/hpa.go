package pathfind

type hpaEdge struct {
	to   int32
	cost int32
	path []Point
}

type hpaEntrance struct {
	idx    int32
	x, y   int32
	chunkX int32
	chunkY int32
	edges  []hpaEdge
}

type pointEntrance struct {
	entIdx int32
	cost   int32
	path   []Point
}

// HPA implements Hierarchical Pathfinding A* with chunk-constrained search.
// Dirs controls movement: 4 (default), 6 (hex), or 8.
type HPA struct {
	grid      *Grid
	chunkSize int32
	chunksX   int32
	chunksY   int32
	Dirs      int // 4, 6, or 8

	entrances   []hpaEntrance
	entranceMap map[int32]int32

	// Chunk-constrained A* for building intra-chunk edges.
	chunkAstar  *AStar

	// For abstract A*.
	abstractClosed *visitMark
	abstractOpen   *minHeap
	abstractG      []int32
	abstractGGen   []int32
	abstractCurGen int32
	abstractParent []int32
	abstractSize   int32

	initialized bool
}

// NewHPA creates HPA* for the given grid with specified chunk size.
func NewHPA(grid *Grid, chunkSize int32) *HPA {
	chunksX := (grid.Width + chunkSize - 1) / chunkSize
	chunksY := (grid.Height + chunkSize - 1) / chunkSize

	return &HPA{
		grid:       grid,
		chunkSize:  chunkSize,
		chunksX:    chunksX,
		chunksY:    chunksY,
		Dirs:       4,
		chunkAstar: NewAStar(grid),
	}
}

// Build constructs the abstract graph.
func (h *HPA) Build() {
	h.entranceMap = make(map[int32]int32, int(h.chunksX*h.chunksY*4))
	h.findEntrances()

	h.abstractSize = int32(len(h.entrances))
	h.abstractClosed = newVisitMark(h.abstractSize)
	h.abstractOpen = newMinHeap(h.abstractSize)
	h.abstractG = make([]int32, h.abstractSize)
	h.abstractGGen = make([]int32, h.abstractSize)
	h.abstractParent = make([]int32, h.abstractSize)

	h.connectEntrances()
	h.initialized = true
}

func (h *HPA) findEntrances() {
	h.entrances = h.entrances[:0]

	for cy := int32(0); cy < h.chunksY; cy++ {
		for cx := int32(0); cx < h.chunksX; cx++ {
			minX := cx * h.chunkSize
			maxX := minX + h.chunkSize - 1
			if maxX >= h.grid.Width {
				maxX = h.grid.Width - 1
			}
			minY := cy * h.chunkSize
			maxY := minY + h.chunkSize - 1
			if maxY >= h.grid.Height {
				maxY = h.grid.Height - 1
			}

			// Right boundary: merge consecutive walkable transitions.
			if cx+1 < h.chunksX {
				h.mergeBoundaryEntrances(minX, maxX, minY, maxY, true, cx, cy, cx+1, cy)
			}
			// Bottom boundary.
			if cy+1 < h.chunksY {
				h.mergeBoundaryEntrances(minX, maxX, minY, maxY, false, cx, cy, cx, cy+1)
			}
		}
	}
}

// mergeBoundaryEntrances scans a chunk boundary and adds merged entrances.
// vertical=true scans the right boundary (x=maxX, iterate y); false scans bottom boundary (y=maxY, iterate x).
func (h *HPA) mergeBoundaryEntrances(minX, maxX, minY, maxY int32, vertical bool,
	cxA, cyA, cxB, cyB int32) {

	var start, end int32
	if vertical {
		start, end = minY, maxY
	} else {
		start, end = minX, maxX
	}

	inRun := false
	runStart := int32(0)

	for i := start; i <= end; i++ {
		var walkA, walkB bool
		if vertical {
			walkA = h.grid.Walkable(maxX, i)
			walkB = h.grid.Walkable(maxX+1, i)
		} else {
			walkA = h.grid.Walkable(i, maxY)
			walkB = h.grid.Walkable(i, maxY+1)
		}

		if walkA && walkB {
			if !inRun {
				inRun = true
				runStart = i
			}
		} else {
			if inRun {
				// End of run. Add merged entrance at midpoint.
				mid := runStart + (i-1-runStart)/2
				if vertical {
					h.addEntrance(maxX, mid, cxA, cyA)
					h.addEntrance(maxX+1, mid, cxB, cyB)
				} else {
					h.addEntrance(mid, maxY, cxA, cyA)
					h.addEntrance(mid, maxY+1, cxB, cyB)
				}
				inRun = false
			}
		}
	}
	if inRun {
		mid := runStart + (end-runStart)/2
		if vertical {
			h.addEntrance(maxX, mid, cxA, cyA)
			h.addEntrance(maxX+1, mid, cxB, cyB)
		} else {
			h.addEntrance(mid, maxY, cxA, cyA)
			h.addEntrance(mid, maxY+1, cxB, cyB)
		}
	}
}

func (h *HPA) addEntrance(x, y, chunkX, chunkY int32) {
	idx := h.grid.Index(x, y)
	if _, exists := h.entranceMap[idx]; exists {
		return
	}
	ei := int32(len(h.entrances))
	h.entranceMap[idx] = ei
	h.entrances = append(h.entrances, hpaEntrance{
		idx:    idx,
		x:      x,
		y:      y,
		chunkX: chunkX,
		chunkY: chunkY,
	})
}

// connectEntrances builds intra-chunk edges and inter-chunk adjacency edges.
func (h *HPA) connectEntrances() {
	chunkEntrances := make(map[[2]int32][]int32)
	for i := range h.entrances {
		key := [2]int32{h.entrances[i].chunkX, h.entrances[i].chunkY}
		chunkEntrances[key] = append(chunkEntrances[key], int32(i))
	}

	// Intra-chunk edges: connect entrances within same chunk via constrained A*.
	for key, ents := range chunkEntrances {
		cx, cy := key[0], key[1]
		bb := h.chunkBBox(cx, cy)

		for i, ei := range ents {
			for j := i + 1; j < len(ents); j++ {
				ej := ents[j]
				a := h.entrances[ei]
				b := h.entrances[ej]

				path := h.constrainedFind(Point{a.x, a.y}, Point{b.x, b.y}, bb)
				if path == nil {
					continue
				}
				cost := int32(len(path) - 1)
				h.entrances[ei].edges = append(h.entrances[ei].edges, hpaEdge{to: ej, cost: cost, path: path})
				revPath := make([]Point, len(path))
				for k := range path {
					revPath[len(path)-1-k] = path[k]
				}
				h.entrances[ej].edges = append(h.entrances[ej].edges, hpaEdge{to: ei, cost: cost, path: revPath})
			}
		}
	}

	// Inter-chunk edges: connect physically adjacent entrances on chunk boundaries.
	// These are entrances at (x,y) in chunk A and (x+1,y) or (x,y+1) in chunk B.
	for i := range h.entrances {
		a := &h.entrances[i]
		// Check adjacent cell in each 4-direction.
		for _, d := range Dir4 {
			nx, ny := a.x+d.X, a.y+d.Y
			if !h.grid.InBounds(nx, ny) {
				continue
			}
			neighborIdx := h.grid.Index(nx, ny)
			ej, exists := h.entranceMap[neighborIdx]
			if !exists {
				continue
			}
			b := &h.entrances[ej]
			// Only connect if they're in different chunks (adjacent boundary).
			if a.chunkX == b.chunkX && a.chunkY == b.chunkY {
				continue
			}
			// Check if edge already exists.
			dup := false
			for _, e := range a.edges {
				if e.to == ej {
					dup = true
					break
				}
			}
			if !dup {
				// Cost = 1 (physically adjacent).
				path := []Point{{a.x, a.y}, {nx, ny}}
				a.edges = append(a.edges, hpaEdge{to: ej, cost: 1, path: path})
			}
		}
	}
}

// chunkBBox returns the bounding box for chunk (cx, cy) with small margin.
func (h *HPA) chunkBBox(cx, cy int32) bbox {
	minX := cx * h.chunkSize
	minY := cy * h.chunkSize
	maxX := minX + h.chunkSize - 1
	maxY := minY + h.chunkSize - 1
	if maxX >= h.grid.Width {
		maxX = h.grid.Width - 1
	}
	if maxY >= h.grid.Height {
		maxY = h.grid.Height - 1
	}
	return bbox{minX, minY, maxX, maxY}
}

type bbox struct {
	minX, minY, maxX, maxY int32
}

// constrainedFind runs A* within a bounding box using HPA's configured directions.
func (h *HPA) constrainedFind(start, goal Point, bb bbox) []Point {
	if !h.grid.Walkable(start.X, start.Y) || !h.grid.Walkable(goal.X, goal.Y) {
		return nil
	}
	if start == goal {
		return []Point{start}
	}

	a := h.chunkAstar
	a.closed.nextGen()
	a.gCurGen++
	a.open.clear()

	startIdx := h.grid.Index(start.X, start.Y)
	goalIdx := h.grid.Index(goal.X, goal.Y)

	a.setG(startIdx, 0)
	a.cameFrom[startIdx] = invalid32
	fStart := heuristic(start, goal, h.Dirs)
	a.open.pushOrUpdate(startIdx, fStart)

	for !a.open.empty() {
		current, _ := a.open.pop()
		if current == goalIdx {
			return a.reconstructPath(current, startIdx)
		}

		a.closed.visit(current)
		cx, cy := h.grid.PointFromIndex(current)
		gCur := a.getG(current)

		neighbors := getNeighbors(cx, cy, h.Dirs)
		for _, d := range neighbors {
			nx, ny := cx+d.X, cy+d.Y
			if nx < bb.minX || nx > bb.maxX || ny < bb.minY || ny > bb.maxY {
				continue
			}
			if !h.grid.Walkable(nx, ny) {
				continue
			}
			neighbor := h.grid.Index(nx, ny)
			if a.closed.visited(neighbor) {
				continue
			}
			tentativeG := gCur + 1
			if tentativeG < a.getG(neighbor) {
				a.setG(neighbor, tentativeG)
				a.cameFrom[neighbor] = current
				f := tentativeG + heuristic(Point{nx, ny}, goal, h.Dirs)
				a.open.pushOrUpdate(neighbor, f)
			}
		}
	}
	return nil
}

// FindPath finds path from start to goal using HPA* with configured directions.
func (h *HPA) FindPath(start, goal Point) []Point {
	return h.findPath(start, goal)
}

// FindPath6 finds path using 6-directional hex HPA*.
func (h *HPA) FindPath6(start, goal Point) []Point {
	prev := h.Dirs
	h.Dirs = 6
	defer func() { h.Dirs = prev }()
	return h.findPath(start, goal)
}

func (h *HPA) findPath(start, goal Point) []Point {
	if !h.initialized {
		h.Build()
	}
	if !h.grid.Walkable(start.X, start.Y) || !h.grid.Walkable(goal.X, goal.Y) {
		return nil
	}
	if start == goal {
		return []Point{start}
	}

	scx, scy := start.X/h.chunkSize, start.Y/h.chunkSize
	gcx, gcy := goal.X/h.chunkSize, goal.Y/h.chunkSize
	if scx == gcx && scy == gcy {
		bb := h.chunkBBox(scx, scy)
		return h.constrainedFind(start, goal, bb)
	}

	startEnts := h.connectPoint(start, scx, scy)
	goalEnts := h.connectPoint(goal, gcx, gcy)

	if len(startEnts) == 0 || len(goalEnts) == 0 {
		return nil
	}

	abstractPath := h.abstractSearch(startEnts, goalEnts, start, goal)
	if abstractPath == nil {
		return nil
	}
	return h.expandPath(abstractPath, start, goal)
}

func (h *HPA) connectPoint(p Point, cx, cy int32) []pointEntrance {
	var result []pointEntrance
	bb := h.chunkBBox(cx, cy)
	for i := range h.entrances {
		e := &h.entrances[i]
		if e.chunkX != cx || e.chunkY != cy {
			continue
		}
		path := h.constrainedFind(p, Point{e.x, e.y}, bb)
		if path != nil {
			result = append(result, pointEntrance{
				entIdx: int32(i),
				cost:   int32(len(path) - 1),
				path:   path,
			})
		}
	}
	return result
}

func (h *HPA) abstractSearch(startEnts, goalEnts []pointEntrance, start, goal Point) []int32 {
	h.abstractClosed.nextGen()
	h.abstractCurGen++
	h.abstractOpen.clear()

	for _, se := range startEnts {
		h.abstractSetG(se.entIdx, se.cost)
		h.abstractParent[se.entIdx] = invalid32
		f := se.cost + heuristic(start, goal, h.Dirs)
		h.abstractOpen.pushOrUpdate(se.entIdx, f)
	}

	goalSet := make(map[int32]bool)
	for _, ge := range goalEnts {
		goalSet[ge.entIdx] = true
	}

	for !h.abstractOpen.empty() {
		current, _ := h.abstractOpen.pop()
		if goalSet[current] {
			var path []int32
			for c := current; c != invalid32; c = h.abstractParent[c] {
				path = append(path, c)
			}
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path
		}

		if h.abstractClosed.visited(current) {
			continue
		}
		h.abstractClosed.visit(current)

		gCur := h.abstractGetG(current)
		for _, edge := range h.entrances[current].edges {
			neighbor := edge.to
			if h.abstractClosed.visited(neighbor) {
				continue
			}
			tentativeG := gCur + edge.cost
			if tentativeG < h.abstractGetG(neighbor) {
				h.abstractSetG(neighbor, tentativeG)
				h.abstractParent[neighbor] = current
				e := &h.entrances[neighbor]
				f := tentativeG + heuristic(Point{e.x, e.y}, goal, h.Dirs)
				h.abstractOpen.pushOrUpdate(neighbor, f)
			}
		}
	}
	return nil
}

func (h *HPA) abstractGetG(idx int32) int32 {
	if h.abstractGGen[idx] < h.abstractCurGen {
		return inf32
	}
	return h.abstractG[idx]
}

func (h *HPA) abstractSetG(idx, val int32) {
	h.abstractG[idx] = val
	h.abstractGGen[idx] = h.abstractCurGen
}

func (h *HPA) expandPath(abstractPath []int32, start, goal Point) []Point {
	if len(abstractPath) == 0 {
		return nil
	}

	var fullPath []Point

	firstEnt := h.entrances[abstractPath[0]]
	firstBB := h.chunkBBox(firstEnt.chunkX, firstEnt.chunkY)
	startPath := h.constrainedFind(start, Point{firstEnt.x, firstEnt.y}, firstBB)
	if startPath != nil {
		fullPath = append(fullPath, startPath...)
		if len(fullPath) > 0 {
			fullPath = fullPath[:len(fullPath)-1]
		}
	}

	for i := 1; i < len(abstractPath); i++ {
		from := h.entrances[abstractPath[i-1]]
		var edgePath []Point
		for _, edge := range from.edges {
			if edge.to == abstractPath[i] {
				edgePath = edge.path
				break
			}
		}
		if edgePath != nil {
			if len(fullPath) > 0 {
				fullPath = append(fullPath, edgePath[1:]...)
			} else {
				fullPath = append(fullPath, edgePath...)
			}
		}
	}

	lastEnt := h.entrances[abstractPath[len(abstractPath)-1]]
	lastBB := h.chunkBBox(lastEnt.chunkX, lastEnt.chunkY)
	goalPath := h.constrainedFind(Point{lastEnt.x, lastEnt.y}, goal, lastBB)
	if goalPath != nil && len(goalPath) > 1 {
		fullPath = append(fullPath, goalPath[1:]...)
	}

	return fullPath
}

// RebuildChunk rebuilds entrances and edges for a single chunk.
func (h *HPA) RebuildChunk(cx, cy int32) {
	// Remove old entrances in this chunk.
	newEnts := h.entrances[:0]
	newMap := make(map[int32]int32)
	for i, e := range h.entrances {
		if e.chunkX == cx && e.chunkY == cy {
			continue
		}
		newMap[e.idx] = int32(len(newEnts))
		newEnts = append(newEnts, h.entrances[i])
	}
	h.entrances = newEnts
	h.entranceMap = newMap

	// Clear stale edges.
	for i := range h.entrances {
		filtered := h.entrances[i].edges[:0]
		for _, e := range h.entrances[i].edges {
			if _, ok := h.entranceMap[h.entrances[e.to].idx]; ok {
				filtered = append(filtered, e)
			}
		}
		h.entrances[i].edges = filtered
	}

	// Re-add entrances at boundaries of this chunk.
	minX := cx * h.chunkSize
	maxX := minX + h.chunkSize - 1
	if maxX >= h.grid.Width {
		maxX = h.grid.Width - 1
	}
	minY := cy * h.chunkSize
	maxY := minY + h.chunkSize - 1
	if maxY >= h.grid.Height {
		maxY = h.grid.Height - 1
	}

	if cx+1 < h.chunksX {
		for y := minY; y <= maxY; y++ {
			if h.grid.Walkable(maxX, y) && h.grid.Walkable(maxX+1, y) {
				h.addEntrance(maxX, y, cx, cy)
				h.addEntrance(maxX+1, y, cx+1, cy)
			}
		}
	}
	if cy+1 < h.chunksY {
		for x := minX; x <= maxX; x++ {
			if h.grid.Walkable(x, maxY) && h.grid.Walkable(x, maxY+1) {
				h.addEntrance(x, maxY, cx, cy)
				h.addEntrance(x, maxY+1, cx, cy+1)
			}
		}
	}

	// Reconnect affected chunks.
	affected := map[[2]int32]bool{{cx, cy}: true}
	if cx > 0 {
		affected[[2]int32{cx - 1, cy}] = true
	}
	if cy > 0 {
		affected[[2]int32{cx, cy - 1}] = true
	}

	chunkEnts := make(map[[2]int32][]int32)
	for i := range h.entrances {
		key := [2]int32{h.entrances[i].chunkX, h.entrances[i].chunkY}
		if affected[key] {
			chunkEnts[key] = append(chunkEnts[key], int32(i))
		}
	}
	for key, ents := range chunkEnts {
		bb := h.chunkBBox(key[0], key[1])
		for i, ei := range ents {
			for j := i + 1; j < len(ents); j++ {
				ej := ents[j]
				a := h.entrances[ei]
				b := h.entrances[ej]
				path := h.constrainedFind(Point{a.x, a.y}, Point{b.x, b.y}, bb)
				if path == nil {
					continue
				}
				cost := int32(len(path) - 1)
				h.entrances[ei].edges = append(h.entrances[ei].edges, hpaEdge{to: ej, cost: cost, path: path})
				rev := make([]Point, len(path))
				for k := range path {
					rev[len(path)-1-k] = path[k]
				}
				h.entrances[ej].edges = append(h.entrances[ej].edges, hpaEdge{to: ei, cost: cost, path: rev})
			}
		}
	}
}
