package pathfind

// BiAStar implements bidirectional A* with generation-based gScore.
// Supports 4, 6 (hex), and 8 directional movement.
type BiAStar struct {
	grid      *Grid
	size      int32
	// Forward search.
	fClosed   *visitMark
	fOpen     *minHeap
	fGScore   []int32
	fGGen     []int32
	fCurGen   int32
	fCameFrom []int32
	// Backward search.
	bClosed   *visitMark
	bOpen     *minHeap
	bGScore   []int32
	bGGen     []int32
	bCurGen   int32
	bCameFrom []int32
}

// NewBiAStar creates bidirectional A* instance for the grid.
func NewBiAStar(grid *Grid) *BiAStar {
	size := grid.Width * grid.Height
	return &BiAStar{
		grid:      grid,
		size:      size,
		fClosed:   newVisitMark(size),
		fOpen:     newMinHeap(size),
		fGScore:   make([]int32, size),
		fGGen:     make([]int32, size),
		fCameFrom: make([]int32, size),
		bClosed:   newVisitMark(size),
		bOpen:     newMinHeap(size),
		bGScore:   make([]int32, size),
		bGGen:     make([]int32, size),
		bCameFrom: make([]int32, size),
	}
}

// FindPath returns shortest path using 4-directional bidirectional A*.
func (b *BiAStar) FindPath(start, goal Point) []Point {
	return b.findPath(start, goal, 4)
}

// FindPath6 returns path using 6-directional hex bidirectional A*.
func (b *BiAStar) FindPath6(start, goal Point) []Point {
	return b.findPath(start, goal, 6)
}

// FindPath8 returns path using 8-directional bidirectional A*.
func (b *BiAStar) FindPath8(start, goal Point) []Point {
	return b.findPath(start, goal, 8)
}

func (b *BiAStar) findPath(start, goal Point, dirs int) []Point {
	if !b.grid.Walkable(start.X, start.Y) || !b.grid.Walkable(goal.X, goal.Y) {
		return nil
	}
	if start == goal {
		return []Point{start}
	}

	startIdx := b.grid.Index(start.X, start.Y)
	goalIdx := b.grid.Index(goal.X, goal.Y)

	b.fClosed.nextGen()
	b.bClosed.nextGen()
	b.fCurGen++
	b.bCurGen++
	b.fOpen.clear()
	b.bOpen.clear()

	b.fSetG(startIdx, 0)
	b.bSetG(goalIdx, 0)
	b.fCameFrom[startIdx] = invalid32
	b.bCameFrom[goalIdx] = invalid32

	fStart := heuristic(start, goal, dirs)
	b.fOpen.pushOrUpdate(startIdx, fStart)
	b.bOpen.pushOrUpdate(goalIdx, fStart)

	bestPathCost := inf32
	var meetingIdx int32 = invalid32

	for !b.fOpen.empty() && !b.bOpen.empty() {
		_, fTopF := b.fOpen.nodes[0].idx, b.fOpen.nodes[0].fScore
		_, bTopF := b.bOpen.nodes[0].idx, b.bOpen.nodes[0].fScore

		if fTopF >= bestPathCost && bTopF >= bestPathCost {
			break
		}

		if fTopF <= bTopF {
			current, _ := b.fOpen.pop()
			b.fClosed.visit(current)
			if b.bClosed.visited(current) {
				cost := b.fGetG(current) + b.bGetG(current)
				if cost < bestPathCost {
					bestPathCost = cost
					meetingIdx = current
				}
			}
			b.expand(current, dirs, goal, true)
		} else {
			current, _ := b.bOpen.pop()
			b.bClosed.visit(current)
			if b.fClosed.visited(current) {
				cost := b.fGetG(current) + b.bGetG(current)
				if cost < bestPathCost {
					bestPathCost = cost
					meetingIdx = current
				}
			}
			b.expand(current, dirs, start, false)
		}
	}

	if meetingIdx == invalid32 {
		return nil
	}
	return b.reconstructBiPath(meetingIdx, startIdx, goalIdx)
}

func (b *BiAStar) expand(current int32, dirs int, target Point, forward bool) {
	cx, cy := b.grid.PointFromIndex(current)
	var gCur int32
	if forward {
		gCur = b.fGetG(current)
	} else {
		gCur = b.bGetG(current)
	}

	neighbors := getNeighbors(cx, cy, dirs)
	for _, d := range neighbors {
		nx, ny := cx+d.X, cy+d.Y
		if !b.grid.Walkable(nx, ny) {
			continue
		}
		neighborIdx := b.grid.Index(nx, ny)
		if forward && b.fClosed.visited(neighborIdx) {
			continue
		}
		if !forward && b.bClosed.visited(neighborIdx) {
			continue
		}
		tentativeG := gCur + 1
		if forward {
			if tentativeG < b.fGetG(neighborIdx) {
				b.fSetG(neighborIdx, tentativeG)
				b.fCameFrom[neighborIdx] = current
				f := tentativeG + heuristic(Point{nx, ny}, target, dirs)
				b.fOpen.pushOrUpdate(neighborIdx, f)
			}
		} else {
			if tentativeG < b.bGetG(neighborIdx) {
				b.bSetG(neighborIdx, tentativeG)
				b.bCameFrom[neighborIdx] = current
				f := tentativeG + heuristic(Point{nx, ny}, target, dirs)
				b.bOpen.pushOrUpdate(neighborIdx, f)
			}
		}
	}
}

func getNeighbors(x, y int32, dirs int) []Point {
	switch dirs {
	case 6:
		offsets := HexNeighbors6(y)
		return (*offsets)[:]
	case 8:
		return Dir8[:]
	default:
		return Dir4[:]
	}
}

func (b *BiAStar) fGetG(idx int32) int32 {
	if b.fGGen[idx] < b.fCurGen {
		return inf32
	}
	return b.fGScore[idx]
}
func (b *BiAStar) fSetG(idx, val int32) {
	b.fGScore[idx] = val
	b.fGGen[idx] = b.fCurGen
}
func (b *BiAStar) bGetG(idx int32) int32 {
	if b.bGGen[idx] < b.bCurGen {
		return inf32
	}
	return b.bGScore[idx]
}
func (b *BiAStar) bSetG(idx, val int32) {
	b.bGScore[idx] = val
	b.bGGen[idx] = b.bCurGen
}

func (b *BiAStar) reconstructBiPath(meeting, start, goal int32) []Point {
	var forward []Point
	for c := meeting; c != invalid32; c = b.fCameFrom[c] {
		x, y := b.grid.PointFromIndex(c)
		forward = append(forward, Point{x, y})
	}
	for i, j := 0, len(forward)-1; i < j; i, j = i+1, j-1 {
		forward[i], forward[j] = forward[j], forward[i]
	}

	var backward []Point
	for c := b.bCameFrom[meeting]; c != invalid32; c = b.bCameFrom[c] {
		x, y := b.grid.PointFromIndex(c)
		backward = append(backward, Point{x, y})
	}

	path := make([]Point, 0, len(forward)+len(backward))
	path = append(path, forward...)
	path = append(path, backward...)
	return path
}
