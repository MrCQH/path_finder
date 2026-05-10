package pathfind

// AStar implements standard A* on a grid.
// Supports 4-directional, 6-directional (hex), and 8-directional movement.
type AStar struct {
	grid     *Grid
	closed   *visitMark
	open     *minHeap
	gScore   []int32
	gGen     []int32
	gCurGen  int32
	cameFrom []int32
	size     int32
}

// NewAStar creates an A* instance pre-allocated for the given grid.
func NewAStar(grid *Grid) *AStar {
	size := grid.Width * grid.Height
	return &AStar{
		grid:     grid,
		closed:   newVisitMark(size),
		open:     newMinHeap(size),
		gScore:   make([]int32, size),
		gGen:     make([]int32, size),
		cameFrom: make([]int32, size),
		size:     size,
	}
}

// FindPath returns shortest path using 4-directional movement.
func (a *AStar) FindPath(start, goal Point) []Point {
	return a.findPath(start, goal, 4)
}

// FindPath6 returns path using 6-directional hex movement.
func (a *AStar) FindPath6(start, goal Point) []Point {
	return a.findPath(start, goal, 6)
}

// FindPath8 returns path using 8-directional movement.
func (a *AStar) FindPath8(start, goal Point) []Point {
	return a.findPath(start, goal, 8)
}

func (a *AStar) findPath(start, goal Point, dirs int) []Point {
	if !a.grid.Walkable(start.X, start.Y) || !a.grid.Walkable(goal.X, goal.Y) {
		return nil
	}
	if start == goal {
		return []Point{start}
	}

	a.closed.nextGen()
	a.gCurGen++
	a.open.clear()

	startIdx := a.grid.Index(start.X, start.Y)
	goalIdx := a.grid.Index(goal.X, goal.Y)

	a.setG(startIdx, 0)
	a.cameFrom[startIdx] = invalid32

	fStart := heuristic(start, goal, dirs)
	a.open.pushOrUpdate(startIdx, fStart)

	for !a.open.empty() {
		current, _ := a.open.pop()
		if current == goalIdx {
			return a.reconstructPath(current, startIdx)
		}

		a.closed.visit(current)
		cx, cy := a.grid.PointFromIndex(current)
		gCur := a.getG(current)

		// Iterate neighbors based on direction count.
		switch dirs {
		case 6:
			offsets := HexNeighbors6(cy)
			for _, d := range *offsets {
				nx, ny := cx+d.X, cy+d.Y
				if !a.grid.Walkable(nx, ny) {
					continue
				}
				a.tryNeighbor(nx, ny, gCur, goal, dirs, current)
			}
		case 8:
			for _, d := range Dir8 {
				nx, ny := cx+d.X, cy+d.Y
				if !a.grid.Walkable(nx, ny) {
					continue
				}
				a.tryNeighbor(nx, ny, gCur, goal, dirs, current)
			}
		default:
			for _, d := range Dir4 {
				nx, ny := cx+d.X, cy+d.Y
				if !a.grid.Walkable(nx, ny) {
					continue
				}
				a.tryNeighbor(nx, ny, gCur, goal, dirs, current)
			}
		}
	}
	return nil
}

func (a *AStar) tryNeighbor(nx, ny int32, gCur int32, goal Point, dirs int, current int32) {
	neighbor := a.grid.Index(nx, ny)
	if a.closed.visited(neighbor) {
		return
	}
	tentativeG := gCur + 1
	if tentativeG < a.getG(neighbor) {
		a.setG(neighbor, tentativeG)
		a.cameFrom[neighbor] = current
		f := tentativeG + heuristic(Point{nx, ny}, goal, dirs)
		a.open.pushOrUpdate(neighbor, f)
	}
}

func (a *AStar) getG(idx int32) int32 {
	if a.gGen[idx] < a.gCurGen {
		return inf32
	}
	return a.gScore[idx]
}

func (a *AStar) setG(idx, val int32) {
	a.gScore[idx] = val
	a.gGen[idx] = a.gCurGen
}

func (a *AStar) reconstructPath(current, start int32) []Point {
	n := int32(0)
	for c := current; c != invalid32 && n < a.size; c = a.cameFrom[c] {
		n++
	}
	if n >= a.size {
		return nil
	}
	path := make([]Point, n)
	idx := n - 1
	for c := current; c != invalid32 && idx >= 0; c = a.cameFrom[c] {
		x, y := a.grid.PointFromIndex(c)
		path[idx] = Point{x, y}
		idx--
	}
	return path
}
