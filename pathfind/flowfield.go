package pathfind

// FlowField computes a flow field (vector field) from every cell toward
// a single goal. Supports 4, 6 (hex), and 8 directional movement.
// Best for group movement: 1000 units = cost of 1 pathfind.
type FlowField struct {
	grid   *Grid
	Dirs   int // 4, 6, or 8
	cost   []int32
	flow   []int32 // direction index: 0..dirs-1, or flowNone
	open   *minHeap
	closed *visitMark
}

const flowNone int32 = 255

// NewFlowField creates flow field for the given grid (default 8-dir).
func NewFlowField(grid *Grid) *FlowField {
	size := grid.Width * grid.Height
	return &FlowField{
		grid:   grid,
		Dirs:   8,
		cost:   make([]int32, size),
		flow:   make([]int32, size),
		open:   newMinHeap(size),
		closed: newVisitMark(size),
	}
}

// Build computes flow field toward goal using configured Dirs.
func (ff *FlowField) Build(goal Point) {
	ff.closed.nextGen()
	ff.open.clear()

	goalIdx := ff.grid.Index(goal.X, goal.Y)

	for i := range ff.cost {
		ff.cost[i] = inf32
		ff.flow[i] = flowNone
	}

	if !ff.grid.Walkable(goal.X, goal.Y) {
		return
	}

	ff.cost[goalIdx] = 0
	ff.open.pushOrUpdate(goalIdx, 0)

	for !ff.open.empty() {
		current, _ := ff.open.pop()
		if ff.closed.visited(current) {
			continue
		}
		ff.closed.visit(current)

		cx, cy := ff.grid.PointFromIndex(current)
		gCur := ff.cost[current]

		neighbors := getNeighbors(cx, cy, ff.Dirs)
		for di, d := range neighbors {
			nx, ny := cx+d.X, cy+d.Y
			if !ff.grid.Walkable(nx, ny) {
				continue
			}
			neighbor := ff.grid.Index(nx, ny)
			if ff.closed.visited(neighbor) {
				continue
			}
			newCost := gCur + 1
			if newCost < ff.cost[neighbor] {
				ff.cost[neighbor] = newCost
				ff.flow[neighbor] = int32(di)
				ff.open.pushOrUpdate(neighbor, newCost)
			}
		}
	}
}

// Direction returns the direction index (0..Dirs-1) to move from pos toward goal.
// For hex (Dirs=6), use HexNeighbors6(pos.Y)[dir] to get the actual Point offset.
// Returns -1 if no path or at goal.
func (ff *FlowField) Direction(pos Point) int {
	idx := ff.grid.Index(pos.X, pos.Y)
	if ff.cost[idx] == 0 {
		return -1
	}
	dir := ff.flow[idx]
	if dir == flowNone {
		return -1
	}
	return int(dir)
}

// Move returns the actual neighbor Point to move toward goal.
// Returns Point{-1,-1} if no move available.
func (ff *FlowField) Move(pos Point) Point {
	d := ff.Direction(pos)
	if d < 0 {
		return Point{-1, -1}
	}
	neighbors := getNeighbors(pos.X, pos.Y, ff.Dirs)
	if d >= len(neighbors) {
		return Point{-1, -1}
	}
	off := neighbors[d]
	return Point{pos.X + off.X, pos.Y + off.Y}
}

// Cost returns distance from pos to goal. Returns -1 if unreachable.
func (ff *FlowField) Cost(pos Point) int32 {
	idx := ff.grid.Index(pos.X, pos.Y)
	c := ff.cost[idx]
	if c == inf32 {
		return -1
	}
	return c
}

// Reachable checks if a position can reach the goal.
func (ff *FlowField) Reachable(pos Point) bool {
	return ff.Direction(pos) >= 0
}
