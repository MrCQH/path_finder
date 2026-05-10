package pathfind

// JPS implements Jump Point Search on a uniform-cost grid
// with generation-based gScore and iterative jump scanning.
type JPS struct {
	grid     *Grid
	closed   *visitMark
	open     *minHeap
	gScore   []int32
	gGen     []int32
	gCurGen  int32
	cameFrom []int32
	size     int32
}

// NewJPS creates a JPS instance for the given grid.
func NewJPS(grid *Grid) *JPS {
	size := grid.Width * grid.Height
	return &JPS{
		grid:     grid,
		closed:   newVisitMark(size),
		open:     newMinHeap(size),
		gScore:   make([]int32, size),
		gGen:     make([]int32, size),
		cameFrom: make([]int32, size),
		size:     size,
	}
}

// FindPath returns shortest path found by JPS (8-directional).
func (j *JPS) FindPath(start, goal Point) []Point {
	return j.findPath(start, goal)
}

func (j *JPS) findPath(start, goal Point) []Point {
	if !j.grid.Walkable(start.X, start.Y) || !j.grid.Walkable(goal.X, goal.Y) {
		return nil
	}
	if start == goal {
		return []Point{start}
	}

	startIdx := j.grid.Index(start.X, start.Y)
	goalIdx := j.grid.Index(goal.X, goal.Y)

	j.closed.nextGen()
	j.gCurGen++
	j.open.clear()

	j.setG(startIdx, 0)
	j.cameFrom[startIdx] = invalid32
	j.open.pushOrUpdate(startIdx, Octile(start, goal))

	for !j.open.empty() {
		current, _ := j.open.pop()
		if current == goalIdx {
			return j.reconstructPath(current, startIdx)
		}
		if j.closed.visited(current) {
			continue
		}
		j.closed.visit(current)

		cx, cy := j.grid.PointFromIndex(current)
		gCur := j.getG(current)

		successors := j.identifySuccessors(cx, cy, gCur, goal)
		for _, succ := range successors {
			si := succ.idx
			if j.closed.visited(si) {
				continue
			}
			tentativeG := succ.cost
			if tentativeG < j.getG(si) {
				j.setG(si, tentativeG)
				j.cameFrom[si] = current
				f := tentativeG + Octile(Point{succ.x, succ.y}, goal)
				j.open.pushOrUpdate(si, f)
			}
		}
	}
	return nil
}

type jpNode struct {
	x, y int32
	idx  int32
	cost int32
}

func (j *JPS) identifySuccessors(cx, cy int32, gCur int32, goal Point) []jpNode {
	gx, gy := goal.X, goal.Y
	succ := make([]jpNode, 0, 8)

	for _, d := range Dir8 {
		// Prune: only jump in directions that move toward goal.
		ndx := gx - cx
		ndy := gy - cy
		if d.X != 0 && (ndx > 0) != (d.X > 0) {
			continue
		}
		if d.Y != 0 && (ndy > 0) != (d.Y > 0) {
			continue
		}

		jx, jy, dist := j.jump(cx, cy, d.X, d.Y, gx, gy)
		if jx != invalid32 {
			succ = append(succ, jpNode{
				x: jx, y: jy,
				idx:  j.grid.Index(jx, jy),
				cost: gCur + dist,
			})
		}
	}
	return succ
}

const maxJumpDist int32 = 500

// jump iteratively scans in direction (dx,dy) for a jump point.
// Returns (x, y, distance). Returns (-1, -1, 0) if blocked or no jp found.
func (j *JPS) jump(sx, sy, dx, dy, gx, gy int32) (int32, int32, int32) {
	x, y := sx+dx, sy+dy
	dist := int32(1)

	for dist <= maxJumpDist {
		if !j.grid.Walkable(x, y) {
			return invalid32, invalid32, 0
		}
		if x == gx && y == gy {
			return x, y, dist
		}

		if dx != 0 && dy != 0 {
			if j.hasForcedDiag(x, y, dx, dy) {
				return x, y, dist
			}
			if _, _, d := j.scanCardinal(x, y, dx, 0, gx, gy); d > 0 {
				return x, y, dist
			}
			if _, _, d := j.scanCardinal(x, y, 0, dy, gx, gy); d > 0 {
				return x, y, dist
			}
		} else if dy == 0 {
			if j.hasForcedHoriz(x, y, dx) {
				return x, y, dist
			}
		} else {
			if j.hasForcedVert(x, y, dy) {
				return x, y, dist
			}
		}

		x += dx
		y += dy
		dist++
	}
	return invalid32, invalid32, 0
}

// scanCardinal scans horizontally or vertically from a diagonal position.
func (j *JPS) scanCardinal(sx, sy, dx, dy, gx, gy int32) (int32, int32, int32) {
	x, y := sx+dx, sy+dy
	dist := int32(1)
	for dist <= maxJumpDist {
		if !j.grid.Walkable(x, y) {
			return invalid32, invalid32, 0
		}
		if x == gx && y == gy {
			return x, y, dist
		}
		if dy == 0 {
			if j.hasForcedHoriz(x, y, dx) {
				return x, y, dist
			}
		} else {
			if j.hasForcedVert(x, y, dy) {
				return x, y, dist
			}
		}
		x += dx
		y += dy
		dist++
	}
	return invalid32, invalid32, 0
}

func (j *JPS) hasForcedDiag(x, y, dx, dy int32) bool {
	return (j.grid.Walkable(x-dx, y) && !j.grid.Walkable(x-dx, y-dy)) ||
		(j.grid.Walkable(x, y-dy) && !j.grid.Walkable(x-dx, y-dy))
}

func (j *JPS) hasForcedHoriz(x, y, dx int32) bool {
	return (j.grid.Walkable(x, y-1) && !j.grid.Walkable(x-dx, y-1)) ||
		(j.grid.Walkable(x, y+1) && !j.grid.Walkable(x-dx, y+1))
}

func (j *JPS) hasForcedVert(x, y, dy int32) bool {
	return (j.grid.Walkable(x-1, y) && !j.grid.Walkable(x-1, y-dy)) ||
		(j.grid.Walkable(x+1, y) && !j.grid.Walkable(x+1, y-dy))
}

func (j *JPS) getG(idx int32) int32 {
	if j.gGen[idx] < j.gCurGen {
		return inf32
	}
	return j.gScore[idx]
}

func (j *JPS) setG(idx, val int32) {
	j.gScore[idx] = val
	j.gGen[idx] = j.gCurGen
}

func (j *JPS) reconstructPath(current, start int32) []Point {
	type jp struct {
		idx int32
		x, y int32
	}
	var jps []jp
	for c := current; c != invalid32 && int32(len(jps)) < j.size; c = j.cameFrom[c] {
		x, y := j.grid.PointFromIndex(c)
		jps = append(jps, jp{c, x, y})
	}
	if len(jps) >= int(j.size) {
		return nil
	}
	for i, k := 0, len(jps)-1; i < k; i, k = i+1, k-1 {
		jps[i], jps[k] = jps[k], jps[i]
	}

	var path []Point
	for i, jp := range jps {
		if i == 0 {
			path = append(path, Point{jp.x, jp.y})
			continue
		}
		prev := jps[i-1]
		dx := sign(jp.x - prev.x)
		dy := sign(jp.y - prev.y)
		cx, cy := prev.x+dx, prev.y+dy
		for cx != jp.x || cy != jp.y {
			path = append(path, Point{cx, cy})
			cx += dx
			cy += dy
		}
		path = append(path, Point{jp.x, jp.y})
	}
	return path
}

func sign(v int32) int32 {
	if v > 0 {
		return 1
	}
	if v < 0 {
		return -1
	}
	return 0
}
