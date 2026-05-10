package pathfind

const (
	inf32     int32 = 1 << 30
	invalid32 int32 = -1
)

// Point on 2D grid.
type Point struct {
	X, Y int32
}

// Grid map with obstacles. true = walkable.
type Grid struct {
	Width, Height int32
	Cells         []bool // row-major: cells[y*Width + x]
}

// NewGrid creates Width×Height grid, all walkable.
func NewGrid(w, h int32) *Grid {
	cells := make([]bool, int(w)*int(h))
	for i := range cells {
		cells[i] = true
	}
	return &Grid{
		Width:  w,
		Height: h,
		Cells:  cells,
	}
}

// SetWalkable sets cell walkability.
func (g *Grid) SetWalkable(x, y int32, walkable bool) {
	if x < 0 || x >= g.Width || y < 0 || y >= g.Height {
		return
	}
	g.Cells[int(y)*int(g.Width)+int(x)] = walkable
}

// SetBlocked sets cell as obstacle.
func (g *Grid) SetBlocked(x, y int32) {
	g.SetWalkable(x, y, false)
}

// Walkable checks if cell is walkable. Out of bounds = false.
func (g *Grid) Walkable(x, y int32) bool {
	if x < 0 || x >= g.Width || y < 0 || y >= g.Height {
		return false
	}
	return g.Cells[int(y)*int(g.Width)+int(x)]
}

// InBounds checks if point is within grid.
func (g *Grid) InBounds(x, y int32) bool {
	return x >= 0 && x < g.Width && y >= 0 && y < g.Height
}

// Index returns flat index for (x,y).
func (g *Grid) Index(x, y int32) int32 {
	return y*g.Width + x
}

// PointFromIndex returns (x,y) from flat index.
func (g *Grid) PointFromIndex(idx int32) (int32, int32) {
	return idx % g.Width, idx / g.Width
}

// Manhattan distance.
func Manhattan(a, b Point) int32 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}
	return dx + dy
}

// Octile distance (diagonal allowed, cost sqrt2 for diagonal).
func Octile(a, b Point) int32 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}
	if dx < dy {
		return dy
	}
	return dx
}

// Directions: 4-directional.
var Dir4 = [4]Point{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}

// Directions: 8-directional.
var Dir8 = [8]Point{
	{0, -1}, {1, -1}, {1, 0}, {1, 1},
	{0, 1}, {-1, 1}, {-1, 0}, {-1, -1},
}

// HexNeighbors returns the 6 neighbors of (x,y) in odd-r offset hex grid.
// odd rows are shifted right by half a cell.
// Returns slice of valid (walkable, in-bounds) neighbor points.
func (g *Grid) HexNeighbors(x, y int32) []Point {
	even := [6]Point{
		{1, 0}, {0, -1}, {-1, -1},
		{-1, 0}, {-1, 1}, {0, 1},
	}
	odd := [6]Point{
		{1, 0}, {1, -1}, {0, -1},
		{-1, 0}, {0, 1}, {1, 1},
	}
	var offsets *[6]Point
	if y&1 == 0 {
		offsets = &even
	} else {
		offsets = &odd
	}
	neighbors := make([]Point, 0, 6)
	for _, d := range offsets {
		nx, ny := x+d.X, y+d.Y
		if g.Walkable(nx, ny) {
			neighbors = append(neighbors, Point{nx, ny})
		}
	}
	return neighbors
}

// HexNeighbors6 returns the 6 offsets for the given y parity (even row or odd row).
// Does not check walkability. Use for iteration where bounds/walkable checks are separate.
func HexNeighbors6(y int32) *[6]Point {
	if y&1 == 0 {
		return &[6]Point{
			{1, 0}, {0, -1}, {-1, -1},
			{-1, 0}, {-1, 1}, {0, 1},
		}
	}
	return &[6]Point{
		{1, 0}, {1, -1}, {0, -1},
		{-1, 0}, {0, 1}, {1, 1},
	}
}

// HexDistance returns the hex grid distance between a and b in odd-r offset coords.
func HexDistance(a, b Point) int32 {
	// Convert odd-r offset to axial coords (q, r).
	ax := a.X - (a.Y-(a.Y&1))/2
	ay := a.Y
	bx := b.X - (b.Y-(b.Y&1))/2
	by := b.Y

	dq := ax - bx
	dr := ay - by

	if dq < 0 {
		dq = -dq
	}
	if dr < 0 {
		dr = -dr
	}
	ds := (ax + ay) - (bx + by)
	if ds < 0 {
		ds = -ds
	}

	// Hex distance = (|dq| + |dr| + |ds|) / 2
	return (dq + dr + ds) / 2
}

// heuristic picks Manhattan for 4-dir, Octile for 8-dir, HexDistance for 6-dir.
func heuristic(a, b Point, dirs int) int32 {
	switch dirs {
	case 8:
		return Octile(a, b)
	case 6:
		return HexDistance(a, b)
	default:
		return Manhattan(a, b)
	}
}
