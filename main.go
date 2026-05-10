package main

import (
	"fmt"
	"math/rand"
	"time"

	"path_find/pathfind"
)

func main() {
	const W, H = 1500, 1500
	grid := pathfind.NewGrid(W, H)
	rng := rand.New(rand.NewSource(42))

	// SLG-realistic: clustered obstacles (~2% density).
	numClusters := 30
	for i := 0; i < numClusters; i++ {
		cx := rng.Int31n(W)
		cy := rng.Int31n(H)
		size := int32(40 + rng.Intn(80))
		x, y := cx, cy
		for j := int32(0); j < size; j++ {
			if grid.InBounds(x, y) {
				grid.SetBlocked(x, y)
			}
			dx := int32(rng.Intn(3) - 1)
			dy := int32(rng.Intn(3) - 1)
			x += dx
			y += dy
			if x < 0 {
				x = 0
			}
			if x >= W {
				x = W - 1
			}
			if y < 0 {
				y = 0
			}
			if y >= H {
				y = H - 1
			}
		}
	}

	tests := []struct {
		name  string
		start pathfind.Point
		goal  pathfind.Point
	}{
		{"近距50", pathfind.Point{20, 20}, pathfind.Point{70, 20}},
		{"中距200", pathfind.Point{20, 20}, pathfind.Point{220, 150}},
		{"远距500", pathfind.Point{20, 20}, pathfind.Point{520, 400}},
		{"远距1000", pathfind.Point{30, 30}, pathfind.Point{1000, 800}},
		{"对角线600", pathfind.Point{100, 100}, pathfind.Point{700, 700}},
	}

	for _, t := range tests {
		clearCorridor(grid, t.start, t.goal, 5)
	}

	fmt.Println("=== SLG 寻路系统 (1500x1500, 聚集障碍~2%) ===\n")

	// ---- A* (4/6/8方向) ----
	astar := pathfind.NewAStar(grid)
	fmt.Println("--- 标准 A* ---")
	for _, t := range tests {
		runTest("A*4", t.name, func() []pathfind.Point { return astar.FindPath(t.start, t.goal) })
	}
	for _, t := range tests {
		runTest("A*6", t.name, func() []pathfind.Point { return astar.FindPath6(t.start, t.goal) })
	}
	for _, t := range tests {
		runTest("A*8", t.name, func() []pathfind.Point { return astar.FindPath8(t.start, t.goal) })
	}

	// ---- Bidirectional A* ----
	biastar := pathfind.NewBiAStar(grid)
	fmt.Println("\n--- 双向 A* ---")
	for _, t := range tests {
		runTest("双向4", t.name, func() []pathfind.Point { return biastar.FindPath(t.start, t.goal) })
	}
	for _, t := range tests {
		runTest("双向6", t.name, func() []pathfind.Point { return biastar.FindPath6(t.start, t.goal) })
	}

	// ---- JPS (8-dir only, no hex) ----
	fmt.Println("\n--- JPS (8方向) ---")
	jps := pathfind.NewJPS(grid)
	for _, t := range tests {
		runTest("JPS", t.name, func() []pathfind.Point { return jps.FindPath(t.start, t.goal) })
	}

	// ---- HPA* (4/6/8方向) ----
	fmt.Println("\n--- 分层 HPA* (chunk=50) ---")
	hpa := pathfind.NewHPA(grid, 50)
	startBuild := time.Now()
	hpa.Build()
	fmt.Printf("  抽象图构建耗时: %v\n", time.Since(startBuild))
	for _, t := range tests {
		runTest("HPA4", t.name, func() []pathfind.Point { return hpa.FindPath(t.start, t.goal) })
	}
	for _, t := range tests {
		runTest("HPA6", t.name, func() []pathfind.Point { return hpa.FindPath6(t.start, t.goal) })
	}

	// ---- Flow Field (4/6/8) ----
	fmt.Println("\n--- Flow Field (群体移动) ---")
	for dx := int32(-5); dx <= 5; dx++ {
		for dy := int32(-5); dy <= 5; dy++ {
			grid.SetWalkable(750+dx, 750+dy, true)
		}
	}
	ff := pathfind.NewFlowField(grid)
	ff.Dirs = 8
	t1 := time.Now()
	ff.Build(pathfind.Point{750, 750})
	fmt.Printf("  Flow Field(8dir)构建: %v\n", time.Since(t1))

	ff.Dirs = 6
	t1 = time.Now()
	ff.Build(pathfind.Point{750, 750})
	fmt.Printf("  Flow Field(6dir)构建: %v\n", time.Since(t1))

	ff.Dirs = 4
	t1 = time.Now()
	ff.Build(pathfind.Point{750, 750})
	fmt.Printf("  Flow Field(4dir)构建: %v\n", time.Since(t1))

	// ---- Throughput test ----
	fmt.Println("\n=== 吞吐量 (中距300格, 单goroutine, 1s) ===")
	mids := pathfind.Point{40, 40}
	midg := pathfind.Point{340, 260}
	clearCorridor(grid, mids, midg, 5)

	throughputTest("A*4  ", func() []pathfind.Point { return astar.FindPath(mids, midg) }, 1*time.Second)
	throughputTest("A*6  ", func() []pathfind.Point { return astar.FindPath6(mids, midg) }, 1*time.Second)
	throughputTest("A*8  ", func() []pathfind.Point { return astar.FindPath8(mids, midg) }, 1*time.Second)
	throughputTest("双向4 ", func() []pathfind.Point { return biastar.FindPath(mids, midg) }, 1*time.Second)
	throughputTest("双向6 ", func() []pathfind.Point { return biastar.FindPath6(mids, midg) }, 1*time.Second)
	throughputTest("JPS  ", func() []pathfind.Point { return jps.FindPath(mids, midg) }, 1*time.Second)
	throughputTest("HPA4 ", func() []pathfind.Point { return hpa.FindPath(mids, midg) }, 1*time.Second)
	throughputTest("HPA6 ", func() []pathfind.Point { return hpa.FindPath6(mids, midg) }, 1*time.Second)

	// ---- Path Cache ----
	fmt.Println("\n--- 路径缓存 ---")
	cache := pathfind.NewPathCache(10000)
	sx := mids.X / 50
	sy := mids.Y / 50
	gx := midg.X / 50
	gy := midg.Y / 50

	t0 := time.Now()
	path := hpa.FindPath(mids, midg)
	fmt.Printf("  首次寻路: %v (路径长度=%d)\n", time.Since(t0), len(path))
	cache.Set(sx, sy, gx, gy, path)

	t0 = time.Now()
	cached, found := cache.Get(sx, sy, gx, gy)
	if found {
		fmt.Printf("  缓存命中: %v (路径长度=%d)\n", time.Since(t0), len(cached))
	}
	fmt.Printf("  缓存大小: %d entries\n", cache.Size())
}

func clearCorridor(grid *pathfind.Grid, a, b pathfind.Point, radius int32) {
	for dx := -radius; dx <= radius; dx++ {
		for dy := -radius; dy <= radius; dy++ {
			grid.SetWalkable(a.X+dx, a.Y+dy, true)
			grid.SetWalkable(b.X+dx, b.Y+dy, true)
		}
	}
	steps := int32(100)
	for i := int32(0); i <= steps; i++ {
		x := a.X + (b.X-a.X)*i/steps
		y := a.Y + (b.Y-a.Y)*i/steps
		for dx := -radius; dx <= radius; dx++ {
			for dy := -radius; dy <= radius; dy++ {
				grid.SetWalkable(x+dx, y+dy, true)
			}
		}
	}
}

func runTest(algo, name string, fn func() []pathfind.Point) {
	start := time.Now()
	path := fn()
	elapsed := time.Since(start)
	if path == nil {
		fmt.Printf("  [%s] %s: 无可达路径 (%v)\n", algo, name, elapsed)
		return
	}
	fmt.Printf("  [%s] %s: %d步, %v\n", algo, name, len(path)-1, elapsed)
}

func throughputTest(name string, fn func() []pathfind.Point, duration time.Duration) {
	count := 0
	deadline := time.Now().Add(duration)
	for time.Now().Before(deadline) {
		fn()
		count++
	}
	perSec := float64(count) / duration.Seconds()
	fmt.Printf("  %s %d次 / %.0f req/s\n", name, count, perSec)
}
