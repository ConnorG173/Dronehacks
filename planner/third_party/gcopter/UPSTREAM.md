# Vendored upstream: GCOPTER

- Repository: https://github.com/ZJU-FAST-Lab/GCOPTER
- Commit: `e0444f6d47b84f972ced91746b05feb36ce1fd4f` (2023-06-06, the current `main`)
- Copied: `gcopter/include/gcopter/*.hpp` (all 13 headers) to `include/gcopter/`,
  and the top-level `LICENSE` (MIT, Copyright (c) 2021 Zhepei Wang and Fei Gao).
- Not copied: `gcopter/include/misc/visualizer.hpp` (ROS/rviz only),
  `gcopter/src/global_planning.cpp` (ROS node; its pipeline is reproduced in
  `../../src/main.cpp` and `../../tests/upstream_demo.cpp`), `map_gen/mockamap`
  (GPLv3; not vendored, our test map generator is written from scratch).

## Local changes

Three changes: two in `include/gcopter/geo_utils.hpp`, one in `include/gcopter/sfc_gen.hpp`; the
other 11 headers are byte-identical to upstream. Reproduce the full diff with
`git -C <upstream clone> show e0444f6:gcopter/include/gcopter/<file> | diff - include/gcopter/<file>`.

### 1. Compiler fix: `filterLess::operator()` made `const` (line 90)

```diff
-                               const Eigen::Vector3d &r)
+                               const Eigen::Vector3d &r) const
```

Why: `filterLess` is the comparator of the `std::set` in `geo_utils::filterVs`
(vertex de-duplication during `gcopter.setup`). The standard library requires
the comparator to be callable as `const`; GCC 16 enforces this in every C++ mode
(older GCC only in C++17 mode, and upstream builds with `-std=c++14`). The
comparison itself is unchanged, so results are identical.

### 2. Bug fix: drop bogus vertices in `enumerateVs` (after line 151)

```diff
             rV.col(i) = normal / normal.dot(point);
         }
+        // DroneHacks fix: ... (comment)
+        int nValid = 0;
+        for (int i = 0; i < hNum; i++)
+        {
+            const Eigen::Vector3d v = rV.col(i) + inner;
+            const double slack = (hPoly.leftCols<3>() * v + hPoly.rightCols<1>()).maxCoeff();
+            if (slack <= 1.0e-6 * (1.0 + v.norm()) * hPoly.leftCols<3>().rowwise().norm().maxCoeff())
+                rV.col(nValid++) = rV.col(i);
+        }
+        rV.conservativeResize(3, nValid);
         filterVs(rV, epsilon, vPoly);
```

What goes wrong upstream: `enumerateVs` gets a polytope's vertices from the
convex hull of its dual points (one per plane). Consecutive corridor polytopes
often share planes (every `convexCover` polytope is clipped to the map bounds),
and duplicate or near-coplanar dual points make quickhull return (near)
zero-area faces. Such a face's "vertex" `normal / normal.dot(point)` is NaN,
infinite, or a finite point far outside the polytope. A single NaN also makes
`filterVs` (whose rounding scale is the max over all vertices) collapse every
vertex into one NaN. Effects observed, on upstream's own demo map as well as
ours:

- NaN shortest path in `gcopter.setup` -> `pieceIdx = INT_MIN` -> Eigen assertion
  in `setInitial` (process abort; undefined behaviour with `-DNDEBUG`);
- huge initial costs (1e16 .. 1e83) -> L-BFGS fails ("negative line-search step",
  "step smaller than min_step"), or "succeeds" with a useless trajectory
  (seen: 21.7 m/s and 115 deg tilt against 4 m/s / 60 deg limits);
- subtly worse trajectories (one upstream-map route: 34.9 s vs 20.5 s fixed).

The fix keeps only vertices that satisfy every inequality of the polytope (with
a 1e-6 relative tolerance; `NaN <= x` is false, so NaN/inf are dropped too). A
true vertex always satisfies them, so the real vertex set is unchanged; it also
appears from the hull's proper faces. Evidence (`tests/upstream_demo.cpp`
built before/after, same RRT routes on the upstream-style map): every dropped
vertex was NaN/inf or at least 1.8 cm outside its polytope (tolerance about
3e-5 m), and bogus vertices occurred in most routes. On the wall course with
one or two checkpoints, optimizer failures went from about 1 in 4 runs to 0
in 100 and the assertion crashes to 0.

### 3. Crash fix: straight collision-free legs skip OMPL in `sfc_gen::planPath` (top of the function)

```diff
                            std::vector<Eigen::Vector3d> &p)
     {
+        // DroneHacks fix: ... (comment)
+        {
+            const double step = 0.5 * mapPtr->getScale();
+            const int n = std::max(2, (int)std::ceil((g - s).norm() / step));
+            bool free = true;
+            for (int i = 0; i <= n && free; i++)
+            {
+                const Eigen::Vector3d q = s + (g - s) * ((double)i / n);
+                free = mapPtr->query(q) == 0;
+            }
+            if (free)
+            {
+                p.clear();
+                p.push_back(s);
+                p.push_back(g);
+                return (g - s).norm();
+            }
+        }
+
         auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
```

What goes wrong upstream (2026-11-07, OMPL 1.7.0 from `ros-humble-ompl`, the version in the drone's
skybeta image; seen on aarch64): on some maps the planner dies with SIGSEGV (exit 139, no output),
deterministically on every run. 4 of 8 random courses from `path_following/tools/make_random_map.py`
(seeds 1, 3, 4, 8) crashed; `course_v3` never did. Backtrace (`-g -O1`, gdb):

```
#0 ompl::base::RealVectorStateSpace::freeState(ompl::base::State*) const   (libompl.so.18)
#1 ompl::geometric::RRTstar::freeMemory()
#2 ompl::geometric::RRTstar::~RRTstar()
#3-6 std::shared_ptr<ompl::geometric::InformedRRTstar>::~shared_ptr()
#7 sfc_gen::planPath<voxel_map::VoxelMap>(...) at sfc_gen.hpp:114, p = std::vector of length 2
#8 main at src/main.cpp:331
```

The solution path has 2 states: the leg is a straight, collision-free segment, and InformedRRTstar's
tree teardown then frees an invalid state. The fix returns such a segment directly (the same 2-point
route OMPL found), so OMPL is only used for legs that need a search. The check uses the same
`query() == 0` test (dilated map) as OMPL's validity checker, sampled every half voxel. Cost returned
is the segment length, like the path-length objective. Results: all 4 crashing courses plan; course_v3
plans as before (13.9–14.4 s, same as before the fix, RRT variance); upstream parity tests unchanged.
Regression test: `tests/run_tests.py` section 5 (random course seed 3: exit -11 x3 before, plans after)
and section 6 (course_v3 still plans).

Not changed in the headers but needed by includers: `gcopter.hpp` uses
`geo_utils` without including it and `quickhull.hpp` uses `std::uint8_t`
without `<cstdint>` (upstream gets both via `misc/visualizer.hpp` and ROS), so
`src/main.cpp` and `tests/upstream_demo.cpp` include `<cstdint>` and
`gcopter/geo_utils.hpp` first.

## Checksums

Verify with `sha256sum -c` from `include/gcopter/` (`geo_utils.hpp` and `sfc_gen.hpp` are the
patched files; upstream's originals are
`9a1e9b70c9a15ef46c2f8658df293f29051e98ab7dac5ffad129bfa8b9d87031` and
`ac69935d3ee2b4192e75a2d7efbcd31082f706c8517fb951e714b860b3e6a776`):

```
e47441312b31dff209a59798a8414cbdf310534720e984c73894cb7ea1beb6ee  firi.hpp
6b7f45b5b313a3981b3eb79a63e37f8025c262d90d6edc6eeabb6503ec9390b2  flatness.hpp
652f2b63087294207c3bbcc9d58cbde893179e8d1aeb9ad4b0c7aaa7eccbd339  gcopter.hpp
8f559ae304086cffec329644893c345668c9055a69f6be8e33a82e34cb08db32  geo_utils.hpp
fdc6b0d4c4b4c50e0368d6983dd28ffd893c824df461aebe9e3b24dd8a99b3ca  lbfgs.hpp
60defee515dd3c96cdc54f3dbfa279b84556c92fce92b77365cc37b70649b512  minco.hpp
c0d2668c2c87ff9dfddc750d2bca9f91e4846f8fa78beffe44def0736b5e0e21  quickhull.hpp
c5eaa3ebed6bc65b5671db8965c0c6e3f79cb0cfa9787c89822315cdbac9d781  root_finder.hpp
d7b7bd66597ca1355d7bf16d902cf17da638ad0fd98870673f209a329ded0abf  sdlp.hpp
b55198c3822b8cc2da9657535013a4df83ac4d39a417e7b03588dbed8238fd23  sfc_gen.hpp
9ba9b8d43be34848f9f3bd955d5bf8c8922e85757255a7363c82fe59a8a2e9fd  trajectory.hpp
80ec4e25859c5758b2f80f538267cfb1eb25ef121d394307b3802a97df63f3e7  voxel_dilater.hpp
4209beca9876eb1e71e459490fbdfb5152793cd6b10626faedc1123c6fab7e1f  voxel_map.hpp
```
