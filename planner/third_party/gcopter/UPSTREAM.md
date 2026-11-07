# Vendored upstream: GCOPTER

- Repository: https://github.com/ZJU-FAST-Lab/GCOPTER
- Commit: `e0444f6d47b84f972ced91746b05feb36ce1fd4f` (2023-06-06, the current `main`)
- Copied: `gcopter/include/gcopter/*.hpp` (all 13 headers) to `include/gcopter/`,
  and the top-level `LICENSE` (MIT, Copyright (c) 2021 Zhepei Wang and Fei Gao).
- Not copied: `gcopter/include/misc/visualizer.hpp` (ROS/rviz only),
  `gcopter/src/global_planning.cpp` (ROS node; its pipeline is reproduced in
  `../../src/main.cpp` and `../../tests/upstream_demo.cpp`), `map_gen/mockamap`
  (GPLv3; not vendored, our test map generator is written from scratch).

The headers are **unmodified**. Verify with `sha256sum -c` from `include/gcopter/`:

```
e47441312b31dff209a59798a8414cbdf310534720e984c73894cb7ea1beb6ee  firi.hpp
6b7f45b5b313a3981b3eb79a63e37f8025c262d90d6edc6eeabb6503ec9390b2  flatness.hpp
652f2b63087294207c3bbcc9d58cbde893179e8d1aeb9ad4b0c7aaa7eccbd339  gcopter.hpp
9a1e9b70c9a15ef46c2f8658df293f29051e98ab7dac5ffad129bfa8b9d87031  geo_utils.hpp
fdc6b0d4c4b4c50e0368d6983dd28ffd893c824df461aebe9e3b24dd8a99b3ca  lbfgs.hpp
60defee515dd3c96cdc54f3dbfa279b84556c92fce92b77365cc37b70649b512  minco.hpp
c0d2668c2c87ff9dfddc750d2bca9f91e4846f8fa78beffe44def0736b5e0e21  quickhull.hpp
c5eaa3ebed6bc65b5671db8965c0c6e3f79cb0cfa9787c89822315cdbac9d781  root_finder.hpp
d7b7bd66597ca1355d7bf16d902cf17da638ad0fd98870673f209a329ded0abf  sdlp.hpp
ac69935d3ee2b4192e75a2d7efbcd31082f706c8517fb951e714b860b3e6a776  sfc_gen.hpp
9ba9b8d43be34848f9f3bd955d5bf8c8922e85757255a7363c82fe59a8a2e9fd  trajectory.hpp
80ec4e25859c5758b2f80f538267cfb1eb25ef121d394307b3802a97df63f3e7  voxel_dilater.hpp
4209beca9876eb1e71e459490fbdfb5152793cd6b10626faedc1123c6fab7e1f  voxel_map.hpp
```
