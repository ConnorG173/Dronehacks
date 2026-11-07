// Parity harness: upstream gcopter/src/global_planning.cpp (commit e0444f6)
// with only ROS removed, so its result can be compared with gcopter_planner.
//
//   upstream_demo map OUT.bin
//       Writes a mockamap-style "perlin3d" map with the parameters of upstream
//       launch/global_planning.launch (seed 1024, 0.25 m, 50x50x5 m,
//       complexity 0.025, fill 0.3, fractal 1, attenuation 0.1) and prints a
//       free start and goal near opposite corners. Written from scratch
//       (mockamap is GPLv3); Ken Perlin's improved noise, seeded shuffle.
//   upstream_demo plan MAP.bin SX SY SZ GX GY GZ [ROUTE.txt]
//       Runs upstream mapCallBack() + plan() on MAP.bin with the parameters of
//       upstream config/global_planning.yaml (hard-coded below, on purpose:
//       not read from our course.yaml). With ROUTE.txt, the front-end route is
//       read from the file instead of planned, so the corridor + optimizer can
//       be compared bit-for-bit on an identical route.

// Include order matters: upstream headers rely on things global_planning.cpp
// pulled in earlier (misc/visualizer.hpp -> geo_utils.hpp; ROS -> <cstdint>).
// gcopter.hpp uses geo_utils without including it; quickhull.hpp uses std::uint8_t.
#include <cstdint>
#include "gcopter/geo_utils.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

// upstream config/global_planning.yaml
struct Config
{
    double dilateRadius = 0.5;
    double voxelWidth = 0.25;
    std::vector<double> mapBound{-25.0, 25.0, -25.0, 25.0, 0.0, 5.0};
    double timeoutRRT = 0.02; // read by upstream but unused: plan() passes 0.01
    double maxVelMag = 4.0;
    double maxBdrMag = 2.1;
    double maxTiltAngle = 1.05;
    double minThrust = 2.0;
    double maxThrust = 12.0;
    double vehicleMass = 0.61;
    double gravAcc = 9.8;
    double horizDrag = 0.70;
    double vertDrag = 0.80;
    double parasDrag = 0.01;
    double speedEps = 0.0001;
    double weightT = 20.0;
    std::vector<double> chiVec{1.0e+4, 1.0e+4, 1.0e+4, 1.0e+4, 1.0e+5};
    double smoothingEps = 1.0e-2;
    int integralIntervs = 16;
    double relCostTol = 1.0e-5;
};

static double msSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// ---------------------------------------------------------------------------
// Map generation

struct Perlin
{
    int p[512];
    explicit Perlin(unsigned seed)
    {
        std::vector<int> v(256);
        std::iota(v.begin(), v.end(), 0);
        std::shuffle(v.begin(), v.end(), std::default_random_engine(seed));
        for (int i = 0; i < 512; i++)
            p[i] = v[i & 255];
    }
    static double fade(double t) { return t * t * t * (t * (t * 6 - 15) + 10); }
    static double lerp(double t, double a, double b) { return a + t * (b - a); }
    static double grad(int hash, double x, double y, double z)
    {
        const int h = hash & 15;
        const double u = h < 8 ? x : y, v = h < 4 ? y : (h == 12 || h == 14) ? x : z;
        return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
    }
    double noise(double x, double y, double z) const // in [0, 1]
    {
        const int X = (int)std::floor(x) & 255, Y = (int)std::floor(y) & 255, Z = (int)std::floor(z) & 255;
        x -= std::floor(x);
        y -= std::floor(y);
        z -= std::floor(z);
        const double u = fade(x), v = fade(y), w = fade(z);
        const int A = p[X] + Y, AA = p[A] + Z, AB = p[A + 1] + Z;
        const int B = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;
        const double r =
            lerp(w, lerp(v, lerp(u, grad(p[AA], x, y, z), grad(p[BA], x - 1, y, z)),
                         lerp(u, grad(p[AB], x, y - 1, z), grad(p[BB], x - 1, y - 1, z))),
                 lerp(v, lerp(u, grad(p[AA + 1], x, y, z - 1), grad(p[BA + 1], x - 1, y, z - 1)),
                      lerp(u, grad(p[AB + 1], x, y - 1, z - 1), grad(p[BB + 1], x - 1, y - 1, z - 1))));
        return (r + 1.0) / 2.0;
    }
};

static int makeMap(const std::string &out)
{
    const unsigned seed = 1024;
    const double res = 0.25, complexity = 0.025, fill = 0.3, attenuation = 0.1;
    const int fractal = 1, nx = 200, ny = 200, nz = 20; // 50 x 50 x 5 m

    const Perlin noise(seed);
    auto value = [&](int i, int j, int k)
    {
        double t = 0.0;
        for (int it = 1; it <= fractal; ++it)
        {
            const int dfv = 1 << it;
            t += attenuation / it * noise.noise(dfv * i * complexity, dfv * j * complexity, dfv * k * complexity);
        }
        return t;
    };
    std::vector<double> v;
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j)
            for (int k = 0; k < nz; ++k)
                v.push_back(value(i, j, k));
    std::vector<double> sorted = v;
    std::sort(sorted.begin(), sorted.end());
    const double threshold = sorted[(size_t)(v.size() * (1.0 - fill))];

    std::vector<float> pts;
    size_t idx = 0;
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j)
            for (int k = 0; k < nz; ++k)
                if (v[idx++] > threshold)
                {
                    pts.push_back((float)(i * res - 25.0));
                    pts.push_back((float)(j * res - 25.0));
                    pts.push_back((float)(k * res));
                }

    std::ofstream f(out, std::ios::binary);
    const uint32_t version = 1, reserved = 0;
    const uint64_t n = pts.size() / 3;
    f.write("DHMAP\0\0\0", 8);
    f.write((const char *)&version, 4);
    f.write((const char *)&reserved, 4);
    f.write((const char *)&n, 8);
    f.write((const char *)pts.data(), (std::streamsize)(pts.size() * sizeof(float)));
    if (!f)
        return std::cerr << "cannot write " << out << "\n", 1;

    // Free start/goal near opposite corners, in the dilated map, at z = 2.5.
    const Config config;
    voxel_map::VoxelMap vm(Eigen::Vector3i(200, 200, 20), Eigen::Vector3d(-25, -25, 0), 0.25);
    for (size_t i = 0; i < pts.size(); i += 3)
        vm.setOccupied(Eigen::Vector3d(pts[i], pts[i + 1], pts[i + 2]));
    vm.dilate(std::ceil(config.dilateRadius / vm.getScale()));
    for (const double corner : {-20.0, 20.0})
    {
        bool found = false;
        for (int r = 0; r < 40 && !found; r++)
            for (int dx = -r; dx <= r && !found; dx++)
                for (int dy = -r; dy <= r && !found; dy++)
                {
                    const Eigen::Vector3d q(corner + 0.25 * dx + 0.125, corner + 0.25 * dy + 0.125, 2.625);
                    if (std::max(std::abs(dx), std::abs(dy)) == r && vm.query(q) == 0)
                    {
                        std::printf("%s %.17g %.17g %.17g\n", corner < 0 ? "start" : "goal", q(0), q(1), q(2));
                        found = true;
                    }
                }
        if (!found)
            return std::cerr << "no free cell near corner\n", 1;
    }
    std::printf("points %llu\n", (unsigned long long)n);
    return 0;
}

// ---------------------------------------------------------------------------
// Upstream mapCallBack() + plan(), ROS and visualizer removed.

static int runPlan(int argc, char **argv)
{
    const Config config;
    const std::string mapPath = argv[2];
    std::vector<Eigen::Vector3d> startGoal{
        Eigen::Vector3d(std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5])),
        Eigen::Vector3d(std::atof(argv[6]), std::atof(argv[7]), std::atof(argv[8]))};
    const std::string routePath = argc > 9 ? argv[9] : "";

    // GlobalPlanner constructor
    const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                              (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                              (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);
    const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);
    voxel_map::VoxelMap voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

    // mapCallBack: the PointCloud2 is replaced by the DHMAP file's float32 xyz
    std::ifstream f(mapPath, std::ios::binary);
    char hdr[24];
    f.read(hdr, 24);
    uint64_t total = 0;
    std::memcpy(&total, hdr + 16, 8);
    std::vector<float> data(3 * total);
    f.read((char *)data.data(), (std::streamsize)(data.size() * sizeof(float)));
    if (!f || std::memcmp(hdr, "DHMAP\0\0\0", 8) != 0)
        return std::cerr << "bad map " << mapPath << "\n", 1;
    {
        size_t cur = 0;
        float *fdata = data.data();
        for (size_t i = 0; i < total; i++)
        {
            cur = 3 * i;

            if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
            {
                continue;
            }
            voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                 fdata[cur + 1],
                                                 fdata[cur + 2]));
        }

        voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));
    }
    for (const Eigen::Vector3d &p : startGoal)
        if (voxelMap.query(p) != 0)
            return std::cerr << "Infeasible Position Selected !!!\n", 1;

    // plan()
    Trajectory<5> traj;
    const auto tFront = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector3d> route;
    if (routePath.empty())
    {
        sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                               startGoal[1],
                                               voxelMap.getOrigin(),
                                               voxelMap.getCorner(),
                                               &voxelMap, 0.01,
                                               route);
    }
    else
    {
        std::ifstream rf(routePath);
        double x, y, z;
        while (rf >> x >> y >> z)
            route.emplace_back(x, y, z);
        if (route.size() < 2)
            return std::cerr << "route file " << routePath << " has fewer than 2 points\n", 1;
    }
    const double frontMs = msSince(tFront);
    const auto tCorr = std::chrono::steady_clock::now();
    std::vector<Eigen::MatrixX4d> hPolys;
    std::vector<Eigen::Vector3d> pc;
    voxelMap.getSurf(pc);

    sfc_gen::convexCover(route,
                         pc,
                         voxelMap.getOrigin(),
                         voxelMap.getCorner(),
                         7.0,
                         3.0,
                         hPolys);
    sfc_gen::shortCut(hPolys);
    const double corrMs = msSince(tCorr);
    const auto tOpt = std::chrono::steady_clock::now();

    if (route.size() > 1)
    {
        Eigen::Matrix3d iniState;
        Eigen::Matrix3d finState;
        iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

        gcopter::GCOPTER_PolytopeSFC gcopter;

        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        // initialize some constraint parameters
        Eigen::VectorXd magnitudeBounds(5);
        Eigen::VectorXd penaltyWeights(5);
        Eigen::VectorXd physicalParams(6);
        magnitudeBounds(0) = config.maxVelMag;
        magnitudeBounds(1) = config.maxBdrMag;
        magnitudeBounds(2) = config.maxTiltAngle;
        magnitudeBounds(3) = config.minThrust;
        magnitudeBounds(4) = config.maxThrust;
        penaltyWeights(0) = (config.chiVec)[0];
        penaltyWeights(1) = (config.chiVec)[1];
        penaltyWeights(2) = (config.chiVec)[2];
        penaltyWeights(3) = (config.chiVec)[3];
        penaltyWeights(4) = (config.chiVec)[4];
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;
        const int quadratureRes = config.integralIntervs;

        traj.clear();

        if (!gcopter.setup(config.weightT,
                           iniState, finState,
                           hPolys, INFINITY,
                           config.smoothingEps,
                           quadratureRes,
                           magnitudeBounds,
                           penaltyWeights,
                           physicalParams))
        {
            return std::cerr << "setup failed\n", 1;
        }

        if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
        {
            return std::cerr << "optimize failed\n", 1;
        }
    }
    const double optMs = msSince(tOpt);
    if (traj.getPieceNum() <= 0)
        return std::cerr << "no trajectory (route size " << route.size() << ")\n", 1;

    std::printf("RESULT duration=%.17g front_ms=%.3f corridor_ms=%.3f opt_ms=%.3f\n",
                traj.getTotalDuration(), frontMs, corrMs, optMs);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc == 3 && std::string(argv[1]) == "map")
        return makeMap(argv[2]);
    if ((argc == 9 || argc == 10) && std::string(argv[1]) == "plan")
        return runPlan(argc, argv);
    std::cerr << "usage: upstream_demo map OUT.bin\n"
                 "       upstream_demo plan MAP.bin SX SY SZ GX GY GZ [ROUTE.txt]\n";
    return 2;
}
