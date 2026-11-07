// gcopter_planner: one GCOPTER trajectory start -> checkpoints -> goal,
// validated and written as a time-sampled PX4 path file. See ../README.md.
//
// The pipeline is upstream gcopter/src/global_planning.cpp (commit e0444f6)
// with ROS removed. "upstream:NNN" comments name the matching line there.
// The only extension is the checkpoint boxes inserted into the corridor.

// Include order matters: upstream headers rely on things global_planning.cpp
// pulled in earlier (misc/visualizer.hpp -> geo_utils.hpp; ROS -> <cstdint>).
// gcopter.hpp uses geo_utils without including it; quickhull.hpp uses std::uint8_t.
#include <cstdint>
#include "gcopter/geo_utils.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using Eigen::Vector3d;

[[noreturn]] static void fail(const std::string &msg)
{
    std::cerr << "ERROR: " << msg << "\nNo path file written." << std::endl;
    std::exit(1);
}

static std::string fmt(const Vector3d &v)
{
    char s[96];
    std::snprintf(s, sizeof(s), "(%.3f, %.3f, %.3f)", v(0), v(1), v(2));
    return s;
}

// ---------------------------------------------------------------------------
// Config: flat "Key: value" lines, '#' comments, numbers separated by spaces
// or commas, optional [ ]. A key may repeat (Checkpoint). Unknown keys fail.

static std::map<std::string, std::vector<std::string>> cfg;
static std::set<std::string> cfgUsed;

static void loadConfig(const std::string &path)
{
    std::ifstream f(path);
    if (!f)
        fail("cannot open config " + path);
    std::string line;
    for (int n = 1; std::getline(f, line); n++)
    {
        line = line.substr(0, line.find('#'));
        for (char &c : line)
            if (c == '[' || c == ']' || c == ',' || c == '\t' || c == '\r')
                c = ' ';
        if (line.find_first_not_of(' ') == std::string::npos)
            continue;
        const size_t colon = line.find(':');
        if (colon == std::string::npos)
            fail(path + ":" + std::to_string(n) + ": expected 'Key: value'");
        std::string key = line.substr(0, colon);
        key.erase(0, key.find_first_not_of(' '));
        key.erase(key.find_last_not_of(' ') + 1);
        cfg[key].push_back(line.substr(colon + 1));
    }
}

static std::vector<double> parseNums(const std::string &key, const std::string &text, size_t n)
{
    std::istringstream ss(text);
    std::vector<double> v;
    std::string tok;
    while (ss >> tok)
    {
        char *end = nullptr;
        const double d = std::strtod(tok.c_str(), &end);
        if (*end != '\0')
            fail("config key " + key + ": '" + tok + "' is not a number");
        v.push_back(d);
    }
    if (v.size() != n)
        fail("config key " + key + ": expected " + std::to_string(n) + " numbers, got " +
             std::to_string(v.size()));
    return v;
}

static std::vector<std::vector<double>> numsAll(const std::string &key, size_t n)
{
    cfgUsed.insert(key);
    std::vector<std::vector<double>> out;
    for (const std::string &text : cfg[key])
        out.push_back(parseNums(key, text, n));
    return out;
}

static std::vector<double> nums(const std::string &key, size_t n)
{
    const auto all = numsAll(key, n);
    if (all.size() != 1)
        fail("config key " + key + " must appear exactly once");
    return all[0];
}

static double num(const std::string &key) { return nums(key, 1)[0]; }
static Vector3d vec3(const std::string &key)
{
    const auto v = nums(key, 3);
    return Vector3d(v[0], v[1], v[2]);
}

static std::string str(const std::string &key)
{
    cfgUsed.insert(key);
    if (cfg[key].size() != 1)
        fail("config key " + key + " must appear exactly once");
    std::istringstream ss(cfg[key][0]);
    std::string s;
    ss >> s;
    return s;
}

// ---------------------------------------------------------------------------
// Map file: 8-byte magic "DHMAP\0\0\0", uint32 version (1), uint32 reserved (0),
// uint64 point count N, then N x (float32 x, y, z) in metres, little-endian.

static std::vector<float> loadMapPoints(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        fail("cannot open map file " + path);
    char magic[8];
    uint32_t version = 0, reserved = 0;
    uint64_t n = 0;
    f.read(magic, 8);
    f.read((char *)&version, 4);
    f.read((char *)&reserved, 4);
    f.read((char *)&n, 8);
    if (!f || std::memcmp(magic, "DHMAP\0\0\0", 8) != 0 || version != 1)
        fail(path + " is not a version-1 DHMAP point file");
    std::vector<float> xyz(3 * n);
    f.read((char *)xyz.data(), (std::streamsize)(xyz.size() * sizeof(float)));
    if ((uint64_t)f.gcount() != xyz.size() * sizeof(float))
        fail(path + " is truncated");
    return xyz;
}

// ---------------------------------------------------------------------------
// Checkpoints: an axis-aligned box in map coordinates, as an H-polytope in the
// format sfc_gen/gcopter use (each row h: h0*x + h1*y + h2*z + h3 <= 0).

struct Box
{
    Vector3d c, h; // center, half-extents
    bool contains(const Vector3d &p) const { return ((p - c).cwiseAbs() - h).maxCoeff() <= 0.0; }
};

static Eigen::MatrixX4d boxPolytope(const Box &b)
{
    Eigen::MatrixX4d H = Eigen::MatrixX4d::Zero(6, 4);
    for (int k = 0; k < 3; k++)
    {
        H(2 * k, k) = 1.0; //  x <= c + h
        H(2 * k, 3) = -(b.c(k) + b.h(k));
        H(2 * k + 1, k) = -1.0; // -x <= -(c - h)
        H(2 * k + 1, 3) = b.c(k) - b.h(k);
    }
    return H;
}

// True if the intersection contains a ball of radius 1 cm. Rows are normalized
// first (as GCOPTER::setup does) so the margin is in metres; the 0.01 is the
// same margin sfc_gen::shortCut uses.
static bool polysOverlap(Eigen::MatrixX4d a, Eigen::MatrixX4d b)
{
    a.array().colwise() /= a.leftCols<3>().rowwise().norm().array();
    b.array().colwise() /= b.leftCols<3>().rowwise().norm().array();
    return geo_utils::overlap(a, b, 0.01);
}

// ---------------------------------------------------------------------------

static double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

static double msSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

int main(int argc, char **argv)
{
    std::string cfgPath, outOverride;
    for (int i = 1; i < argc; i++)
    {
        const std::string a = argv[i];
        if (a == "--out" && i + 1 < argc)
            outOverride = argv[++i];
        else if (cfgPath.empty() && a[0] != '-')
            cfgPath = a;
        else
        {
            std::cerr << "unknown argument " << a << "\n";
            return 2;
        }
    }
    if (cfgPath.empty())
    {
        std::cerr << "usage: gcopter_planner CONFIG.yaml [--out DIR]\n";
        return 2;
    }

    // Relative paths in the config are relative to the config file's directory.
    const size_t slash = cfgPath.find_last_of("/\\");
    const std::string cfgDir = slash == std::string::npos ? "." : cfgPath.substr(0, slash);
    auto resolve = [&](const std::string &p)
    { return (!p.empty() && (p[0] == '/' || p.find(':') != std::string::npos)) ? p : cfgDir + "/" + p; };

    loadConfig(cfgPath);

    const std::string outDir = outOverride.empty() ? resolve(str("OutDir")) : (cfgUsed.insert("OutDir"), outOverride);
    const std::string pathBin = outDir + "/path.bin", pathCsv = outDir + "/path.csv",
                      routeTxt = outDir + "/route.txt";
    // Remove the previous result first so a failed run can never leave a stale path behind.
    std::remove(pathBin.c_str());
    std::remove(pathCsv.c_str());
    std::remove(routeTxt.c_str());
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);

    const std::string mapFile = resolve(str("MapFile"));
    const std::vector<double> mapBound = nums("MapBound", 6);
    const double voxelWidth = num("VoxelWidth");
    const double dilateRadius = num("DilateRadius");

    const Vector3d startPos = vec3("StartPos");
    const double startHeading = num("StartHeadingDeg") * M_PI / 180.0;
    const double hoverHeight = num("HoverHeight");
    const Vector3d goal = vec3("Goal");
    std::vector<Box> cps;
    for (const auto &v : numsAll("Checkpoint", 6))
    {
        cps.push_back({Vector3d(v[0], v[1], v[2]), Vector3d(v[3], v[4], v[5])});
        if (cps.back().h.minCoeff() <= 0.0)
            fail("Checkpoint half-extents must be positive");
    }

    const double timeoutRRT = num("TimeoutRRT");
    const double corridorProgress = num("CorridorProgress");
    const double corridorRange = num("CorridorRange");
    const double maxVelMag = num("MaxVelMag");
    const double maxBdrMag = num("MaxBdrMag");
    const double maxTiltAngle = num("MaxTiltAngle");
    const double minThrust = num("MinThrust");
    const double maxThrust = num("MaxThrust");
    const double vehicleMass = num("VehicleMass");
    const double gravAcc = num("GravAcc");
    const double horizDrag = num("HorizDrag");
    const double vertDrag = num("VertDrag");
    const double parasDrag = num("ParasDrag");
    const double speedEps = num("SpeedEps");
    const double weightT = num("WeightT");
    const std::vector<double> chiVec = nums("ChiVec", 5);
    const double smoothingEps = num("SmoothingEps");
    const int integralIntervs = (int)num("IntegralIntervs");
    const double relCostTol = num("RelCostTol");

    const double sampleDt = num("SampleDt");
    const double yawMinSpeed = num("YawMinSpeed");
    const double yawRateMax = num("YawRateMaxDeg") * M_PI / 180.0;
    const double yawSmoothTime = num("YawSmoothTime");

    for (const auto &kv : cfg)
        if (!cfgUsed.count(kv.first))
            fail("unknown config key '" + kv.first + "'");
    if (sampleDt <= 0.0 || yawRateMax <= 0.0 || yawSmoothTime < 0.0)
        fail("SampleDt and YawRateMaxDeg must be > 0, YawSmoothTime >= 0");

    // --- Map (upstream:100-106 constructor, upstream:121-137 mapCallBack) ---
    const Eigen::Vector3i xyz((mapBound[1] - mapBound[0]) / voxelWidth,
                              (mapBound[3] - mapBound[2]) / voxelWidth,
                              (mapBound[5] - mapBound[4]) / voxelWidth);
    const Vector3d offset(mapBound[0], mapBound[2], mapBound[4]);
    voxel_map::VoxelMap voxelMap(xyz, offset, voxelWidth);

    const std::vector<float> fdata = loadMapPoints(mapFile);
    for (size_t cur = 0; cur < fdata.size(); cur += 3)
    {
        if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
            std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
            std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
        {
            continue;
        }
        voxelMap.setOccupied(Vector3d(fdata[cur + 0], fdata[cur + 1], fdata[cur + 2]));
    }
    voxelMap.dilate(std::ceil(dilateRadius / voxelMap.getScale()));

    // --- Waypoints: hover above the pad, checkpoint centers, goal ---
    std::vector<Vector3d> wps{startPos + Vector3d(0.0, 0.0, hoverHeight)};
    std::vector<std::string> names{"start hover point"};
    for (size_t k = 0; k < cps.size(); k++)
    {
        wps.push_back(cps[k].c);
        names.push_back("checkpoint " + std::to_string(k + 1) + " center");
    }
    wps.push_back(goal);
    names.push_back("goal");
    for (size_t i = 0; i < wps.size(); i++)
        if (voxelMap.query(wps[i]) != 0) // as upstream:244 checks the goal
            fail(names[i] + " " + fmt(wps[i]) +
                 " is inside an obstacle, its DilateRadius margin, or outside MapBound");

    // --- Front end: upstream:146-154, once per leg ---
    const auto tFront = std::chrono::steady_clock::now();
    std::vector<std::vector<Vector3d>> legs;
    std::vector<Vector3d> route; // all legs, joint points not repeated
    for (size_t i = 0; i + 1 < wps.size(); i++)
    {
        std::vector<Vector3d> leg;
        const double cost = sfc_gen::planPath<voxel_map::VoxelMap>(wps[i],
                                                                    wps[i + 1],
                                                                    voxelMap.getOrigin(),
                                                                    voxelMap.getCorner(),
                                                                    &voxelMap, timeoutRRT,
                                                                    leg);
        // OMPL may return an approximate solution; we need one that reaches the target.
        if (std::isinf(cost) || leg.size() < 2 || (leg.back() - wps[i + 1]).norm() > 1.0e-6)
            fail("no collision-free route from " + names[i] + " to " + names[i + 1] +
                 " within TimeoutRRT");
        route.insert(route.end(), leg.begin() + (route.empty() ? 0 : 1), leg.end());
        legs.push_back(leg);
    }
    const double frontMs = msSince(tFront);
    {
        // The RRT route (map frame), for debugging and the parity test. Not a path
        // file: it is written even if the trajectory later fails validation.
        std::ofstream rt(routeTxt);
        char line[128];
        for (const Vector3d &p : route)
        {
            std::snprintf(line, sizeof(line), "%.17g %.17g %.17g\n", p(0), p(1), p(2));
            rt << line;
        }
        if (!rt)
            fail("cannot write " + routeTxt);
    }

    // --- Corridor: upstream:155-165 once per leg, legs joined by checkpoint boxes ---
    // Per leg, so shortCut cannot drop the polytopes of a detour to a checkpoint.
    // shortCut keeps each leg's first and last polytope, which contain the leg's
    // endpoints, so box k normally overlaps the end of leg k and the start of leg k+1:
    //   [leg 1 polytopes] box 1 [leg 2 polytopes] box 2 ... [last leg polytopes]
    const auto tCorr = std::chrono::steady_clock::now();
    std::vector<Eigen::MatrixX4d> hPolys;
    std::vector<Vector3d> pc;
    voxelMap.getSurf(pc);
    for (size_t i = 0; i < legs.size(); i++)
    {
        std::vector<Eigen::MatrixX4d> legPolys;
        sfc_gen::convexCover(legs[i],
                             pc,
                             voxelMap.getOrigin(),
                             voxelMap.getCorner(),
                             corridorProgress,
                             corridorRange,
                             legPolys);
        sfc_gen::shortCut(legPolys);
        if (i > 0)
        {
            const Eigen::MatrixX4d B = boxPolytope(cps[i - 1]);
            if (!polysOverlap(hPolys.back(), B) || !polysOverlap(B, legPolys.front()))
                fail("checkpoint " + std::to_string(i) + " at " + fmt(cps[i - 1].c) +
                     ": box does not overlap the safe corridor on both sides. "
                     "Move it into free space or enlarge its half-extents.");
            hPolys.push_back(B);
        }
        hPolys.insert(hPolys.end(), legPolys.begin(), legPolys.end());
    }
    const double corrMs = msSince(tCorr);

    // --- Optimization: upstream:167-221 ---
    const auto tOpt = std::chrono::steady_clock::now();
    if (route.size() <= 1)
        fail("route has fewer than two points");
    Eigen::Matrix3d iniState;
    Eigen::Matrix3d finState;
    iniState << route.front(), Vector3d::Zero(), Vector3d::Zero();
    finState << route.back(), Vector3d::Zero(), Vector3d::Zero();

    gcopter::GCOPTER_PolytopeSFC gcopter;

    // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
    // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
    // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
    //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
    Eigen::VectorXd magnitudeBounds(5);
    Eigen::VectorXd penaltyWeights(5);
    Eigen::VectorXd physicalParams(6);
    magnitudeBounds << maxVelMag, maxBdrMag, maxTiltAngle, minThrust, maxThrust;
    penaltyWeights << chiVec[0], chiVec[1], chiVec[2], chiVec[3], chiVec[4];
    physicalParams << vehicleMass, gravAcc, horizDrag, vertDrag, parasDrag, speedEps;
    const int quadratureRes = integralIntervs;

    Trajectory<5> traj;
    if (!gcopter.setup(weightT,
                       iniState, finState,
                       hPolys, INFINITY,
                       smoothingEps,
                       quadratureRes,
                       magnitudeBounds,
                       penaltyWeights,
                       physicalParams))
        fail("gcopter.setup failed (corridor vertex enumeration)");
    if (std::isinf(gcopter.optimize(traj, relCostTol)))
        fail("gcopter.optimize failed");
    if (traj.getPieceNum() <= 0)
        fail("optimizer returned an empty trajectory");
    const double optMs = msSince(tOpt);
    // Machine-readable solver result (parity test), printed before validation so a
    // rejected trajectory can still be compared with upstream, which has no validation.
    std::printf("RESULT duration=%.17g front_ms=%.3f corridor_ms=%.3f opt_ms=%.3f\n",
                traj.getTotalDuration(), frontMs, corrMs, optMs);
    std::fflush(stdout);

    // --- Validation: corridor and limits are only soft penalties, so check the result ---
    // Dense 1 ms samples: never in an occupied (undilated) voxel or outside
    // MapBound, inside every checkpoint box in order, and within the
    // magnitudeBounds (flatness map as upstream process(), yaw excluded) plus
    // kLimitTol for the normal soft-penalty overshoot. Comparisons are written
    // as !(x <= limit) so that NaN fails too.
    const double kLimitTol = 0.05;
    const double T = traj.getTotalDuration();
    if (!(T > 0.0) || !std::isfinite(T))
        fail("optimizer returned a trajectory with duration " + std::to_string(T));
    const Vector3d lo = voxelMap.getOrigin(), hi = voxelMap.getCorner();
    const Eigen::Vector3i size = voxelMap.getSize();
    const std::vector<uint8_t> &vox = voxelMap.getVoxels();
    flatness::FlatnessMap flatmap;
    flatmap.reset(vehicleMass, gravAcc, horizDrag, vertDrag, parasDrag, speedEps);
    double maxSpeed = 0.0, maxTilt = 0.0, maxBdr = 0.0, thrMin = INFINITY, thrMax = -INFINITY;
    std::vector<double> cpTime(cps.size(), NAN);
    size_t nextCp = 0;
    long nMargin = 0;
    const long nCheck = (long)std::ceil(T / 1.0e-3);
    for (long i = 0; i <= nCheck; i++)
    {
        const double t = std::min(i * 1.0e-3, T);
        const Vector3d p = traj.getPos(t);
        double thr;
        Eigen::Vector4d quat;
        Vector3d omg;
        flatmap.forward(traj.getVel(t), traj.getAcc(t), traj.getJer(t), 0.0, 0.0, thr, quat, omg);
        const double speed = traj.getVel(t).norm();
        const double tilt = std::acos(std::min(1.0, std::max(-1.0, 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)))));
        const std::string at = " at t=" + std::to_string(t);
        if (!(speed <= maxVelMag * (1.0 + kLimitTol)))
            fail("speed " + std::to_string(speed) + " m/s exceeds MaxVelMag" + at);
        if (!(tilt <= maxTiltAngle * (1.0 + kLimitTol)))
            fail("tilt " + std::to_string(tilt) + " rad exceeds MaxTiltAngle" + at);
        if (!(thr >= minThrust * (1.0 - kLimitTol) && thr <= maxThrust * (1.0 + kLimitTol)))
            fail("thrust " + std::to_string(thr) + " outside MinThrust..MaxThrust" + at);
        if (!(omg.norm() <= maxBdrMag * (1.0 + kLimitTol)))
            fail("body rate " + std::to_string(omg.norm()) + " rad/s exceeds MaxBdrMag" + at);
        maxSpeed = std::max(maxSpeed, speed);
        maxTilt = std::max(maxTilt, tilt);
        maxBdr = std::max(maxBdr, omg.norm());
        thrMin = std::min(thrMin, thr);
        thrMax = std::max(thrMax, thr);
        if ((p - lo).minCoeff() < 0.0 || (hi - p).minCoeff() <= 0.0)
            fail("trajectory leaves MapBound at t=" + std::to_string(t) + " " + fmt(p));
        const Eigen::Vector3i id = voxelMap.posD2I(p);
        const uint8_t v = vox[id(0) + size(0) * (id(1) + size(1) * id(2))];
        if (v == voxel_map::Occupied)
            fail("trajectory enters an occupied voxel at t=" + std::to_string(t) + " " + fmt(p));
        nMargin += (v == voxel_map::Dilated);
        if (nextCp < cps.size() && cps[nextCp].contains(p))
            cpTime[nextCp++] = t;
    }
    if (nextCp < cps.size())
        fail("trajectory misses checkpoint " + std::to_string(nextCp + 1) + " box at " +
             fmt(cps[nextCp].c) + " (or reaches it out of order)");

    // --- Sample at SampleDt ---
    std::vector<double> ts;
    for (long i = 0; i * sampleDt < T - 1.0e-9; i++)
        ts.push_back(i * sampleDt);
    ts.push_back(T);
    const size_t N = ts.size();
    std::vector<Vector3d> P(N), V(N), A(N);
    for (size_t i = 0; i < N; i++)
    {
        P[i] = traj.getPos(ts[i]);
        V[i] = traj.getVel(ts[i]);
        A[i] = traj.getAcc(ts[i]);
    }

    // --- Yaw (map frame, CCW from +x, unwrapped): heading of horizontal
    // velocity, held when slow, rate-limited, then a centered moving average.
    std::vector<double> yawLim(N);
    double raw = startHeading, lim = startHeading;
    for (size_t i = 0; i < N; i++)
    {
        if (std::hypot(V[i](0), V[i](1)) > yawMinSpeed)
            raw += wrapPi(std::atan2(V[i](1), V[i](0)) - raw);
        const double step = yawRateMax * (i > 0 ? ts[i] - ts[i - 1] : 0.0);
        lim += std::max(-step, std::min(step, raw - lim));
        yawLim[i] = lim;
    }
    std::vector<double> cum(N + 1, 0.0), yaw(N), yawRate(N, 0.0);
    for (size_t i = 0; i < N; i++)
        cum[i + 1] = cum[i] + yawLim[i];
    const long H = (long)std::lround(0.5 * yawSmoothTime / sampleDt);
    for (long i = 0; i < (long)N; i++)
    {
        const long h = std::min({H, i, (long)N - 1 - i}); // symmetric, shrinks at the ends
        yaw[i] = (cum[i + h + 1] - cum[i - h]) / (2 * h + 1);
    }
    for (size_t i = 0; i < N && N > 1; i++)
    {
        const size_t a = i > 0 ? i - 1 : 0, b = i + 1 < N ? i + 1 : N - 1;
        yawRate[i] = (yaw[b] - yaw[a]) / (ts[b] - ts[a]);
    }

    // --- Map -> PX4 local NED, origin at the pad (StartPos), +x along StartHeading ---
    const double ch = std::cos(startHeading), sh = std::sin(startHeading);
    auto toNed = [&](const Vector3d &d)
    { return Vector3d(ch * d(0) + sh * d(1), sh * d(0) - ch * d(1), -d(2)); };

    std::ofstream bin(pathBin + ".tmp", std::ios::binary), csv(pathCsv + ".tmp");
    if (!bin || !csv)
        fail("cannot write to " + outDir);
    const uint32_t version = 1, ncols = 12;
    const uint64_t nrows = N;
    bin.write("DHPATH\0\0", 8);
    bin.write((const char *)&version, 4);
    bin.write((const char *)&ncols, 4);
    bin.write((const char *)&nrows, 8);
    bin.write((const char *)&sampleDt, 8);
    csv << "t,x,y,z,vx,vy,vz,ax,ay,az,yaw,yaw_rate\n";
    double maxYawRate = 0.0;
    for (size_t i = 0; i < N; i++)
    {
        const Vector3d p = toNed(P[i] - startPos), v = toNed(V[i]), a = toNed(A[i]);
        const double row[12] = {ts[i], p(0), p(1), p(2), v(0), v(1), v(2), a(0), a(1), a(2),
                                wrapPi(startHeading - yaw[i]), -yawRate[i]};
        bin.write((const char *)row, sizeof(row));
        char line[512];
        std::snprintf(line, sizeof(line), "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                      row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8], row[9],
                      row[10], row[11]);
        csv << line;
        maxYawRate = std::max(maxYawRate, std::abs(yawRate[i]));
    }
    bin.close();
    csv.close();
    if (!bin || !csv ||
        std::rename((pathBin + ".tmp").c_str(), pathBin.c_str()) != 0 ||
        std::rename((pathCsv + ".tmp").c_str(), pathCsv.c_str()) != 0)
        fail("error writing output files in " + outDir);

    // --- Summary ---
    std::printf("GCOPTER plan OK\n");
    std::printf("  route            : %zu points, %zu legs\n", route.size(), wps.size() - 1);
    std::printf("  corridor         : %zu polytopes (incl. %zu checkpoint boxes)\n",
                hPolys.size(), cps.size());
    std::printf("  total time       : %.3f s (%d pieces)\n", T, traj.getPieceNum());
    std::printf("  max speed        : %.2f m/s (MaxVelMag %.2f)\n", maxSpeed, maxVelMag);
    std::printf("  max tilt         : %.1f deg (MaxTiltAngle %.1f deg)\n",
                maxTilt * 180.0 / M_PI, maxTiltAngle * 180.0 / M_PI);
    std::printf("  thrust range     : %.2f .. %.2f (MinThrust %.2f, MaxThrust %.2f)\n",
                thrMin, thrMax, minThrust, maxThrust);
    std::printf("  max body rate    : %.2f rad/s (MaxBdrMag %.2f, yaw excluded)\n", maxBdr, maxBdrMag);
    std::printf("  max yaw rate     : %.1f deg/s (YawRateMaxDeg %.1f)\n",
                maxYawRate * 180.0 / M_PI, yawRateMax * 180.0 / M_PI);
    for (size_t k = 0; k < cps.size(); k++)
        std::printf("  checkpoint %-2zu    : reached at t = %.3f s, box %s +/- %s\n",
                    k + 1, cpTime[k], fmt(cps[k].c).c_str(), fmt(cps[k].h).c_str());
    std::printf("  dilation margin  : %ld of %ld 1-ms samples inside DilateRadius (no occupied voxel hit)\n",
                nMargin, nCheck + 1);
    std::printf("  solve time       : %.1f ms (front-end %.1f, corridor %.1f, optimizer %.1f)\n",
                frontMs + corrMs + optMs, frontMs, corrMs, optMs);
    std::printf("  wrote            : %s (%zu rows), %s, %s\n",
                pathBin.c_str(), N, pathCsv.c_str(), routeTxt.c_str());
    return 0;
}
