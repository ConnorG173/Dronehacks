#!/usr/bin/env python3
"""Bench and tethered tests for the real drone, run ON THE JETSON from an SSH session.

There is no RC transmitter: this program, PX4's failsafes and the people at the rig are
the only safety layer.  Read README.md (this folder) before the first run, and run the tests in order:

  props OFF    python3 bench.py link          topics, rates, PX4 state            (publishes nothing)
               python3 bench.py estimator     VIO health, EKF2 fusion, drift      (publishes nothing)
               python3 bench.py frames        hand-carry axis/sign check          (publishes nothing)
               python3 bench.py motors        spin each motor alone               (motors spin)
               python3 bench.py arm           arm / disarm / KILL / stream loss   (motors spin)
  props ON,    python3 bench.py hover         take off, hold, land
  TETHERED     python3 bench.py steps         step responses on each axis + yaw
               python3 bench.py circle        smooth circle with feedforward
               python3 bench.py path --file ../planner/out/path.bin --speed 0.3
  any time     python3 bench.py status        live state display                  (publishes nothing)
               python3 bench.py kill          FORCE DISARM now (keep it ready in a 2nd terminal)
               python3 bench.py land          ask PX4 to land

Exit code 0 = the test passed.  Run from this folder (cd bench). Logs go to bench_logs/ here.
"""
import argparse
import sys
import time

TESTS = ("link", "estimator", "frames", "motors", "arm", "hover", "steps", "circle", "path",
         "status", "kill", "land")
FLIGHT = ("hover", "steps", "circle", "path")


def parse(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="test", required=True)

    def common(p):
        p.add_argument("--health-topic", default="/estimator/healthy",
                       help="std_msgs/Bool estimator health (default %(default)s, from onboard/vio/ev_bridge.py)")
        p.add_argument("--vio-topic", default="/visual_slam/tracking/odometry", help="nav_msgs/Odometry from VIO")
        p.add_argument("--no-health", action="store_true",
                       help="do not require the estimator health topic (e.g. GPS/mocap instead of VIO)")
        p.add_argument("--log-dir", default="bench_logs")
        return p

    for name in ("link", "frames", "status", "kill", "land"):
        common(sub.add_parser(name))
    p = common(sub.add_parser("estimator"))
    p.add_argument("--duration", type=float, default=30.0, help="static measurement (s)")
    p.add_argument("--max-drift", type=float, default=0.05, help="m over the duration")
    p.add_argument("--max-noise", type=float, default=0.01, help="m standard deviation")
    p.add_argument("--min-vio-hz", type=float, default=15.0)
    p.add_argument("--cover", action="store_true", help="also do the cover-the-camera health test")
    p = common(sub.add_parser("motors"))
    p.add_argument("--count", type=int, default=4)
    p.add_argument("--value", type=float, default=0.1, help="0 = idle spin .. 1 = full; keep it low")
    p.add_argument("--seconds", type=float, default=2.0)
    common(sub.add_parser("arm"))

    for name in FLIGHT:
        p = common(sub.add_parser(name))
        p.add_argument("--height", type=float, default=0.4, help="hover height above the takeoff point (m)")
        p.add_argument("--fence", type=float, default=0.5, help="land if further than this from takeoff (m)")
        p.add_argument("--ceiling", type=float, default=1.0, help="land if higher than this (m)")
        p.add_argument("--max-tilt", type=float, default=30.0, help="land above this tilt (deg)")
        p.add_argument("--max-track-err", type=float, default=0.35, help="land on a larger tracking error (m)")
        p.add_argument("--max-flight-s", type=float, default=120.0)
        p.add_argument("--min-battery-land", type=float, default=0.20, help="land below this charge fraction")
        p.add_argument("--climb-rate", type=float, default=0.3)
        p.add_argument("--settle", type=float, default=3.0, help="hover this long before the profile (s)")
        p.add_argument("--allow-multiplexer", action="store_true",
                       help="allow running inside tmux/screen (disables the SSH-loss landing!)")
        if name == "hover":
            p.add_argument("--duration", type=float, default=10.0)
        if name == "steps":
            p.add_argument("--step-m", type=float, default=0.2, help="forward and right steps (m)")
            p.add_argument("--step-z-m", type=float, default=0.15, help="up step (m)")
            p.add_argument("--step-yaw-deg", type=float, default=30.0)
            p.add_argument("--hold", type=float, default=4.0)
            p.add_argument("--move-time", type=float, default=1.0, help="smooth-shape transition time (s)")
            p.add_argument("--shape", choices=("smooth", "step"), default="smooth")
        if name == "circle":
            p.add_argument("--radius", type=float, default=0.2)
            p.add_argument("--period", type=float, default=6.0, help="seconds per lap at full speed")
            p.add_argument("--laps", type=float, default=2.0)
            p.add_argument("--ramp", type=float, default=3.0, help="speed-up / slow-down time (s)")
        if name == "path":
            p.add_argument("--file", required=True, help="gcopter_planner path.bin or path.csv")
            p.add_argument("--speed", type=float, default=0.3, help="time scale, (0, 1]")

    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        argv, ros = argv[:i], argv[i:]
    else:
        ros = []
    return ap.parse_args(argv), ros


def main(argv=None):
    args, ros_args = parse(sys.argv[1:] if argv is None else argv)
    # build anything that can fail on the arguments before ROS starts
    if args.test in FLIGHT:
        from benchtest.procedures import make_profile
        try:
            make_profile(args)
        except (OSError, ValueError) as e:
            print(f"ERROR: {e}")
            return 2

    import rclpy
    from benchtest import procedures as P
    from benchtest.link import Px4Link
    rclpy.init(args=["bench", *ros_args])
    link = Px4Link(health_topic="" if args.no_health else args.health_topic, vio_topic=args.vio_topic)
    fn = {"link": P.link_test, "estimator": P.estimator_test, "frames": P.frames_test, "motors": P.motors_test,
          "arm": P.arm_test, "status": P.status, "kill": P.kill, "land": P.land}.get(args.test, P.fly)
    ok = False
    try:
        ok = fn(link, args)
    except KeyboardInterrupt:
        # props-off tests that spin motors: stop them now rather than waiting for PX4's failsafe
        # (flight tests handle Ctrl-C themselves and land)
        if args.test == "arm":
            print("\nCtrl-C: KILL (force disarm)")
            for _ in range(5):
                link.do("kill")
                time.sleep(0.05)
        elif args.test == "motors":
            print("\nCtrl-C: stopping motor test")
            for i in range(1, args.count + 1):
                link.actuator_test(i, float("nan"), 0.0)
                time.sleep(0.05)
    finally:
        link.close()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
