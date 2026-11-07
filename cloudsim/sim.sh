#!/usr/bin/env bash
# Start/stop the simulator on the cloud sim VM. Everything runs in the tmux session "sim":
#   px4    PX4 SITL v1.16.2 + Gazebo Harmonic (headless), stock x500 in WORLD (default: empty)
#   agent  Micro XRCE-DDS Agent, UDP 8888 (PX4 <-> ROS 2 /fmu/*)
#   clock  ros_gz_bridge Gazebo /clock -> ROS /clock (for use_sim_time)
#
#   bash ~/Dronehacks/cloudsim/sim.sh start | stop | status | attach
# "stop" kills the whole world, so the next "start" is a fresh world (our Reset world).
# NAV_DLL_ACT=0: there is no ground station on this headless VM, and PX4's SITL default (2, RTL on
# data-link loss) refuses to arm without one. Sim-only; check the drone's own value separately.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
WORLD=${WORLD:-default}
MODEL=${MODEL:-gz_x500}

case "${1:-}" in
start)
    if tmux has-session -t sim 2>/dev/null; then echo "already running (sim.sh stop first)"; exit 1; fi
    tmux new-session -d -s sim -n px4 \
        "cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=$WORLD PX4_PARAM_NAV_DLL_ACT=0 make px4_sitl $MODEL; bash"
    tmux new-window -t sim -n agent "MicroXRCEAgent udp4 -p 8888; bash"
    tmux new-window -t sim -n clock \
        "source $HERE/env.sh && ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock; bash"
    echo "started; 'sim.sh status' in ~30 s, 'sim.sh attach' to watch (Ctrl-b d to detach)"
    ;;
stop)
    tmux kill-session -t sim 2>/dev/null || true
    pkill -f 'px4_sitl_default/bin/px4' || true
    pkill -f 'gz sim' || true
    pkill -f MicroXRCEAgent || true
    echo "stopped"
    ;;
status)
    tmux has-session -t sim 2>/dev/null && echo "tmux session: running" || echo "tmux session: not running"
    for p in 'bin/px4' 'gz sim' MicroXRCEAgent parameter_bridge; do
        pgrep -f "$p" >/dev/null && echo "  up    $p" || echo "  DOWN  $p"
    done
    set +u; source "$HERE/env.sh"; set -u
    echo "PX4 topics: $(ros2 topic list 2>/dev/null | grep -c '^/fmu/') under /fmu"
    timeout 5 ros2 topic echo --once /clock 2>/dev/null | grep -A2 clock: | tr -s ' \n' ' ' || echo "no /clock"
    echo
    ;;
attach)
    tmux attach -t sim
    ;;
*)
    echo "usage: sim.sh start|stop|status|attach   (env: WORLD=default MODEL=gz_x500)"; exit 2
    ;;
esac
