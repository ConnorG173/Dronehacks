# Questions for the organizers (Kelvin on Slack, kelvinkn@stanford.edu)

Collected overnight by the workstream sessions. Most important first.

1. **Tailscale on drone 12.** The board was moved from the organizer tailnet to a personal one on Nov 6 before
   we read the README rule. It needs an organizer to approve re-adding `tag:drone` (or an auth key):
   `sudo tailscale up --reset --hostname=skybeta-12 --ssh --advertise-tags=tag:drone --authkey=...`
2. **Return bonus (rule 7.10).** After landing and disarming on the pad, the drone must take off "by itself", but the
   flight interface has no arm path and rule 5.2 allows only a start command and aborts. Is an autonomous re-arm
   from the Jetson (`/fmu/in/vehicle_command`) allowed for the return? If not, is the return possible at all?
   (A touch-and-go without disarming would lose checkpoint 5, so we will not do that.)
3. **Checkpoint geometry.** `planner/config/course_v3.yaml`'s checkpoint list was read off the CAD. Do the official
   checkpoint lines (corridor end, Gate A, tunnel exit, Gate B / window, pad ring) match the published v3 model?
4. **Depth.** The drone publishes no depth topic today. May we enable on-camera stereo depth in the OAK-D pipeline,
   or should teams compute depth on the Jetson?
5. **Lights.** Is a small light on the drone (for the dim tunnel) allowed, or does rule 3.4 forbid anything that
   draws power except declared sensors?
6. **Organizer sim bug.** `docker/Dockerfile.sim` COLCON_IGNOREs `px4-ros2-interface-lib` (its `package.xml` is one
   level down), so `competitor_advanced_cpp` fails to build. Our patched copy is in `sim/gcp/setup_vm.sh`.
