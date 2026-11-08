#!/usr/bin/env python3
"""Replace one ROS launch pane only while fresh PX4 telemetry says grounded."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

parser=argparse.ArgumentParser()
parser.add_argument('--pane',required=True,choices=['skybeta:2.0','skybeta:3.0'])
parser.add_argument('--expected',required=True)
parser.add_argument('--launch',required=True)
parser.add_argument('--ground-source',choices=['dds','usb'],default='dds',
                    help='USB ground state is usable when restarting the bridge pane only')
parser.add_argument('--launcher',choices=['/usr/local/bin/run_bridge.sh',
                                         '/usr/local/bin/run_competitor.sh'])
args=parser.parse_args()
if args.ground_source=='usb' and args.pane!='skybeta:2.0':
    raise RuntimeError('USB guard requires the USB adapter to remain running')
if args.launcher and args.launcher != {'skybeta:2.0':'/usr/local/bin/run_bridge.sh',
                                    'skybeta:3.0':'/usr/local/bin/run_competitor.sh'}[args.pane]:
    raise RuntimeError('Launcher does not match pane')
root=Path('/root/team_ws/claude/mavlink_usb')
folder=Path((root/'active_bench.txt').read_text().strip().replace('/home/dronehacks/team_ws/','/root/team_ws/'))

def ground():
    if args.ground_source=='usb':
        state=json.loads((root/'runtime-status.json').read_text())
        if (time.time_ns()-state['wall_ns'] > 2_000_000_000
                or not state['connected'] or not state['router_alive']
                or state['fc_heartbeat_age_s']>1.5 or state['fc_landed_age_s']>1):
            raise RuntimeError('Stale USB ground telemetry')
        if state['fc_armed'] is not False or state['fc_landed'] is not True:
            raise RuntimeError('PX4 must be disarmed and landed')
        return
    state=json.loads((folder/'recording.status.json').read_text())
    for key in ('status','landed'):
        if time.time_ns()-state['latest'][key]['wall_ns']>3_000_000_000:
            raise RuntimeError('Stale PX4 status')
    if state['latest']['status']['arming_state']!=1 or not state['latest']['landed']['landed']:
        raise RuntimeError('PX4 must be disarmed and landed')

ground()
pid=int(subprocess.check_output(['tmux','display-message','-p','-t',args.pane,'#{pane_pid}'],text=True))
argv=Path('/proc',str(pid),'cmdline').read_bytes().split(b'\0')
if args.expected.encode() not in argv:
    raise RuntimeError('Unexpected launch process: '+repr(argv))
os.kill(pid,signal.SIGINT)
deadline=time.monotonic()+15
while time.monotonic()<deadline:
    dead=subprocess.check_output(['tmux','display-message','-p','-t',args.pane,'#{pane_dead}'],text=True).strip()
    if dead=='1': break
    time.sleep(.2)
else:
    raise RuntimeError('Old launch did not stop; refusing to create duplicate nodes')
ground()
command='bash -c "source skybeta_env.sh; exec ros2 launch '+args.launch+'"'
if args.launcher:
    command='bash -c "exec '+args.launcher+'"'
subprocess.run(['tmux','respawn-pane','-t',args.pane,command],check=True)
with (folder/'launch-changes.jsonl').open('a') as output:
    output.write(json.dumps(dict(wall_ns=time.time_ns(),pane=args.pane,old=args.expected,
                                new=args.launch,launcher=args.launcher))+'\n')
print('Started',args.launch,'in',args.pane)
