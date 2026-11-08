#!/usr/bin/env python3
"""Disarmed-only PX4 console through the adapter's local diagnostic socket."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0,'/root/team_ws/claude/gap_trace_20261107')
from usb_console import Console


class UDPConsole(Console):
    def __init__(self):
        # Start with v2 message classes instead of changing dialect versions
        # mid-stream, which breaks this pymavlink build's instance cache.
        os.environ['MAVLINK20']='1'
        from pymavlink import mavutil
        self.mavutil=mavutil
        self.link=mavutil.mavlink_connection('udpin:127.0.0.1:14613',source_system=254,source_component=190)
        self.heartbeat=None
        self.heartbeat_time=0
        self.text=''
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and self.heartbeat is None:
            self.pump(.2)
        self.assert_disarmed()
        self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
        self.send('\n')
        self.pump(.5)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--commands',required=True)
    parser.add_argument('--out',required=True)
    args=parser.parse_args()
    console=UDPConsole()
    rows=[]
    try:
        for command in json.loads(Path(args.commands).read_text()):
            try:
                result=console.command(command)
            except Exception as exc:
                rows.append(dict(wall_ns=time.time_ns(),command=command,
                                 output=console.text,error=str(exc)))
                raise
            rows.append(dict(wall_ns=time.time_ns(),command=command,output=result))
            print(json.dumps(rows[-1]),flush=True)
    finally:
        console.close()
        Path(args.out).write_text(json.dumps(rows,indent=2)+'\n')


if __name__=='__main__':main()
