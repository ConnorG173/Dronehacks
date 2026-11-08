#!/usr/bin/env python3
"""Passive comparison of USB return data with the old UART DDS feed."""
import sys
sys.path.insert(0,'/root/team_ws/claude/gap_trace_20261107')
import trace_delivery

for name in ('vision','heartbeat','setpoint'):
    topic,message_type=trace_delivery.TOPICS[name]
    trace_delivery.TOPICS['dds_'+name]=(topic,message_type)
    trace_delivery.TOPICS[name]=(topic.replace('/fmu/in/','/claude/usb/in/'),message_type)
for name in ('local','attitude','status','landed'):
    topic,message_type=trace_delivery.TOPICS[name]
    trace_delivery.TOPICS['dds_'+name]=(topic,message_type)
    trace_delivery.TOPICS[name]=(topic.replace('/fmu/out/','/claude/usb/out/'),message_type)

trace_delivery.main()
