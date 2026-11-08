"""Single USB owner with bounded queues and QGC/loopback diagnostic forwarding."""
from collections import Counter, deque
import glob
import json
import os
from pathlib import Path
import select
import socket
import stat
import threading
import time

import serial
from pymavlink.dialects.v20 import common as mav
from telemetry_mailbox import TelemetryMailbox


def split_packets(data):
    packets, i = [], 0
    while i < len(data):
        if data[i] == 0xfd:
            if len(data)-i < 3:
                break
            size = data[i+1] + 12 + (13 if data[i+2] & 1 else 0)
        elif data[i] == 0xfe:
            if len(data)-i < 2:
                break
            size = data[i+1] + 8
        else:
            i += 1
            continue
        if len(data)-i < size:
            break
        packets.append(bytes(data[i:i+size]))
        i += size
    return packets, data[i:]


def packet_id(packet):
    return int.from_bytes(packet[7:10], 'little') if packet[0] == 0xfd else packet[5]


def owners(device):
    target = os.stat(device).st_rdev
    found = set()
    for fd in glob.glob('/proc/[0-9]*/fd/*'):
        try:
            info = os.stat(fd)
            if stat.S_ISCHR(info.st_mode) and info.st_rdev == target:
                found.add(int(fd.split('/')[2]))
        except (OSError, ValueError):
            pass
    return sorted(found)


class LatestQueue:
    """Replace unsent streaming samples; bound operator traffic separately."""
    def __init__(self):
        self.latest = {}
        self.operator = deque(maxlen=128)
        self.lock = threading.Lock()

    def put(self, kind, message, ttl, metadata=None):
        item = (kind, message, time.monotonic()+ttl, metadata or {})
        with self.lock:
            if kind in ('vision', 'setpoint', 'timesync', 'heartbeat'):
                self.latest[kind] = item
            else:
                if len(self.operator) == self.operator.maxlen:
                    return False
                self.operator.append(item)
        return True

    def take(self, counts):
        with self.lock:
            # Time sync and control first; at most one sample per stream.
            for kind in ('timesync', 'setpoint', 'vision', 'heartbeat'):
                item = self.latest.pop(kind, None)
                if item is not None:
                    if time.monotonic() <= item[2]:
                        return item
                    counts['expired_'+kind] += 1
            while self.operator:
                item = self.operator.popleft()
                if time.monotonic() <= item[2]:
                    return item
                counts['expired_operator'] += 1
        return None

    def clear(self):
        with self.lock:
            self.latest.clear()
            self.operator.clear()


class SerialRouter(threading.Thread):
    def __init__(self, device, folder, *, qgc_bind=('0.0.0.0',14550),
                 qgc_destination=('100.107.69.115',14550),
                 diag_bind=('127.0.0.1',14612), diag_destination=('127.0.0.1',14613),
                 write_bytes=os.write):
        super().__init__(daemon=True)
        self.device, self.folder = device, Path(folder)
        self.queue = LatestQueue()
        self.telemetry = TelemetryMailbox()
        self.stop_event = threading.Event()
        self.counts = Counter()
        self.fc_heartbeat_at = 0.0
        self.fc_armed = None
        self.fc_landed = None
        self.fc_landed_at = 0.0
        self.connected = False
        self.error = None
        self.log_error = None
        self.write_bytes = write_bytes
        self.encoder = mav.MAVLink(None, srcSystem=12, srcComponent=197)
        self.decoder = mav.MAVLink(None)
        self.decoder.robust_parsing = True
        self.qgc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # USB delivery must start even before Tailscale has its address.
        # outbound_udp still accepts QGC input only from qgc_destination[0].
        self.qgc.bind(qgc_bind)
        self.diag = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.diag.bind(diag_bind)
        self.qgc.setblocking(False)
        self.diag.setblocking(False)
        self.qgc_destination = qgc_destination
        self.diag_destination = diag_destination
        self.sent_log = (self.folder/'sent.jsonl').open('a', buffering=65536)

    def snapshot(self):
        return dict(connected=self.connected, error=self.error, log_error=self.log_error,
                    fc_heartbeat_age_s=round(time.monotonic()-self.fc_heartbeat_at,3),
                    fc_armed=self.fc_armed, fc_landed=self.fc_landed,
                    fc_landed_age_s=round(time.monotonic()-self.fc_landed_at,3),
                    telemetry_age_s=self.telemetry.ages(), counts=dict(self.counts))

    def incoming(self, packet):
        msgid = packet_id(packet)
        # This USB link uses the Jetson clock for vision. Do not mix QGC's
        # timesync replies into the same PX4 MAVLink-instance clock estimator.
        if msgid != mav.MAVLINK_MSG_ID_TIMESYNC:
            for sock, destination in ((self.qgc,self.qgc_destination),(self.diag,self.diag_destination)):
                try:
                    sock.sendto(packet,destination)
                except OSError:
                    self.counts['udp_send_errors'] += 1
        try:
            msg = self.decoder.parse_char(packet)
        except Exception:
            self.counts['decode_errors'] += 1
            return
        if msg is None or msg.get_srcSystem() != 12 or msg.get_srcComponent() != 1:
            return
        self.telemetry.put(msg)
        if msgid == mav.MAVLINK_MSG_ID_HEARTBEAT:
            self.fc_heartbeat_at = time.monotonic()
            self.fc_armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
        elif msgid == mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
            self.fc_landed_at = time.monotonic()
            self.fc_landed = (None if msg.landed_state == mav.MAV_LANDED_STATE_UNDEFINED
                              else msg.landed_state == mav.MAV_LANDED_STATE_ON_GROUND)
        elif msgid == mav.MAVLINK_MSG_ID_TIMESYNC and msg.tc1 == 0:
            self.queue.put('timesync',mav.MAVLink_timesync_message(time.time_ns(),msg.ts1),.05)
            self.counts['timesync_requests'] += 1

    def outbound_udp(self, sock):
        data, source = sock.recvfrom(8192)
        allowed = source[0] == (self.qgc_destination[0] if sock is self.qgc else '127.0.0.1')
        if not allowed:
            self.counts['udp_rejected_source'] += 1
            return
        packets, rest = split_packets(data)
        if rest:
            self.counts['udp_partial_packet'] += 1
            return
        for packet in packets:
            # One source owns automated vision and Offboard setpoints.
            if packet_id(packet) in (111, 331, 102, 104, 138, 84, 86, 82):
                self.counts['udp_owned_message_filtered'] += 1
                continue
            if not self.queue.put('operator',packet,.5):
                self.counts['operator_queue_full'] += 1

    def run(self):
        port = None
        pending = b''
        item = None
        buf = bytearray()
        opened_at = next_open = last_heartbeat = last_flush = 0.0
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                if port is None:
                    if now < next_open:
                        self.stop_event.wait(.05)
                        continue
                    try:
                        held = owners(self.device)
                        if held:
                            raise RuntimeError('USB port already held by PIDs '+repr(held))
                        port = serial.Serial(self.device,115200,timeout=0,write_timeout=0,exclusive=True)
                        self.queue.clear()
                        self.telemetry.clear()
                        self.fc_heartbeat_at = 0.0
                        self.fc_landed_at = 0.0
                        self.fc_armed = self.fc_landed = None
                        self.connected, self.error = True, None
                        self.counts['opens'] += 1
                        buf.clear()
                        opened_at = now
                    except Exception as exc:
                        self.error = str(exc)
                        next_open = now+1
                        continue
                try:
                    if now-last_heartbeat >= 1:
                        self.queue.put('heartbeat',mav.MAVLink_heartbeat_message(
                            mav.MAV_TYPE_ONBOARD_CONTROLLER,mav.MAV_AUTOPILOT_INVALID,
                            0,0,mav.MAV_STATE_ACTIVE,3),.5)
                        last_heartbeat = now
                    if not pending:
                        item = self.queue.take(self.counts)
                        if item is not None:
                            kind,message,deadline,metadata = item
                            if kind in ('vision','setpoint') and now-self.fc_heartbeat_at > 2:
                                self.counts['no_fc_'+kind] += 1
                                item = None
                            else:
                                pending = message if isinstance(message,bytes) else message.pack(self.encoder)
                                pending_length = len(pending)
                                if not isinstance(message,bytes):
                                    self.encoder.seq = (self.encoder.seq+1)&255
                                pending_started = now
                    ready, writable, _ = select.select([port.fileno(),self.qgc,self.diag],
                                                       [port.fileno()] if pending else [],[],.005)
                    if port.fileno() in ready:
                        data = os.read(port.fileno(),16384)
                        if not data:
                            raise OSError('USB disconnected')
                        self.counts['rx_bytes'] += len(data)
                        buf.extend(data)
                        packets,remaining = split_packets(buf)
                        buf = bytearray(remaining)
                        for packet in packets:
                            self.incoming(packet)
                    for sock in (self.qgc,self.diag):
                        if sock in ready:
                            self.outbound_udp(sock)
                    if pending and time.monotonic() > item[2]:
                        if len(pending) != pending_length:
                            raise OSError('Partially written message expired; reset USB connection')
                        self.counts['expired_'+item[0]] += 1
                        pending, item = b'', None
                    if writable and pending:
                        try:
                            written = self.write_bytes(port.fileno(),pending)
                        except BlockingIOError:
                            written = 0
                        self.counts['tx_bytes'] += written
                        pending = pending[written:]
                        if not pending:
                            kind,_,_,metadata = item
                            self.counts['sent_'+kind] += 1
                            if kind in ('vision','setpoint'):
                                try:
                                    self.sent_log.write(json.dumps(dict(kind=kind,wall_ns=time.time_ns(),**metadata))+'\n')
                                except (OSError, TypeError, ValueError) as exc:
                                    self.counts['log_errors'] += 1
                                    self.log_error = str(exc)
                            item = None
                    if pending and time.monotonic()-pending_started > .1:
                        raise OSError('USB write stalled >100 ms; dropping connection and queued samples')
                    if now-last_flush > 1:
                        try:
                            self.sent_log.flush()
                        except OSError as exc:
                            self.counts['log_errors'] += 1
                            self.log_error = str(exc)
                        last_flush = now
                except (OSError,serial.SerialException) as exc:
                    self.error, self.connected = str(exc), False
                    self.counts['disconnects'] += 1
                    port.close()
                    port = None
                    pending, item = b'', None
                    self.queue.clear()
                    self.telemetry.clear()
                    next_open = time.monotonic()+1
        except BaseException as exc:
            self.error = repr(exc)
            self.connected = False
        finally:
            if port is not None:
                port.close()
            self.qgc.close()
            self.diag.close()
            self.sent_log.close()

    def close(self):
        self.stop_event.set()
        self.join(timeout=3)
