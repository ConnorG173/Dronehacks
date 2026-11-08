"""Offline freshness and single-owner return-path checks."""
from pathlib import Path
import socket
import tempfile
from types import SimpleNamespace
import unittest

from pymavlink.dialects.v20 import common as mav
from serial_router import SerialRouter
from telemetry_mailbox import TelemetryMailbox


def packet(kind='ODOMETRY', stamp=100):
    return SimpleNamespace(get_type=lambda:kind, time_usec=stamp)


class MailboxTests(unittest.TestCase):
    def test_latest_only_and_no_timer_replay(self):
        box=TelemetryMailbox()
        for i in range(50):
            self.assertTrue(box.put(packet(stamp=i),now=10+i*.001,wall_us=i))
        items=box.take(now=10.1)
        self.assertEqual(len(items),1)
        self.assertEqual(items[0][0].time_usec,49)
        self.assertEqual(box.take(now=10.2),[])

    def test_delayed_and_repeated_data_cannot_refresh_position(self):
        box=TelemetryMailbox()
        self.assertTrue(box.put(packet(),now=10))
        self.assertFalse(box.put(packet(),now=11))
        self.assertFalse(box.put(packet(stamp=99),now=11))
        self.assertEqual(box.take(now=11),[])
        self.assertTrue(box.put(packet(stamp=101),now=11))
        self.assertEqual(len(box.take(now=11.1)),1)

    def test_reconnect_clears_old_messages_and_allows_new_boot_clock(self):
        box=TelemetryMailbox()
        box.put(packet(stamp=1000),now=10)
        box.clear()
        self.assertEqual(box.take(now=10.1),[])
        self.assertEqual(box.ages(),{})
        self.assertTrue(box.put(packet(stamp=1),now=11))

    def test_ack_queue_is_bounded_and_preserves_independent_events(self):
        box=TelemetryMailbox()
        for i in range(40):
            msg=SimpleNamespace(get_type=lambda:'COMMAND_ACK',result=i)
            box.put(msg,now=10)
        self.assertEqual([v[0].result for v in box.take(now=10.1)],list(range(8,40)))

    def test_only_fc_source_reaches_return_consumers(self):
        with tempfile.TemporaryDirectory() as directory:
            router=SerialRouter('/unused',Path(directory),qgc_bind=('127.0.0.1',0),
                                diag_bind=('127.0.0.1',0))
            heartbeat=mav.MAVLink_heartbeat_message(2,12,0,6<<16,3,3)
            try:
                for system,component in ((255,190),(12,197),(13,1)):
                    router.incoming(heartbeat.pack(mav.MAVLink(None,srcSystem=system,srcComponent=component)))
                self.assertEqual(router.telemetry.take(),[])
                router.incoming(heartbeat.pack(mav.MAVLink(None,srcSystem=12,srcComponent=1)))
                values=router.telemetry.take()
                self.assertEqual(len(values),1)
                self.assertFalse(router.fc_armed)
                self.assertEqual(values[0][0].custom_mode,6<<16)
            finally:
                router.qgc.close();router.diag.close();router.sent_log.close()


if __name__=='__main__': unittest.main()
