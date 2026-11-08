import math
from pathlib import Path
import struct
import tempfile
import unittest

from path_import import read_dhpath,to_course_flu,document,describe


class PathImportTests(unittest.TestCase):
    def blob(self,rows):
        return b'DHPATH\0\0'+struct.pack('<IIQd',1,12,len(rows),.01)+b''.join(struct.pack('<12d',*r) for r in rows)

    def test_transform_keeps_time_and_consistently_rotates_all_derivatives(self):
        rows=[[0,1,2,-3,4,5,-6,7,8,-9,.4,.2],[2,2,3,-4,0,0,0,0,0,0,.5,0]]
        flu=to_course_flu(rows)
        self.assertEqual(flu[0],[0,1,-2,3,4,-5,6,7,-8,9,-.4,-.2])
        self.assertEqual(rows[0][2],2)
        doc=document(flu,'test')
        self.assertEqual(doc['times_s'],[0,2]);self.assertEqual(doc['waypoints'][0],[1,-2,3])
        stats=describe(flu)
        self.assertEqual(stats['minimum_uniform_time_stretch'],20.)
        self.assertFalse(stats['timing_applied']);self.assertFalse(stats['execution_supported'])

    def test_exact_length_finite_monotonic_header_validation(self):
        rows=[[0,0,0,-1,0,0,0,0,0,0,0,0],[.01,1,0,-1,0,0,0,0,0,0,0,0]]
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'path.bin';raw=self.blob(rows);path.write_bytes(raw)
            parsed,meta=read_dhpath(path)
            self.assertEqual(len(parsed),2);self.assertEqual(meta['rows'],2)
            invalid=[raw[:-1],raw+b'\0',b'BADPATH!'+raw[8:]]
            for field,value in ((0,0),(1,math.nan)):
                bad=[r[:] for r in rows];bad[1][field]=value;invalid.append(self.blob(bad))
            for blob in invalid:
                path.write_bytes(blob)
                with self.assertRaises(ValueError):read_dhpath(path)


if __name__=='__main__':unittest.main()
