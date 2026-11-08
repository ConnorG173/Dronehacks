"""Local synthetic HTTP tests; no flight node or real dashboard endpoint."""
import copy
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
import threading
from types import SimpleNamespace
import unittest
from urllib.parse import urlencode

from dash_node import make_handler


class LinkTests(unittest.TestCase):
    def test_persistent_connection_and_conditional_geometry_keep_metadata_fresh(self):
        base=dict(pos={'age_s':.02},depth=dict(geometry_version='depth:1:live',status='live',
            age_s=.2,points=[[1,2]],camera_xy=[0,0],view_polygon=[]),course=dict(
            original_version='course:1',adjusted_version='course:1:2:adjusted',
            original_count=2,original_course=[[0,0,1],[1,0,1]],adjusted_course=[],
            original_world_enu=[[0,0,1],[1,0,1]],adjusted_world_enu=[[0,0,1],[1,.2,1]],
            adjustment={'status':'adjusted','age_s':.2}))
        calls=[]
        def snapshot(now,client):
            calls.append(client);result=copy.deepcopy(base);result['sequence']=len(calls);return result
        node=SimpleNamespace(snapshot=snapshot)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(node,'test-token','page'))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        conn=HTTPConnection('127.0.0.1',server.server_address[1],timeout=2)
        try:
            query={'k':'test-token','c':'test-browser'}
            conn.request('GET','/api/state?'+urlencode(query));response=conn.getresponse()
            self.assertEqual(response.version,11);self.assertFalse(response.will_close)
            first=json.loads(response.read());sock=conn.sock
            self.assertIsNotNone(sock);self.assertIn('points',first['depth'])
            self.assertNotIn('original_course',first['course'])
            query.update(dg=first['depth']['geometry_version'],co=first['course']['original_version'],
                         ca=first['course']['adjusted_version'])
            base['depth']['age_s']=.4
            conn.request('GET','/api/state?'+urlencode(query));response=conn.getresponse();second=json.loads(response.read())
            self.assertIs(conn.sock,sock)
            self.assertNotIn('points',second['depth']);self.assertTrue(second['depth']['geometry_unchanged'])
            self.assertNotIn('original_world_enu',second['course'])
            self.assertEqual(second['depth']['age_s'],.4);self.assertEqual(second['sequence'],2)
            base['depth'].update(geometry_version='depth:1:stale',status='stale',points=[],age_s=2)
            conn.request('GET','/api/state?'+urlencode(query));third=json.loads(conn.getresponse().read())
            self.assertFalse(third['depth']['geometry_unchanged']);self.assertEqual(third['depth']['points'],[])
            self.assertIs(conn.sock,sock)
        finally:
            conn.close();server.shutdown();server.server_close();thread.join(2)


if __name__=='__main__':unittest.main()
