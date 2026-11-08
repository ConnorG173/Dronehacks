import copy
import unittest

from state_wire import compact_state


class WireTests(unittest.TestCase):
    def state(self):
        return dict(pos={'age_s':.03},fc={'age_s':.1},depth=dict(
            geometry_version='d:1:live',status='live',age_s=.2,points=[[1,2]],
            camera_xy=[0,0],view_polygon=[]),course=dict(
            original_version='c:1',adjusted_version='c:1:2:adjusted',
            original_count=2,adjusted_count=2,original_course=[[0,0,1],[1,0,1]],
            adjusted_course=[[0,0,1],[1,.2,1]],original_world_enu=[[10,20,1],[11,20,1]],
            adjusted_world_enu=[[10,20,1],[11,20.2,1]],adjustment={'status':'adjusted','age_s':.2}))

    def test_unchanged_geometry_omitted_but_freshness_always_sent(self):
        original=self.state();state=compact_state(copy.deepcopy(original),{
            'dg':'d:1:live','co':'c:1','ca':'c:1:2:adjusted'})
        self.assertNotIn('points',state['depth']);self.assertTrue(state['depth']['geometry_unchanged'])
        self.assertNotIn('original_world_enu',state['course']);self.assertTrue(state['course']['original_unchanged'])
        self.assertNotIn('adjusted_world_enu',state['course']);self.assertTrue(state['course']['adjusted_unchanged'])
        self.assertNotIn('original_course',state['course'])
        self.assertEqual(state['depth']['age_s'],.2);self.assertEqual(state['pos'],original['pos'])
        self.assertEqual(state['course']['adjustment'],original['course']['adjustment'])

    def test_changed_or_stale_geometry_is_always_sent(self):
        state=self.state();state['depth'].update(geometry_version='d:1:stale',status='stale',points=[])
        result=compact_state(state,{'dg':'d:1:live','co':'old-process:1','ca':'old'})
        self.assertFalse(result['depth']['geometry_unchanged']);self.assertEqual(result['depth']['points'],[])
        self.assertIn('original_world_enu',result['course']);self.assertIn('adjusted_world_enu',result['course'])
        self.assertIn('points',compact_state(self.state())['depth'])

    def test_unknown_versions_cannot_suppress_geometry(self):
        state=self.state()
        for item in (state['depth'],state['course']):
            for key in list(item):
                if key.endswith('version'):del item[key]
        result=compact_state(state,{'dg':'None','co':'None','ca':'None'})
        self.assertIn('points',result['depth']);self.assertIn('original_world_enu',result['course'])


if __name__=='__main__':unittest.main()
