"""Execute the shipped page JavaScript with a mocked DOM/network, never the drone.

Run on the host with PYTHONPATH=/tmp/skybeta_dash_jscheck python3 test_ui.py.
This checks behavior and coordinate math; it is not a browser layout test.
"""
import json
from pathlib import Path
import re
import unittest
from urllib.parse import parse_qs, urlparse
import quickjs


class PageTests(unittest.TestCase):
    def setUp(self):
        self.ctx=quickjs.Context()
        self.ctx.eval('''
          let clockNow=1000, requests=[], elements={}, drawCalls=[], replies=[];
          const performance={now:()=>clockNow};
          const location={search:''};
          class URLSearchParams { get(){return '';} }
          const crypto={getRandomValues:a=>a};
          class AbortController { constructor(){this.signal={};} abort(){} }
          const setTimeout=()=>1, clearTimeout=()=>{}, setInterval=()=>1;
          const context=new Proxy({}, {get:(o,k)=>k in o?o[k]:((...args)=>
            drawCalls.push({op:k,args,color:o.fillStyle,stroke:o.strokeStyle,width:o.lineWidth}))});
          function element(id){
            return elements[id]||(elements[id]={value:'0',style:{},width:400,height:400,
              disabled:false,checked:['show_depth','show_course_original','show_course_adjusted'].includes(id),replaceChildren:()=>{},getContext:()=>context,
              getBoundingClientRect:()=>({left:10,top:20,width:200,height:200}),
              reportValidity(){const n=Number.parseFloat(this.value);return Number.isFinite(n)&&
                (this.min===undefined||n>=Number(this.min))&&(this.max===undefined||n<=Number(this.max));}});
          }
          const document={hidden:false,getElementById:element,addEventListener:()=>{},createElement:()=>({})};
          async function fetch(url,args){requests.push({url,args});const reply=replies.shift();
            return reply?{ok:reply.ok!==false,json:async()=>reply.body}:{ok:false,json:async()=>({ok:true,msg:'accepted'})};}
        ''')
        page=(Path(__file__).parent/'dash.html').read_text()
        script=re.search(r'<script>([\s\S]+)</script>',page).group(1)
        self.ctx.eval(script);self.flush()
        self.ctx.eval("$('course_clearance').min=.15;$('course_clearance').max=1;$('course_clearance').value='.30';$('course_offset').min=0;$('course_offset').max=1;$('course_offset').value='.60';")
        state=dict(mode='engaged',reason='',live=True,you_own=True,owner='pilot',link_alive=True,link_age_s=0,
            params=dict(hb_timeout_s=1,fence_r_m=1.5,min_alt_m=.3,max_alt_m=2,v_xy=.5,v_z=.3),
            fc=dict(px4_link=True,armed=True,offboard=True,landed=False,nav='OFFBOARD',age_s=0,clamped=[]),
            home=dict(x=4,y=-3,z=-.8,yaw_deg=90),pos=dict(x=4,y=-3,z=1.2,yaw_deg=90,valid=True,age_s=0),
            engage_xy=[4,-3],target=[4,-3,1.2],cmd=[4,-3,1.2],ground_z=-.8,target_alt=2,
            ready=True,runtime_ready=True,blockers=[],events=[],other_publishers=0,
            was_offboard=True,course_launch=dict(active=False,state='idle',reason=''),
            course_controller=dict(selected='smooth_position',available=True,reason='',
                options=[dict(id='smooth_position',speed_mps=.35,waypoint_acceptance_m=.05),
                         dict(id='velocity_tracker',speed_mps=.35)]))
        self.ctx.eval('S='+json.dumps(state)+'; lastOk=clockNow; requests=[]; render();')

    def flush(self):
        while self.ctx.execute_pending_job():pass

    def value(self,expression):
        return json.loads(self.ctx.eval('JSON.stringify('+expression+')'))

    def posts(self):
        self.flush()
        return self.value("requests.filter(r=>r.args&&r.args.method==='POST').map(r=>JSON.parse(r.args.body))")

    def test_height_uses_reported_limits_and_sends_two_metres(self):
        self.assertEqual(self.value("Number($('alt').max)"),2)
        self.ctx.eval("$('alt').value='2';$('setalt').onclick();")
        self.assertEqual(self.posts()[-1]['alt'],2)
        self.ctx.eval("requests=[];$('alt').value='2.1';$('setalt').onclick();")
        self.assertEqual(self.posts(),[])

    def test_course_start_requires_fresh_adjusted_plan_and_stop_does_not_arm(self):
        self.ctx.eval('''S.course={status:'preview',name:'test',original_count:2,
          anchor:{valid:true,origin_enu:[4,-3,-.8],yaw_rad:Math.PI/2},
          original_world_enu:[[4,-3,1.2],[4,-1,1.2]],
          adjusted_world_enu:[[4,-3,1.2],[4.4,-2,1.2],[4,-1,1.2]],
          adjustment:{status:'adjusted',age_s:.1,execution_supported:true}};
          $('course_radius').value='6';render();''')
        self.assertFalse(self.value("$('course_start').disabled"))
        self.ctx.eval("$('course_start').onclick();")
        self.assertEqual(self.posts()[-1]['op'],'start')
        self.assertEqual(self.posts()[-1]['radius_m'],6)
        self.ctx.eval("requests=[];S.fc.armed=false;render();$('course_start').onclick();")
        self.assertEqual(self.posts(),[])
        self.ctx.eval("S.fc.armed=true;S.course_run={active:true,progress:.4,path_time_s:4,duration_s:10,reason:'following'};render();")
        self.assertTrue(self.value("$('course_start').disabled"))
        self.assertFalse(self.value("$('course_stop').disabled"))
        self.ctx.eval("$('course_stop').onclick();")
        self.assertEqual([p['op'] for p in self.posts()],['stop'])
        self.ctx.eval("requests=[];S.course_run=null;S.pos.x+=.3;render();$('course_start').onclick();")
        self.assertEqual(self.posts(),[])

    def test_map_selection_then_explicit_go_uses_correct_home_coordinates(self):
        self.ctx.eval("$('map').onclick({clientX:110-0.5*(400/4.6)/2,clientY:120-(400/4.6)/2});")
        self.assertEqual(self.posts(),[])
        self.assertEqual(self.value('draftPoint'),dict(forward=1,left=.5))
        self.assertEqual(self.value('homeToWorld(1,.5)'),[3.5,-2])
        self.ctx.eval("$('point_form').onsubmit({preventDefault:()=>{}});")
        sent=self.posts()[-1]
        self.assertEqual((sent['op'],sent['forward'],sent['left']),('goto',1,.5))
        self.assertNotIn('alt',sent)

    def test_numeric_point_outside_fence_and_grounded_vehicle_cannot_send(self):
        self.ctx.eval("$('point_forward').value='2';$('point_left').value='0';selectPoint();$('point_form').onsubmit({preventDefault:()=>{}});")
        self.assertTrue(self.value("$('goto').disabled"));self.assertEqual(self.posts(),[])
        self.ctx.eval("$('point_forward').value='1';S.fc.landed=true;selectPoint();$('point_form').onsubmit({preventDefault:()=>{}});")
        self.assertEqual(self.posts(),[])

    def test_lost_link_and_home_change_clear_actionable_selection(self):
        self.ctx.eval("$('point_forward').value='1';selectPoint();clockNow=2500;$('point_form').onsubmit({preventDefault:()=>{}});")
        self.assertEqual(self.posts(),[])
        self.ctx.eval('S.home.x=5;render();')
        self.assertIsNone(self.value('draftPoint'))
        self.assertTrue(self.value("$('goto').disabled"))

    def test_go_uses_runtime_readiness_not_ground_height_or_ekf(self):
        self.ctx.eval("S.ready=false;S.floor_offset=-2;$('point_forward').value='1';selectPoint();")
        self.assertFalse(self.value("$('goto').disabled"))
        self.ctx.eval("S.runtime_ready=false;renderPoint();$('point_form').onsubmit({preventDefault:()=>{}});")
        self.assertTrue(self.value("$('goto').disabled"))
        self.assertEqual(self.posts(),[])

    def set_depth(self, **changes):
        depth=dict(source='Depth Anything V2',status='live',reason='',age_s=.2,
                   cell_m=.08,points=[[3.5,-2]],camera_xy=[4,-3],view_polygon=[])
        depth.update(changes)
        self.ctx.eval('S.depth='+json.dumps(depth)+';drawCalls=[];render();')

    def depth_cells(self):
        return self.value("drawCalls.filter(c=>c.op==='fillRect').map(c=>c.args)")

    def test_depth_world_cells_share_map_home_rotation_and_reset(self):
        self.set_depth()
        cell=self.depth_cells()[0];scale=400/4.6
        self.assertAlmostEqual(cell[0]+cell[2]/2,200-.5*scale)
        self.assertAlmostEqual(cell[1]+cell[3]/2,200-scale)
        self.assertAlmostEqual(cell[2],.08*scale)
        self.assertTrue(self.value("drawCalls.findIndex(c=>c.op==='fillRect') < drawCalls.findIndex(c=>c.op==='arc')"))
        self.ctx.eval("S.mode='idle';S.home.x=5;S.home.yaw_deg=0;drawCalls=[];render();")
        cell=self.depth_cells()[0]
        self.assertAlmostEqual(cell[0]+cell[2]/2,200-scale)
        self.assertAlmostEqual(cell[1]+cell[3]/2,200+1.5*scale)
        self.assertEqual(self.posts(),[])

    def test_stale_unscaled_waiting_depth_and_lost_api_never_draw_cells(self):
        for changes in (dict(status='stale'),dict(status='waiting'),dict(status='unscaled',reason='waiting for scale'),
                        dict(status='error'),dict(age_s=1.51),dict(age_s=None)):
            self.set_depth(**changes)
            self.assertEqual(self.depth_cells(),[])
            self.assertIn(changes.get('status','stale'),self.value("$('depth_status').textContent"))
        self.set_depth()
        self.assertEqual(len(self.depth_cells()),1)
        self.ctx.eval('clockNow=2101;drawCalls=[];watchdog();')
        self.assertEqual(self.depth_cells(),[])
        self.assertIn('dashboard telemetry is stale',self.value("$('depth_status').textContent"))

    def test_depth_toggle_is_display_only_and_empty_live_frame_stays_live(self):
        self.set_depth()
        self.ctx.eval("$('point_forward').value='1';selectPoint();$('show_depth').checked=false;drawCalls=[];$('show_depth').onchange();")
        self.assertEqual(self.depth_cells(),[])
        self.assertFalse(self.value("$('goto').disabled"))
        self.assertIn('hidden',self.value("$('depth_status').textContent"))
        self.assertEqual(self.posts(),[])
        self.ctx.eval("$('show_depth').checked=true;drawCalls=[];$('show_depth').onchange();")
        self.assertEqual(len(self.depth_cells()),1)
        self.set_depth(points=[])
        self.assertEqual(self.depth_cells(),[])
        self.assertIn('live',self.value("$('depth_status').textContent"))
        self.assertNotIn('waiting',self.value("$('depth_status').textContent"))

    def test_speed_help_tracks_live_horizontal_limit(self):
        self.ctx.eval('S.params.v_xy=1;render();')
        self.assertEqual(self.value("$('horizontal_speed').textContent"),'1')
        self.ctx.eval('S.params.course_v_xy=1.2;S.heading={max_rate_dps:90};render();')
        self.assertIn('1.2 m/s',self.value("$('course_motion_limits').textContent"))
        self.assertIn('90°/s',self.value("$('course_motion_limits').textContent"))
        self.assertEqual(self.value("$('horizontal_speed').textContent"),'1')

    def set_course(self, **changes):
        course=dict(revision=2,name='Maze',status='preview',reason='',original_count=1394,
                    original_version='original-1',adjusted_version='adjusted-1',preview_enabled=True,
                    anchor=dict(valid=True,origin_enu=[4,-3,-.8],yaw_rad=0),
                    original_course=[[0,0,1],[5,0,1],[10,2,1]],
                    adjusted_course=[[0,0,1],[5,.5,1],[10,2,1]],
                    original_world_enu=[[4,-3,.2],[9,-3,.2],[14,-1,.2]],
                    adjusted_world_enu=[[4,-3,.2],[9,-2.5,.2],[14,-1,.2]],
                    original_yaw_enu_rad=[0,.380506,.380506],
                    adjusted_yaw_enu_rad=[.099669,.291457,.291457],
                    adjustment=dict(status='adjusted',reason='Wall preview',age_s=.2,
                                    clearance_m=.3,max_offset_m=.6,execution_supported=True))
        course.update(changes)
        self.ctx.eval('S.course='+json.dumps(course)+';drawCalls=[];render();')

    def test_course_fitted_map_click_preserves_home_go_coordinates_and_fence_center(self):
        self.set_course()
        self.ctx.eval("$('fit_course').checked=true;drawCalls=[];$('fit_course').onchange();")
        self.assertTrue(self.value('mapView.fitted'))
        self.assertIn('1394 waypoints',self.value("$('course_status').textContent"))
        self.assertIn('+X is up',self.value("$('map_frame').textContent"))
        self.ctx.eval("const clicked=homeToWorld(1,.5);$('map').onclick({clientX:10+mapView.X(...clicked)/2,clientY:20+mapView.Y(...clicked)/2});")
        self.assertEqual(self.value('draftPoint'),dict(forward=1,left=.5))
        self.assertEqual(self.posts(),[])
        fence=self.value("drawCalls.filter(c=>c.op==='arc'&&c.stroke==='#e3b341').slice(-1)[0].args")
        center=self.value('S.engage_xy.map((_,i)=>i?mapView.Y(...S.engage_xy):mapView.X(...S.engage_xy))')
        self.assertAlmostEqual(fence[0],center[0]);self.assertAlmostEqual(fence[1],center[1])
        self.assertNotAlmostEqual(center[1],200)
        self.ctx.eval("$('point_form').onsubmit({preventDefault:()=>{}});")
        self.assertEqual(self.posts()[-1]['op'],'goto')
        self.assertEqual(self.posts()[-1]['forward'],1)
        self.assertEqual(self.posts()[-1]['left'],.5)

    def test_course_routes_share_depth_transform_and_stale_adjusted_is_hidden(self):
        self.set_depth(points=[[9,-3]])
        self.set_course()
        self.ctx.eval("$('fit_course').checked=true;drawCalls=[];draw();")
        cell=self.depth_cells()[0]
        line=self.value("drawCalls.find(c=>c.op==='lineTo'&&c.stroke==='#a5afbb').args")
        self.assertAlmostEqual(cell[0]+cell[2]/2,line[0]);self.assertAlmostEqual(cell[1]+cell[3]/2,line[1])
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#d2a8ff')"))
        self.ctx.eval('S.course.adjustment.status="stale";drawCalls=[];render();')
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#d2a8ff')"))
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#a5afbb')"))
        self.ctx.eval("S.course.adjustment.status='adjusted';clockNow=2101;drawCalls=[];watchdog();")
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#d2a8ff')"))

    def test_course_controls_send_only_course_endpoint_and_capture_requires_ground(self):
        self.set_course()
        self.assertTrue(self.value("$('course_capture').disabled"))
        self.ctx.eval("$('course_capture').onclick();")
        self.assertEqual(self.posts(),[])
        self.ctx.eval("S.fc.armed=false;S.fc.landed=true;S.runtime_ready=false;render();$('course_capture').onclick();")
        self.assertEqual(self.posts()[-1]['op'],'capture_anchor')
        self.ctx.eval("$('course_clearance').value='.3';$('course_offset').value='.6';$('course_preview').onclick();")
        self.assertEqual(self.posts()[-1]['op'],'preview')
        self.assertEqual(self.posts()[-1]['clearance_m'],.3)
        count=len(self.posts())
        self.ctx.eval("$('course_offset').value='1.01';$('course_preview').onclick();")
        self.assertEqual(len(self.posts()),count)
        self.ctx.eval("$('course_offset').value='.6';S.pos.age_s=2;render();$('course_capture').onclick();")
        self.assertEqual(len(self.posts()),count)
        self.ctx.eval('S.pos.age_s=0;render();')
        route=dict(version=1,frame='course_flu',units='m',name='Test',waypoints=[[0,0,1],[1,0,1]],times_s=[0,1])
        self.ctx.eval("$('course_json').value="+json.dumps(json.dumps(route))+";$('course_save').onclick();")
        self.assertEqual(self.posts()[-1]['trajectory'],route)
        self.assertTrue(self.value("requests.every(r=>r.url.startsWith('/api/course?'))"))
        self.ctx.eval("requests=[];S.you_own=false;render();$('course_save').onclick();$('course_capture').onclick();$('course_preview').onclick();")
        self.assertEqual(self.posts(),[])

    def test_unanchored_course_and_visibility_toggles_do_not_command_flight(self):
        self.set_course(anchor=None,status='needs_anchor')
        self.assertTrue(self.value("$('course_preview').disabled"))
        self.assertTrue(self.value("$('fit_course').disabled"))
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&['#a5afbb','#d2a8ff'].includes(c.stroke))"))
        self.set_course()
        self.ctx.eval("$('show_course_original').checked=false;$('show_course_adjusted').checked=false;drawCalls=[];$('show_course_original').onchange();")
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&['#a5afbb','#d2a8ff'].includes(c.stroke))"))
        self.assertEqual(self.posts(),[])

    def test_adjusted_preview_is_fresh_at_one_point_two_and_ghosts_after_one_point_five_seconds(self):
        self.set_course()
        self.ctx.eval('S.course.adjustment.age_s=1.2;drawCalls=[];draw();')
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#d2a8ff')"))
        self.ctx.eval('S.course.adjustment.age_s=1.5;drawCalls=[];draw();')
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#d2a8ff')"))
        self.ctx.eval('clockNow=1001;drawCalls=[];watchdog();')
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#d2a8ff')"))
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#e5a84b')"))
        self.assertIn('Last preview — stale, 1.5s old',self.value("$('course_adjustment').textContent"))
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#a5afbb')"))
        self.assertTrue(self.value('performance.now()-lastOk<1000'))

    def test_stale_api_shows_amber_dashed_last_preview_with_headings_then_expires(self):
        self.set_course()
        yaws=self.value('coursePaths().adjustedYaws')
        self.ctx.eval('clockNow=2101;drawCalls=[];watchdog();')
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#d2a8ff')"))
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#e5a84b'&&c.width===2)"))
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='setLineDash'&&c.stroke==='#e5a84b'&&c.args[0].join(',')==='6,4')"))
        self.assertEqual(self.value('coursePaths().adjustedYaws'),yaws)
        self.assertIn('Last preview — stale, 1.3s old',self.value("$('course_adjustment').textContent"))
        self.ctx.eval('clockNow=10801;drawCalls=[];watchdog();')
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#e5a84b')"))
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#a5afbb')"))
        self.assertNotIn('Last preview',self.value("$('course_adjustment').textContent"))
        self.assertEqual(self.posts(),[])

    def test_last_preview_clears_on_route_reference_disabled_or_blocked_changes(self):
        for change in ("S.course.original_version='replacement'", "S.course.anchor.origin_enu[0]=8",
                       "S.course.anchor.yaw_rad=1", "S.course.preview_enabled=false",
                       "S.course.adjustment.status='blocked'", "S.course.anchor=null"):
            self.set_course()
            self.ctx.eval("S.course.adjustment.status='stale';"+change+";drawCalls=[];render();")
            self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#e5a84b')"),change)
            self.assertNotIn('Last preview',self.value("$('course_adjustment').textContent"),change)
            self.assertIsNone(self.value('lastCoursePreview'),change)

    def test_initial_missing_preview_is_not_invented_and_shared_start_has_one_marker(self):
        self.set_course(adjusted_world_enu=[],adjusted_yaw_enu_rad=[],
                        adjustment=dict(status='waiting',reason='Waiting for depth',age_s=None))
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#e5a84b')"))
        self.assertNotIn('Last preview',self.value("$('course_adjustment').textContent"))
        labels=self.value("drawCalls.filter(c=>c.op==='fillText').map(c=>c.args[0])")
        self.assertIn('home / course start',labels)
        self.assertNotIn('home',labels);self.assertNotIn('course start',labels)
        self.ctx.eval('S.course.anchor.origin_enu[0]=6;drawCalls=[];draw();')
        labels=self.value("drawCalls.filter(c=>c.op==='fillText').map(c=>c.args[0])")
        self.assertIn('home',labels);self.assertIn('course start',labels)

    def test_preview_clearance_matches_backend_point_one_five_to_one_metre_bounds(self):
        self.set_course()
        for value in ('.15','1'):
            self.ctx.eval("$('course_clearance').value="+json.dumps(value)+";$('course_preview').onclick();")
        self.assertEqual([p['clearance_m'] for p in self.posts()],[.15,1])
        self.ctx.eval('requests=[];')
        for value in ('.149','1.001'):
            self.ctx.eval("$('course_clearance').value="+json.dumps(value)+";$('course_preview').onclick();")
        self.assertEqual(self.posts(),[])

    def versioned_state(self):
        self.set_depth(geometry_version='depth-1')
        self.set_course(original_version='original-1',adjusted_version='adjusted-1')

    def omitted_geometry(self):
        self.ctx.eval('''
          const oldState=S, nextState=JSON.parse(JSON.stringify(S));
          delete nextState.depth.points;delete nextState.depth.camera_xy;delete nextState.depth.view_polygon;
          nextState.depth.geometry_unchanged=true;
          delete nextState.course.original_world_enu;delete nextState.course.adjusted_world_enu;
          delete nextState.course.original_yaw_enu_rad;delete nextState.course.adjusted_yaw_enu_rad;
          delete nextState.course.original_course;delete nextState.course.adjusted_course;
          nextState.course.original_unchanged=true;nextState.course.adjusted_unchanged=true;
        ''')

    def test_matching_conditional_geometry_reuses_arrays_but_replaces_metadata(self):
        self.versioned_state();self.omitted_geometry()
        self.ctx.eval("nextState.depth.age_s=.6;nextState.depth.reason='fresh metadata';nextState.course.adjustment.age_s=.4;applyState(nextState);render();")
        self.assertTrue(self.value('S.depth.points===oldState.depth.points'))
        self.assertTrue(self.value('S.depth.camera_xy===oldState.depth.camera_xy'))
        self.assertTrue(self.value('S.course.original_world_enu===oldState.course.original_world_enu'))
        self.assertTrue(self.value('S.course.adjusted_world_enu===oldState.course.adjusted_world_enu'))
        self.assertTrue(self.value('S.course.original_yaw_enu_rad===oldState.course.original_yaw_enu_rad'))
        self.assertTrue(self.value('S.course.adjusted_yaw_enu_rad===oldState.course.adjusted_yaw_enu_rad'))
        self.assertEqual(self.value('S.depth.age_s'),.6)
        self.assertEqual(self.value('S.depth.reason'),'fresh metadata')
        self.assertEqual(self.value('S.course.adjustment.age_s'),.4)
        self.assertTrue(self.value("!('original_course' in S.course)&&!('adjusted_course' in S.course)"))
        self.assertIn('1394 waypoints',self.value("$('course_status').textContent"))

    def test_new_full_geometry_replaces_old_and_poll_encodes_version_keys(self):
        self.versioned_state()
        self.ctx.eval('''
          const nextState=JSON.parse(JSON.stringify(S));
          nextState.depth.geometry_version='g &/+?';nextState.depth.points=[[8,9]];
          nextState.course.original_version=0;nextState.course.original_world_enu=[[1,2,3],[4,5,6]];
          nextState.course.original_yaw_enu_rad=[.785398,.785398];
          nextState.course.adjusted_version='r#2';nextState.course.adjusted_world_enu=[];nextState.course.adjusted_yaw_enu_rad=[];
          applyState(nextState);requests=[];poll();
        ''')
        self.flush()
        self.assertEqual(self.value('S.depth.points'),[[8,9]])
        self.assertEqual(self.value('S.course.original_world_enu'),[[1,2,3],[4,5,6]])
        self.assertEqual(self.value('S.course.adjusted_world_enu'),[])
        self.assertEqual(self.value('S.course.adjusted_yaw_enu_rad'),[])
        query=parse_qs(urlparse(self.value('requests[0].url')).query,keep_blank_values=True)
        self.assertEqual({key:query[key] for key in ('dg','co','ca')},
                         dict(dg=['g &/+?'],co=['0'],ca=['r#2']))
        self.assertEqual(self.posts(),[])

    def test_conditional_version_mismatch_clears_geometry_and_requests_full_next_poll(self):
        self.versioned_state();self.omitted_geometry()
        self.ctx.eval('''
          nextState.depth.geometry_version='different';nextState.course.original_version='different';
          nextState.course.adjusted_version='different';applyState(nextState);requests=[];poll();
        ''')
        self.flush()
        self.assertEqual(self.value('S.depth.points'),[])
        self.assertIsNone(self.value('S.depth.camera_xy'))
        self.assertEqual(self.value('S.depth.view_polygon'),[])
        self.assertEqual(self.value('S.course.original_world_enu'),[])
        self.assertEqual(self.value('S.course.adjusted_world_enu'),[])
        self.assertEqual(self.value('S.course.original_yaw_enu_rad'),[])
        self.assertEqual(self.value('S.course.adjusted_yaw_enu_rad'),[])
        query=parse_qs(urlparse(self.value('requests[0].url')).query)
        self.assertTrue(set(query).isdisjoint(('dg','co','ca')))
        self.assertEqual(self.posts(),[])

    def test_unchanged_geometry_cannot_restore_stale_depth_or_adjustment_metadata(self):
        self.versioned_state();self.omitted_geometry()
        self.ctx.eval("nextState.depth.status='stale';nextState.depth.age_s=2;nextState.course.adjustment.status='stale';nextState.course.adjustment.age_s=1.2;applyState(nextState);drawCalls=[];render();")
        self.assertEqual(self.value('S.depth.status'),'stale')
        self.assertEqual(self.value('S.depth.age_s'),2)
        self.assertEqual(self.depth_cells(),[])
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#d2a8ff')"))
        self.assertTrue(self.value("drawCalls.some(c=>c.op==='stroke'&&c.stroke==='#a5afbb')"))

    def test_command_response_uses_full_state_before_conditional_poll(self):
        self.versioned_state()
        self.ctx.eval("const nextState=JSON.parse(JSON.stringify(S));nextState.depth.geometry_version='command-full';nextState.depth.points=[[7,8]];replies.push({body:{ok:true,msg:'held',state:nextState}});cmd('hold');")
        self.flush()
        self.assertEqual(self.value('S.depth.points'),[[7,8]])
        self.ctx.eval("const courseState=JSON.parse(JSON.stringify(S));courseState.course.original_version='course-full';courseState.course.original_world_enu=[];courseState.course.original_yaw_enu_rad=[];replies.push({body:{ok:true,msg:'saved',state:courseState}});courseCmd('preview',{clearance_m:.3,max_offset_m:.6});")
        self.flush()
        self.assertEqual(self.value('S.course.original_world_enu'),[])
        query=parse_qs(urlparse(self.value('stateRequestUrl()')).query)
        self.assertEqual(query['dg'],['command-full'])
        self.assertEqual(query['co'],['course-full'])

    def test_route_heading_arrows_use_world_yaw_and_map_rotation_and_are_sparse(self):
        self.set_course(original_yaw_enu_rad=[0,0,0],adjusted_yaw_enu_rad=[0,0,0])
        stem=self.value("drawCalls.filter(c=>c.width===1.5&&c.stroke==='#a5afbb'&&['moveTo','lineTo'].includes(c.op)).slice(0,2).map(c=>c.args)")
        # Home points north (90 degrees): an east-facing nose points right.
        self.assertAlmostEqual(stem[1][0]-stem[0][0],10)
        self.assertAlmostEqual(stem[1][1]-stem[0][1],0)
        self.ctx.eval("$('fit_course').checked=true;drawCalls=[];draw();")
        stem=self.value("drawCalls.filter(c=>c.width===1.5&&c.stroke==='#a5afbb'&&['moveTo','lineTo'].includes(c.op)).slice(0,2).map(c=>c.args)")
        # Fitted course points east: the same ENU heading points up.
        self.assertAlmostEqual(stem[1][0]-stem[0][0],0)
        self.assertAlmostEqual(stem[1][1]-stem[0][1],-10)
        self.ctx.eval("S.course.original_world_enu=Array.from({length:600},(_,i)=>[4+i/50,-3,1]);S.course.original_yaw_enu_rad=Array(600).fill(0);drawCalls=[];draw();")
        arrows=self.value("drawCalls.filter(c=>c.op==='stroke'&&c.width===1.5&&c.stroke==='#a5afbb').length")
        self.assertGreater(arrows,0);self.assertLessEqual(arrows,12)
        self.ctx.eval("S.course.adjustment.status='stale';drawCalls=[];render();")
        self.assertFalse(self.value("drawCalls.some(c=>c.op==='stroke'&&c.width===1.5&&c.stroke==='#d2a8ff')"))

    def test_heading_status_explains_flight_phase_without_changing_go_controls(self):
        self.assertEqual(self.value("$('heading_status').textContent"),
                         'Horizontal moves turn to face travel before advancing.')
        self.ctx.eval("$('point_forward').value='1';selectPoint();S.heading={policy:'path_tangent',phase:'turning',reason:'Turning to face the path'};render();")
        self.assertEqual(self.value("$('heading_status').textContent"),'Forward camera: Turning to face the path')
        self.assertFalse(self.value("$('goto').disabled"))
        self.ctx.eval("S.mode='idle';render();")
        self.assertEqual(self.value("$('heading_status').textContent"),
                         'Horizontal moves turn to face travel before advancing.')
        self.assertEqual(self.posts(),[])


if __name__=='__main__':unittest.main(verbosity=2)
