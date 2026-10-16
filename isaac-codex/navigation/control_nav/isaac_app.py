"""Isaac Sim adapter and map/control panel. Run start_control_navigation.py in Isaac."""
import asyncio
import gc
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw
import yaml
from scipy.ndimage import distance_transform_edt
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd
from pxr import UsdGeom, Gf
from isaacsim.core.api import World
from .core import Grid, Navigator, Pose, Goal

ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / 'warehouse_map__n2duzc9/map.yaml'
CALLBACK = 'warehouse_control_navigation'


def load_grid(path, radius=0.65, stride=4):
    data=yaml.safe_load(Path(path).read_text())
    if abs(data['origin'][2])>1e-9:
        raise ValueError('This warehouse adapter requires a map with zero origin yaw')
    pixels=np.array(Image.open(Path(path).parent/data['image']).convert('L'))
    occupancy=pixels/255.0 if data.get('negate',0) else 1-pixels/255.0
    free=occupancy<data.get('free_thresh',0.196)
    resolution=float(data['resolution'])
    # Pad before distance transform so the outside of the map is also occupied.
    clearance=distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*resolution
    # Additional half-cell margin covers occupied pixel area, not just its centre.
    safe=np.flipud(clearance>radius+resolution*0.7072)
    h,w=safe.shape
    ph,pw=(-h)%stride,(-w)%stride
    safe=np.pad(safe,((0,ph),(0,pw)),constant_values=False)
    coarse=safe.reshape(safe.shape[0]//stride,stride,safe.shape[1]//stride,stride).all(axis=(1,3))
    return Grid(coarse.shape[1],coarse.shape[0],resolution*stride,data['origin'][:2],coarse.astype('uint8').tobytes())


class ControlNavigationApp:
    def __init__(self):
        self.grid=load_grid(MAP)
        self.nav=Navigator(self.grid)
        self.robot=None
        self.stage=omni.usd.get_context().get_stage()
        self.world=None
        self.extension=None
        self.waypoints=[]
        self.known_obstacles=[]
        self.pose=Pose(0,0,0)
        self.status='Initializing robot…'
        self.closed=False
        self.last_draw=0.0
        self.last_state=None
        self.events=[]
        self.window=None
        self.timeline_sub=None
        self.update_sub=None
        self.recovering=False
        self.auto_recovery_attempted=False
        self.provider=ui.ByteImageProvider()
        self.log_path=ROOT/'navigation/logs/control_navigation.jsonl'
        self.log_path.parent.mkdir(exist_ok=True,parents=True)

    async def start(self):
        self.world=World.instance()
        self.extension=next((o for o in gc.get_objects()
                            if type(o).__name__=='MobilityGenExtension' and hasattr(o,'scenario')),None)
        scenario=getattr(self.extension,'scenario',None)
        if self.world:
            for name in ('scenario_physics',CALLBACK):
                if self.world.physics_callback_exists(name): self.world.remove_physics_callback(name)
        timeline=omni.timeline.get_timeline_interface()
        timeline.pause()
        stage=omni.usd.get_context().get_stage()
        if not stage or not stage.GetPrimAtPath('/World/robot/chassis_link'):
            raise RuntimeError('Open Team_Warehouse_MobilityGen.usd with Carter at /World/robot first')
        if self.robot is not None:
            pass  # Reuse wrappers when re-running the launcher on the same stage.
        elif scenario is not None:
            self.robot=scenario.robot
            self.extension.disable_recording()
        else:
            # Wrap the existing articulation. Do not spawn a second robot or camera.
            from isaacsim.robot.wheeled_robots.robots import WheeledRobot
            from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
            from isaacsim.replicator.mobility_gen.examples.robots import CarterRobot
            from isaacsim.core.prims import Articulation
            if self.world is None:
                self.world=World(physics_dt=0.005)
                await self.world.initialize_simulation_context_async()
            raw=self.world.scene.add(WheeledRobot('/World/robot',name='control_carter',
                wheel_dof_names=CarterRobot.wheel_dof_names,create_robot=False))
            view=self.world.scene.add(Articulation('/World/robot/chassis_link',name='control_carter_view'))
            controller=DifferentialController(name='control_carter_drive',wheel_radius=0.14,wheel_base=0.413)
            self.robot=CarterRobot('/World/robot',raw,view,controller)
        await self.world.reset_async()
        self._write(0,0,0.005)
        # Absolute camera transform avoids MobilityGen Build's accumulated transforms.
        camera=stage.GetPrimAtPath('/World/robot/chassis_link/chase_camera')
        if camera:
            xf=UsdGeom.Xformable(camera)
            matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(-1.5,0,0.8),Gf.Vec3d(0.3,0,0.25),Gf.Vec3d(0,0,1)).GetInverse()
            xf.MakeMatrixXform().Set(matrix)
        self.world.add_physics_callback(CALLBACK,self._physics)
        self.pose=self.read_pose()
        self.build_ui()
        self.timeline_sub=timeline.get_timeline_event_stream().create_subscription_to_pop(self._timeline)
        self.update_sub=omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._update)
        self.status='Ready — choose a map goal; MobilityGen random driving is detached'
        self._event('ready')
        await self.world.play_async()
        self.draw()
        viewport=ui.Workspace.get_window('Viewport')
        if viewport:
            self.window.dock_in(viewport,ui.DockPosition.RIGHT,0.34)

    def read_pose(self):
        p=self.robot.get_pose_2d()
        return Pose(float(p.x),float(p.y),float(p.theta))

    def _write(self,v,w,dt):
        self.robot.action.set_value(np.array([v,w],dtype=float))
        self.robot.write_action(dt)

    def _physics(self,dt):
        if self.closed or self.recovering: return
        try:
            if not self.robot.robot._articulation_view.is_physics_handle_valid():
                self.request_recovery()
                return
            self.pose=self.read_pose()
            v,w=self.nav.step(self.pose,float(dt))
            self._write(v,w,dt)
            if self.nav.state!=self.last_state:
                self.last_state=self.nav.state
                self._event('state')
        except Exception as e:
            self.nav.stop('FAILED',str(e))
            try: self._write(0,0,dt)
            except Exception: pass
            if not self.events or self.events[-1]['message']!=str(e): self._event('error')

    def _timeline(self,event):
        if self.closed or self.recovering: return
        if event.type==int(omni.timeline.TimelineEventType.STOP):
            self.nav.cancel()
            self.auto_recovery_attempted=False
            self.status='Timeline stopped. Play will reconnect physics; then choose a mission.'
        elif event.type==int(omni.timeline.TimelineEventType.PAUSE):
            self.nav.pause()
        elif event.type==int(omni.timeline.TimelineEventType.PLAY):
            if not self.robot.robot._articulation_view.is_physics_handle_valid():
                self.request_recovery()
            elif self.nav.state=='PAUSED':
                self.nav.resume(self.read_pose())

    def request_recovery(self):
        # One attempt per Stop/Play cycle; never create a reset loop on failure.
        if self.recovering or self.auto_recovery_attempted: return
        self.auto_recovery_attempted=True
        self.nav.cancel()
        self.status='Reconnecting wheel controller…'
        self.recovery_task=asyncio.ensure_future(self.recover())

    async def recover(self):
        if self.recovering: return
        self.recovering=True
        try:
            self.nav.cancel()
            if self.world.physics_callback_exists(CALLBACK): self.world.remove_physics_callback(CALLBACK)
            await self.world.reset_async()
            if not self.robot.robot._articulation_view.is_physics_handle_valid():
                raise RuntimeError('Robot joint handles remain invalid after reset')
            self._write(0,0,0.005)
            self.pose=self.read_pose()
            self.world.add_physics_callback(CALLBACK,self._physics)
            self.status='Ready. Click Run queue or Go to goal to move.'
            await self.world.play_async()
        except Exception as e:
            self.status='Recovery failed: '+str(e)
            if self.nav.state!='ESTOP': self.nav.stop('FAILED',self.status)
        finally:
            self.recovering=False

    def _event(self,kind):
        row={'event':kind,'wall_time':time.time(),'sim_elapsed':self.nav.time,'state':self.nav.state,
             'message':self.nav.message,'pose':[self.pose.x,self.pose.y,self.pose.yaw],
             'completed':self.nav.completed,'laps':self.nav.laps,'distance_m':self.nav.distance}
        self.events.append(row)
        with self.log_path.open('a') as f: f.write(json.dumps(row)+'\n')

    def invoke(self,fn):
        try:
            fn()
            self.status=self.nav.message
        except Exception as e:
            self.status=str(e)
        self.draw()

    def selected_goal(self):
        yaw=math.radians(self.yaw_model.as_float) if self.yaw_enabled.as_bool else None
        return Goal(self.x_model.as_float,self.y_model.as_float,yaw)

    def go(self):
        if not self.world.is_playing():
            raise ValueError('Start simulation or use Reinitialize physics first')
        self.nav.navigate(self.read_pose(),[self.selected_goal()])

    def run_queue(self,patrol=False):
        if not self.world.is_playing(): raise ValueError('Start simulation first')
        self.nav.navigate(self.read_pose(),self.waypoints,patrol)

    def resume(self):
        if not self.robot.robot._articulation_view.is_physics_handle_valid():
            raise ValueError('Use Reinitialize physics after Stop')
        self.nav.resume(self.read_pose())
        omni.timeline.get_timeline_interface().play()

    def manual(self,v,w):
        if not self.world.is_playing(): raise ValueError('Start simulation before a manual command')
        self.nav.manual(v,w)

    def set_known_obstacles(self,circles):
        """Input physical obstacle circles (x,y,r), supplied by scenario/operator, not sensing."""
        circles=list(circles)
        if any(len(c)!=3 or not all(math.isfinite(v) for v in c) or c[2]<0 for c in circles):
            raise ValueError('Obstacle coordinates and radius must be finite; radius must be nonnegative')
        self.grid.obstacles([(x,y,r+0.65) for x,y,r in circles])
        self.known_obstacles=circles
        self._event('known_obstacles_updated')

    def _click_map(self,x,y,button,modifiers):
        if button!=0: return
        # Omni UI sends screen coordinates; convert to coordinates inside the map.
        x -= self.map_widget.screen_position_x
        y -= self.map_widget.screen_position_y
        width,height=self.map_widget.computed_width,self.map_widget.computed_height
        if width<=0 or height<=0: return
        if not (0<=x<=width and 0<=y<=height): return
        wx=self.grid.origin[0]+x/width*self.grid.width*self.grid.resolution
        wy=self.grid.origin[1]+(1-y/height)*self.grid.height*self.grid.resolution
        self.x_model.set_value(wx)
        self.y_model.set_value(wy)
        self.status='Goal selected; press Go to goal'
        self.draw()

    def build_ui(self):
        self.window=ui.Window('Warehouse Control & Navigation',width=610,height=920)
        self.window.set_visibility_changed_fn(lambda visible: self.cancel() if not visible else None)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5):
                    ui.Label('Known map + simulator pose | No sensing / SLAM',height=22)
                    self.status_label=ui.Label('',height=48,word_wrap=True)
                    with ui.HStack(height=30):
                        ui.Button('Pause',clicked_fn=lambda:self.invoke(self.nav.pause))
                        ui.Button('Resume',clicked_fn=lambda:self.invoke(self.resume))
                        ui.Button('Cancel',clicked_fn=self.cancel)
                        ui.Button('E-STOP',clicked_fn=self.estop,style={'background_color':0xff3030b0})
                    with ui.HStack(height=26):
                        ui.Button('Clear E-stop',clicked_fn=lambda:self.invoke(self.nav.clear_estop))
                        ui.Button('Reinitialize physics',clicked_fn=lambda:asyncio.ensure_future(self.recover()))
                    ui.Label('Manual: each press drives for 0.5 simulation seconds',height=20)
                    with ui.HStack(height=28):
                        for title,v,w in [('Forward',0.35,0),('Reverse',-0.25,0),('Left',0,0.6),('Right',0,-0.6)]:
                            ui.Button(title,clicked_fn=lambda v=v,w=w:self.invoke(lambda:self.manual(v,w)))
                    with ui.HStack(height=26):
                        ui.Label('Goal X',width=50)
                        self.x_model=ui.SimpleFloatModel(self.pose.x)
                        ui.FloatField(model=self.x_model)
                        ui.Label('Y',width=18)
                        self.y_model=ui.SimpleFloatModel(self.pose.y)
                        ui.FloatField(model=self.y_model)
                        self.yaw_enabled=ui.SimpleBoolModel(False)
                        ui.CheckBox(model=self.yaw_enabled,width=20)
                        ui.Label('Yaw °',width=42)
                        self.yaw_model=ui.SimpleFloatModel(0)
                        ui.FloatField(model=self.yaw_model,width=65)
                    with ui.HStack(height=28):
                        ui.Button('Go to goal',clicked_fn=lambda:self.invoke(self.go))
                        ui.Button('Add waypoint',clicked_fn=lambda:self.invoke(lambda:self.waypoints.append(self.selected_goal())))
                        ui.Button('Clear queue',clicked_fn=lambda:self.invoke(self.waypoints.clear))
                    with ui.HStack(height=28):
                        ui.Button('Run queue',clicked_fn=lambda:self.invoke(self.run_queue))
                        ui.Button('Patrol queue',clicked_fn=lambda:self.invoke(lambda:self.run_queue(True)))
                        ui.Button('Save mission',clicked_fn=lambda:self.invoke(self.save_mission))
                        ui.Button('Load mission',clicked_fn=lambda:self.invoke(self.load_mission))
                    self.queue_label=ui.Label('',height=30,word_wrap=True)
                    ui.Label('Map: click to select a goal. Green = route; red = robot.',height=20)
                    self.map_widget=ui.ImageWithProvider(self.provider,width=420,height=480,fill_policy=ui.IwpFillPolicy.IWP_STRETCH,
                        mouse_pressed_fn=self._click_map)
                    ui.Label('Known obstacle at goal X/Y (planning exclusion only):',height=20)
                    with ui.HStack(height=26):
                        ui.Button('Block radius 0.5 m',clicked_fn=lambda:self.invoke(lambda:self.set_known_obstacles(
                            self.known_obstacles+[(self.x_model.as_float,self.y_model.as_float,0.5)])))
                        ui.Button('Clear exclusions',clicked_fn=lambda:self.invoke(lambda:self.set_known_obstacles([])))

    def cancel(self):
        self.nav.cancel()
        if self.robot and self.robot.robot._articulation_view.is_physics_handle_valid(): self._write(0,0,0.005)
        self.status='Mission cancelled; wheels stopped'

    def estop(self):
        self.nav.estop()
        if self.robot and self.robot.robot._articulation_view.is_physics_handle_valid(): self._write(0,0,0.005)
        self._event('emergency_stop')

    def save_mission(self):
        payload={'map':str(MAP),'waypoints':[vars(g) for g in self.waypoints]}
        (ROOT/'navigation/mission.json').write_text(json.dumps(payload,indent=2))
        self.nav.message='Mission saved to navigation/mission.json'

    def load_mission(self):
        payload=json.loads((ROOT/'navigation/mission.json').read_text())
        if payload['map']!=str(MAP): raise ValueError('Mission map differs from current map')
        goals=[Goal(**g) for g in payload['waypoints']]
        if any(not self.grid.valid(g.x,g.y) or (g.yaw is not None and not math.isfinite(g.yaw)) for g in goals):
            raise ValueError('Mission contains an invalid waypoint')
        self.waypoints=goals
        self.nav.message=f'Loaded {len(goals)} waypoints'

    def _update(self,event):
        if self.closed: return
        if time.monotonic()-self.last_draw>0.3:
            self.last_draw=time.monotonic()
            self.draw()

    def draw(self):
        if not self.window: return
        self.status_label.text=(f'{self.nav.state}: {self.nav.message}\n'
            f'Pose ({self.pose.x:.2f}, {self.pose.y:.2f}) | v={self.nav.command[0]:.2f} '
            f'ω={self.nav.command[1]:.2f} | {self.status}')
        self.queue_label.text=f'Queue: {[(round(g.x,1),round(g.y,1)) for g in self.waypoints]} | reached {self.nav.completed}; laps {self.nav.laps}'
        data=np.frombuffer(self.grid.free,dtype=np.uint8).reshape(self.grid.height,self.grid.width)
        im=Image.fromarray(np.flipud(data)*200+25).convert('RGBA')
        draw=ImageDraw.Draw(im)
        def pixel(x,y):
            return ((x-self.grid.origin[0])/self.grid.resolution,
                    self.grid.height-(y-self.grid.origin[1])/self.grid.resolution)
        if len(self.nav.path)>1: draw.line([pixel(*p) for p in self.nav.path],fill='lime',width=2)
        for g in self.waypoints:
            x,y=pixel(g.x,g.y)
            draw.ellipse((x-2,y-2,x+2,y+2),fill='cyan')
        x,y=pixel(self.pose.x,self.pose.y)
        draw.ellipse((x-3,y-3,x+3,y+3),fill='red')
        draw.line((x,y,x+8*math.cos(self.pose.yaw),y-8*math.sin(self.pose.yaw)),fill='red',width=2)
        gx,gy=pixel(self.x_model.as_float,self.y_model.as_float)
        draw.ellipse((gx-3,gy-3,gx+3,gy+3),outline='orange',width=2)
        self.provider.set_bytes_data(list(im.tobytes()),[im.width,im.height])

    def close(self):
        self.cancel()
        self.closed=True
        if self.world and self.world.physics_callback_exists(CALLBACK): self.world.remove_physics_callback(CALLBACK)
        self.timeline_sub=self.update_sub=None
        if self.window: self.window.visible=False


async def launch():
    import builtins
    old=getattr(builtins,'warehouse_navigation',None)
    if old: old.close()
    app=ControlNavigationApp()
    if old and getattr(old,'stage',None)==app.stage and old.world is World.instance():
        app.robot=old.robot
    builtins.warehouse_navigation=app
    try:
        await app.start()
        print('Warehouse Control & Navigation ready. Access with builtins.warehouse_navigation')
    except Exception:
        app.close()
        raise
    return app
