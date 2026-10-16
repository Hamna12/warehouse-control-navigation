"""Simulator-independent differential-drive navigation. No sensing or ROS required."""
from dataclasses import dataclass
import heapq
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float | None = None


@dataclass(frozen=True)
class Config:
    max_speed: float = 0.55
    max_turn: float = 0.8
    acceleration: float = 0.45
    angular_acceleration: float = 1.5
    position_tolerance: float = 0.15
    yaw_tolerance: float = 0.08
    progress_timeout: float = 12.0
    max_replans: int = 2
    goal_timeout: float = 240.0


class Grid:
    """Bottom-up occupancy grid, already inflated for the robot footprint."""
    def __init__(self, width, height, resolution, origin, free):
        if width <= 0 or height <= 0 or resolution <= 0 or len(free) != width * height:
            raise ValueError('Invalid occupancy grid')
        self.width, self.height = width, height
        self.resolution, self.origin = resolution, tuple(origin)
        self.base = bytearray(free)
        self.free = bytearray(free)
        self.version = 0

    def cell(self, x, y):
        return (math.floor((x-self.origin[0])/self.resolution),
                math.floor((y-self.origin[1])/self.resolution))

    def point(self, cell):
        return (self.origin[0]+(cell[0]+0.5)*self.resolution,
                self.origin[1]+(cell[1]+0.5)*self.resolution)

    def clear(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and bool(self.free[y*self.width+x])

    def valid(self, x, y):
        return math.isfinite(x) and math.isfinite(y) and self.clear(self.cell(x, y))

    def line_clear(self, a, b):
        """Supercover traversal: diagonal corner touching is treated as collision."""
        x, y = self.cell(*a)
        end = self.cell(*b)
        if not self.clear((x,y)) or not self.clear(end):
            return False
        dx, dy = b[0]-a[0], b[1]-a[1]
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        r = self.resolution
        tx = ((self.origin[0]+(x+(sx>0))*r-a[0])/dx) if dx else math.inf
        ty = ((self.origin[1]+(y+(sy>0))*r-a[1])/dy) if dy else math.inf
        dtx, dty = (r/abs(dx) if dx else math.inf), (r/abs(dy) if dy else math.inf)
        while (x,y) != end:
            if abs(tx-ty) < 1e-10:
                if not self.clear((x+sx,y)) or not self.clear((x,y+sy)):
                    return False
                x, y, tx, ty = x+sx, y+sy, tx+dtx, ty+dty
            elif tx < ty:
                x, tx = x+sx, tx+dtx
            else:
                y, ty = y+sy, ty+dty
            if not self.clear((x,y)):
                return False
        return True

    def obstacles(self, circles):
        """Replace known obstacles. Circle radii must INCLUDE robot clearance."""
        circles = list(circles)
        if any(len(c)!=3 or not all(math.isfinite(v) for v in c) or c[2]<0 for c in circles):
            raise ValueError('Obstacle must be finite x, y, nonnegative radius')
        self.free = bytearray(self.base)
        for cx,cy,radius in circles:
            lo = self.cell(cx-radius, cy-radius)
            hi = self.cell(cx+radius, cy+radius)
            for y in range(max(0,lo[1]), min(self.height,hi[1]+1)):
                for x in range(max(0,lo[0]), min(self.width,hi[0]+1)):
                    px,py = self.point((x,y))
                    if math.hypot(px-cx,py-cy) <= radius+self.resolution*0.7072:
                        self.free[y*self.width+x] = 0
        self.version += 1

    def plan(self, start, goal):
        if not self.valid(*start):
            raise ValueError('Robot is outside navigable map clearance')
        if not self.valid(*goal):
            raise ValueError('Goal is occupied, unknown, outside map, or too close to an obstacle')
        if self.line_clear(start,goal):
            return [start,goal]
        source, target = self.cell(*start), self.cell(*goal)
        queue = [(math.dist(source,target),0.0,source)]
        costs, parents = {source:0.0}, {}
        moves = [(x,y) for x in (-1,0,1) for y in (-1,0,1) if x or y]
        while queue:
            _, cost, node = heapq.heappop(queue)
            if cost > costs[node]:
                continue
            if node == target:
                cells = [node]
                while node != source:
                    node = parents[node]
                    cells.append(node)
                points = [start]+[self.point(c) for c in reversed(cells)]+[goal]
                # Greedy visibility simplification, preserving clearance at corners.
                out, i = [points[0]], 0
                while i < len(points)-1:
                    j = len(points)-1
                    while j>i+1 and not self.line_clear(points[i],points[j]):
                        j -= 1
                    out.append(points[j])
                    i = j
                return out
            for dx,dy in moves:
                nxt = (node[0]+dx,node[1]+dy)
                if not self.clear(nxt):
                    continue
                if dx and dy and (not self.clear((node[0]+dx,node[1])) or not self.clear((node[0],node[1]+dy))):
                    continue
                nc = cost+math.hypot(dx,dy)
                if nc < costs.get(nxt,math.inf):
                    costs[nxt],parents[nxt] = nc,node
                    heapq.heappush(queue,(nc+math.dist(nxt,target),nc,nxt))
        raise ValueError('No route to goal in the supplied map')


class Navigator:
    def __init__(self, grid, config=Config()):
        self.grid, self.cfg = grid, config
        self.state, self.message = 'READY','Choose a goal or manual command'
        self.path, self.goals = [],[]
        self.index, self.completed, self.laps = 0,0,0
        self.patrol = False
        self.command = (0.0,0.0)
        self.time, self.distance = 0.0,0.0
        self.previous_pose = None
        self.replans = 0
        self.map_version = grid.version

    def stop(self, state='READY', message='Stopped'):
        self.state,self.message,self.command = state,message,(0.0,0.0)
        return self.command

    def cancel(self):
        self.goals,self.path = [],[]
        if self.state == 'ESTOP':
            return self.stop('ESTOP','Emergency stop latched; clear it explicitly')
        return self.stop(message='Mission cancelled')

    def estop(self):
        return self.stop('ESTOP','Emergency stop latched')

    def clear_estop(self):
        self.goals,self.path = [],[]
        return self.stop(message='Emergency stop cleared; choose a new command')

    def pause(self):
        if self.state in ('NAVIGATING','MANUAL'):
            self.resume_state = self.state
            self.stop('PAUSED','Paused')

    def resume(self,pose):
        if self.state == 'PAUSED':
            if self.resume_state == 'NAVIGATING':
                self._plan(pose)
            else:
                self.stop(message='Manual command expired; choose a new command')

    def manual(self,v,w,duration=0.5):
        if self.state == 'ESTOP':
            return
        if not all(math.isfinite(a) for a in (v,w,duration)) or duration<=0 or duration>2:
            raise ValueError('Manual command requires finite values and duration in (0,2] seconds')
        self.goals,self.path = [],[]
        self.manual_command = (max(-self.cfg.max_speed,min(self.cfg.max_speed,v)),
                               max(-self.cfg.max_turn,min(self.cfg.max_turn,w)))
        self.manual_until = self.time+duration
        self.state,self.message = 'MANUAL','Timed manual command'

    def navigate(self,pose,goals,patrol=False):
        if self.state == 'ESTOP':
            raise ValueError('Clear emergency stop before starting')
        goals = list(goals)
        if not goals or any(not all(math.isfinite(v) for v in (g.x,g.y)) or
                            (g.yaw is not None and not math.isfinite(g.yaw)) for g in goals):
            raise ValueError('Provide at least one finite goal')
        # Validate entire mission before accepting it.
        for g in goals:
            if not self.grid.valid(g.x,g.y):
                raise ValueError('A waypoint is outside navigable map clearance')
        self.goals,self.patrol,self.index,self.completed,self.laps = goals,patrol,0,0,0
        self.distance,self.previous_pose = 0.0,pose
        self.replans = 0
        self.goal_started = self.time
        self._plan(pose)

    def _plan(self,pose):
        self.command = (0.0,0.0)
        try:
            g = self.goals[self.index]
            self.path = self.grid.plan((pose.x,pose.y),(g.x,g.y))
            self.cursor = 1
            self.progress_pose,self.progress_yaw = pose,pose.yaw
            self.progress_time = self.time
            self.map_version = self.grid.version
            self.state,self.message = 'NAVIGATING',f'Waypoint {self.index+1}/{len(self.goals)}'
        except ValueError as e:
            self.path = []
            self.stop('FAILED',str(e))

    def _safe(self,pose,v,w):
        if not self.grid.valid(pose.x,pose.y):
            return False
        # Predict commanded arc through braking horizon on the inflated map.
        horizon = 0.35+abs(v)/self.cfg.acceleration
        n = max(8,math.ceil(abs(v)*horizon/(self.grid.resolution*0.4)))
        prev = (pose.x,pose.y)
        for i in range(1,n+1):
            t = horizon*i/n
            if abs(w)<1e-7:
                xy=(pose.x+v*t*math.cos(pose.yaw),pose.y+v*t*math.sin(pose.yaw))
            else:
                xy=(pose.x+v/w*(math.sin(pose.yaw+w*t)-math.sin(pose.yaw)),
                    pose.y-v/w*(math.cos(pose.yaw+w*t)-math.cos(pose.yaw)))
            if not self.grid.line_clear(prev,xy):
                return False
            prev = xy
        return True

    def step(self,pose,dt):
        if self.state == 'ESTOP':
            return (0.0,0.0)
        if not math.isfinite(dt) or dt<=0 or dt>0.25 or not all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw)):
            return self.stop('FAILED','Invalid pose or control timestep')
        if self.state not in ('MANUAL','NAVIGATING'):
            self.previous_pose = pose
            return (0.0,0.0)
        self.time += dt
        if self.previous_pose:
            self.distance += math.hypot(pose.x-self.previous_pose.x,pose.y-self.previous_pose.y)
        self.previous_pose = pose
        if self.state == 'MANUAL':
            if self.time>=self.manual_until:
                return self.stop(message='Manual command timeout: stopped')
            desired = self.manual_command
        else:
            if self.time-self.goal_started > self.cfg.goal_timeout:
                return self.stop('FAILED','Goal time limit exceeded')
            if self.map_version != self.grid.version:
                self._plan(pose)
                if self.state != 'NAVIGATING': return self.command
            g=self.goals[self.index]
            distance=math.hypot(g.x-pose.x,g.y-pose.y)
            if distance <= self.cfg.position_tolerance:
                error=0.0 if g.yaw is None else wrap(g.yaw-pose.yaw)
                if abs(error)<=self.cfg.yaw_tolerance:
                    self.completed += 1
                    self.index += 1
                    if self.index == len(self.goals):
                        if not self.patrol:
                            return self.stop('SUCCEEDED','All waypoints reached')
                        self.laps += 1
                        self.index = 0
                    self.goal_started,self.replans = self.time,0
                    self._plan(pose)
                    return (0.0,0.0)
                desired=(0.0,max(-self.cfg.max_turn,min(self.cfg.max_turn,2.0*error)))
            else:
                while self.cursor<len(self.path)-1 and math.dist((pose.x,pose.y),self.path[self.cursor])<0.04 and self.grid.line_clear((pose.x,pose.y),self.path[self.cursor+1]):
                    self.cursor += 1
                # Select a short visible target on the current path segment.
                target=self.path[self.cursor]
                dx,dy=target[0]-pose.x,target[1]-pose.y
                angle=wrap(math.atan2(dy,dx)-pose.yaw)
                turn=max(-self.cfg.max_turn,min(self.cfg.max_turn,2.0*angle))
                speed=min(self.cfg.max_speed,0.8*distance,0.8*math.hypot(dx,dy))
                speed *= max(0.0,math.cos(angle))**2
                if abs(angle)>0.6: speed=0.0
                desired=(speed,turn)
            moved=math.hypot(pose.x-self.progress_pose.x,pose.y-self.progress_pose.y)>0.08
            turned=abs(wrap(pose.yaw-self.progress_yaw))>0.15
            if moved or turned:
                self.progress_pose,self.progress_yaw,self.progress_time=pose,pose.yaw,self.time
            elif self.time-self.progress_time>self.cfg.progress_timeout:
                if self.replans>=self.cfg.max_replans:
                    return self.stop('FAILED','No progress after bounded replanning attempts')
                self.replans += 1
                self._plan(pose)
                return (0.0,0.0)
        v,w=self.command
        dv,dw=self.cfg.acceleration*dt,self.cfg.angular_acceleration*dt
        command=(v+max(-dv,min(dv,desired[0]-v)),w+max(-dw,min(dw,desired[1]-w)))
        if not self._safe(pose,*command):
            if self.state=='MANUAL':
                return self.stop('FAILED','Manual motion blocked by supplied map')
            # Stop translation immediately; rotation is safe for the inflated circular footprint.
            command=(0.0,command[1])
            if not self._safe(pose,*command):
                return self.stop('FAILED','Robot is outside navigable map clearance')
        self.command=command
        return command
