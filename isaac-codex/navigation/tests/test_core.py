import math
import unittest
from control_nav.core import Config, Goal, Grid, Navigator, Pose, wrap


def grid(w=60,h=60):
    return Grid(w,h,0.2,(-6,-6),[1]*(w*h))


def simulate(nav,pose,steps=10000,dt=0.05):
    for _ in range(steps):
        v,w=nav.step(pose,dt)
        assert abs(v)<=nav.cfg.max_speed+1e-9
        assert abs(w)<=nav.cfg.max_turn+1e-9
        pose=Pose(pose.x+v*math.cos(pose.yaw)*dt,pose.y+v*math.sin(pose.yaw)*dt,wrap(pose.yaw+w*dt))
        if nav.state in ('SUCCEEDED','FAILED','ESTOP'):
            break
    return pose


class NavigationTests(unittest.TestCase):
    def test_goal_and_final_heading(self):
        nav=Navigator(grid())
        nav.navigate(Pose(0,0),[Goal(3,2,math.pi)])
        pose=simulate(nav,Pose(0,0))
        self.assertEqual(nav.state,'SUCCEEDED',nav.message)
        self.assertLess(math.hypot(pose.x-3,pose.y-2),0.16)
        self.assertLess(abs(wrap(pose.yaw-math.pi)),0.09)

    def test_wall_detour_and_collision_free_segments(self):
        g=grid()
        for y in range(12,48): g.free[y*g.width+30]=0
        nav=Navigator(g)
        nav.navigate(Pose(-2,0),[Goal(2,0)])
        self.assertTrue(all(g.line_clear(a,b) for a,b in zip(nav.path,nav.path[1:])))
        simulate(nav,Pose(-2,0))
        self.assertEqual(nav.state,'SUCCEEDED',nav.message)

    def test_no_diagonal_corner_cut(self):
        g=Grid(2,2,1,(0,0),[1,0,0,1])
        self.assertFalse(g.line_clear((0.5,0.5),(1.5,1.5)))
        with self.assertRaises(ValueError):g.plan((0.5,0.5),(1.5,1.5))

    def test_unreachable_and_invalid_goal(self):
        g=grid()
        for y in range(g.height):g.free[y*g.width+30]=0
        nav=Navigator(g)
        nav.navigate(Pose(-2,0),[Goal(2,0)])
        self.assertEqual(nav.state,'FAILED')
        with self.assertRaises(ValueError):nav.navigate(Pose(-2,0),[Goal(100,0)])
        self.assertEqual(nav.step(Pose(-2,0),0.05),(0,0))

    def test_waypoints(self):
        nav=Navigator(grid())
        nav.navigate(Pose(0,0),[Goal(2,0),Goal(2,2),Goal(0,2)])
        simulate(nav,Pose(0,0))
        self.assertEqual(nav.state,'SUCCEEDED',nav.message)
        self.assertEqual(nav.completed,3)

    def test_patrol(self):
        nav=Navigator(grid())
        nav.navigate(Pose(0,0),[Goal(1,0),Goal(0,0)],patrol=True)
        simulate(nav,Pose(0,0),steps=2500)
        self.assertEqual(nav.state,'NAVIGATING',nav.message)
        self.assertGreaterEqual(nav.laps,2)
        nav.cancel()
        self.assertEqual(nav.command,(0,0))

    def test_manual_timeout_and_speed(self):
        nav=Navigator(grid())
        nav.manual(100,100)
        for _ in range(12):nav.step(Pose(0,0),0.05)
        self.assertEqual(nav.state,'READY')
        self.assertEqual(nav.command,(0,0))

    def test_pause_freezes_timeout_resume_replans(self):
        nav=Navigator(grid())
        nav.navigate(Pose(0,0),[Goal(2,0)])
        nav.pause()
        for _ in range(10000):self.assertEqual(nav.step(Pose(0,0),0.05),(0,0))
        self.assertEqual(nav.time,0)
        nav.resume(Pose(0,0))
        simulate(nav,Pose(0,0))
        self.assertEqual(nav.state,'SUCCEEDED')

    def test_estop_latched(self):
        nav=Navigator(grid())
        nav.estop()
        nav.manual(1,0)
        nav.cancel()
        nav.step(Pose(float('nan'),0),0)
        self.assertEqual(nav.state,'ESTOP')
        with self.assertRaises(ValueError):nav.navigate(Pose(0,0),[Goal(1,0)])
        nav.clear_estop()
        self.assertEqual(nav.state,'READY')

    def test_known_obstacle_replan(self):
        g=grid()
        nav=Navigator(g)
        nav.navigate(Pose(-2,0),[Goal(2,0)])
        g.obstacles([(0,0,0.8)])
        nav.step(Pose(-2,0),0.05)
        self.assertGreater(len(nav.path),2)
        simulate(nav,Pose(-2,0))
        self.assertEqual(nav.state,'SUCCEEDED',nav.message)

    def test_stuck_robot_bounded_failure(self):
        nav=Navigator(grid(),Config(progress_timeout=0.5,max_replans=2))
        nav.navigate(Pose(0,0),[Goal(2,0)])
        for _ in range(60):nav.step(Pose(0,0),0.05)
        self.assertEqual(nav.state,'FAILED')
        self.assertEqual(nav.replans,2)

    def test_reverse_into_obstacle_stops(self):
        g=grid()
        g.obstacles([(-0.5,0,0.3)])
        nav=Navigator(g)
        nav.manual(-0.55,0,2)
        pose=simulate(nav,Pose(0,0),steps=50)
        self.assertEqual(nav.state,'FAILED')
        self.assertTrue(g.valid(pose.x,pose.y))


if __name__=='__main__':unittest.main()
