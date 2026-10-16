# Warehouse control and navigation — no perception work

This project commands the existing Carter robot in Isaac Sim using the supplied
warehouse occupancy map and the robot's **ground-truth simulator pose**. It does
not require lidar processing, camera perception, SLAM, AMCL, ROS, RViz, or Nav2.

The main implementation is `control_nav/core.py` (planner, controller and mission
state machine), with `control_nav/isaac_app.py` connecting it to Isaac's wheels
and providing a live map beside the viewport.

## Start after a restart

1. Open `/home/hmna/Downloads/Team_Warehouse_MobilityGen.usd` in Isaac Sim.
2. Enable the MobilityGen core/examples extensions and the VS Code executor.
   You do **not** need to fill in MobilityGen's fields or press Build.
3. From a desktop terminal:

   ```bash
   cd /home/hmna/Downloads/isaac-codex
   .venv/bin/python navigation/run_control_navigation.py
   ```

   Alternatively, run this in Isaac's Script Editor:

   ```python
   exec(open('/home/hmna/Downloads/isaac-codex/navigation/start_control_navigation.py').read())
   ```

4. Wait for **Warehouse Control & Navigation** to show READY. Startup initializes
   physics, sets the chase camera's pose absolutely, removes MobilityGen's random
   driving callback, and starts simulation with zero wheel commands. Physics
   initialization restores the articulation's default pose; it does not preserve
   the robot's location from a previously running mission.

Re-running the launcher stops the old controller and replaces its panel. It does
not spawn a second robot. The source warehouse USD is not overwritten. The source
scene must contain Carter at `/World/robot/chassis_link`; this is not an arbitrary
robot launcher.

## Operate the panel

- **Go to goal:** click the map or enter world X/Y coordinates in meters, then
  click Go to goal. Optionally enable Yaw and enter the final heading in degrees.
- **Manual:** Forward/Reverse/Left/Right commands each last 0.5 simulation seconds.
  The command expires automatically. Manual driving cancels the active mission.
- **Waypoints:** select each goal and click Add waypoint. Run queue executes them
  once; Patrol queue repeats them. Clear queue edits the pending list, not an
  already running mission. Cancel stops the active mission.
- **Pause / Resume:** stop and continue a mission. Resume replans from the current
  pose. Simulation pauses do not consume the mission timeout.
- **E-STOP:** immediately commands zero wheel velocity and latches the stop.
  Clear E-stop discards the mission and enables new commands; it never resumes
  movement by itself. Zero command is not an instantaneous physical brake or a
  real-robot safety system.
- **Timeline Stop → Play:** automatically attempts to restore joint handles once.
  Stop cancels the mission; after recovery, use Run queue or Go to goal.
  Timeline Pause → Play resumes a paused navigation mission.
- **Reinitialize physics:** manually retries recovery if automatic recovery fails.
  It cancels the mission and restores the default robot pose.
- **Save/Load mission:** writes/reads `navigation/mission.json` in world coordinates.
- **Known obstacle:** adds a 0.5 m radius *planning exclusion* at the selected X/Y.
  It does not spawn an obstacle in Isaac or detect anything. It triggers route
  replanning; an impossible route fails stopped. Clear exclusions removes these
  operator-provided exclusions. Set a new goal to retry a failed mission.

The map shows inflated navigable space, the robot in red, the route in green,
queued waypoints in cyan, and the selected goal in orange. Obstacles and areas
too close to them appear dark. It is a control interface within Isaac, not RViz.

## Implemented navigation logic

1. Read the existing ROS-format occupancy map. Unknown cells are blocked.
2. Inflate obstacles by a configurable circular footprint clearance (0.65 m
   default), plus a pixel margin, and conservatively reduce the planning grid
   to 0.20 m cells. This trades some narrow-passage access for clearance.
3. Use eight-connected A* without diagonal corner cutting, followed by
   visibility-based path simplification.
4. Follow successive path points with bounded linear/angular velocity and
   acceleration, slowing near corners and the destination. Rotate in place
   for large heading errors. Optionally align the final yaw.
5. Check a projected motion arc through a braking horizon against the inflated
   map. Reject unsafe translations, reverse motion into mapped obstacles, invalid
   goals and out-of-map poses.
6. Replan when the supplied obstacle map changes. Detect lack of progress,
   perform at most two replans, then fail stopped. Each goal has a 240 s active
   simulation-time limit. Failures never teleport the robot or silently choose
   a different goal.
7. Manage single goals, waypoint sequences, repeated patrols, pause/resume,
   cancellation, emergency stop and timed manual commands.

Default limits are 0.55 m/s, 0.8 rad/s, 0.45 m/s² and 1.5 rad/s². Goal tolerances
are 0.15 m and 0.08 rad. Edit the `Config` dataclass in `core.py` to tune them.
The map clearance is set in `load_grid()` in `isaac_app.py`; validate it against
the actual robot's footprint before reducing it.

## Supplied obstacle interface

For a scenario with known moving obstacles, an external scenario controller can
provide current obstacle coordinates; no perception implementation is needed:

```python
import builtins
app = builtins.warehouse_navigation
app.set_known_obstacles([(x_world, y_world, physical_radius_m)])
# Later, replace the list with updated positions or [] to remove them.
```

This is reactive replanning from supplied coordinates, not prediction of obstacle
motion. Unreported obstacles are invisible to this controller. The existing map
was derived from conservative collider bounds, so map/scene accuracy remains an
input assumption. No physical collision-avoidance guarantee is claimed.

## Tests and evidence

Run the simulator-independent tests:

```bash
cd /home/hmna/Downloads/isaac-codex
PYTHONPATH=navigation /usr/bin/python3 -m unittest discover -s navigation/tests -v
```

The tests exercise goal and heading arrival, obstacle detours, corner clearance,
unreachable goals, multi-waypoint navigation, patrol, manual timeout, pause/resume,
latched emergency stop, map-change replanning, bounded stuck recovery, and reverse
collision checks. These use an ideal differential-drive model, not Isaac physics.

Live Isaac validation and state transitions are written to:

- `logs/live_validation.json`: successful single-goal and two-waypoint runs,
  emergency-stop checks, startup without a MobilityGen scenario, map click
  conversion, and successful physics recovery after timeline Stop.
- `logs/control_navigation.jsonl`: timestamped mission state changes, pose,
  distance, completed waypoints and patrol laps.

Only tests explicitly recorded as live tests should be described as validated in
Isaac. Narrow aisles, moving physical obstacles and all warehouse routes still
require scenario-by-scenario physics testing.

Validation completed in this session: all 12 automated tests passed; Carter
reached the live single goal within 0.143 m and completed both live waypoints.
After timeline Stop invalidated the articulation handle, Reinitialize physics
restored it while preserving the emergency-stop latch. The controller was left
READY with zero commanded velocity. A full operating-system restart was not tested.

## Presentation script

“My work focuses on control and navigation of Carter in a simulated warehouse.
The controller receives an existing map and the true robot pose from Isaac Sim;
perception and localization are supplied inputs, outside my task. I implemented
map-based path planning, wheel velocity control, goal and heading tracking,
waypoint missions, patrol, manual commands, pause/resume, cancellation and an
emergency stop. The controller checks mapped clearance, replans when known
obstacles change, and reports failure when it cannot make progress. A live map
shows the robot and its planned route beside the simulation. Automated tests
cover the controller logic, and live Isaac tests verify integration with Carter.”

The earlier `view_map.sh` and `warehouse_map.rviz` are optional ROS map-viewer
files from the previous approach. They are not used by this control-only setup.
