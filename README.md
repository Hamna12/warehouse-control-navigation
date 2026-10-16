# Warehouse Robot Control & Navigation

A warehouse robot navigation prototype developed using
NVIDIA Isaac Sim 5.1.0, Python, and MobilityGen-related components.

## Project Overview

This project implements a custom Control & Navigation system
for a Carter robot inside a simulated warehouse environment.

The robot uses a known occupancy map to plan routes and navigate
between warehouse racks.

## Development Approach

### 1. Initial Robot Development

Initially, a custom robot was developed and tested with
basic movement and control logic inside NVIDIA Isaac Sim.

### 2. MobilityGen Integration

The implementation was later transitioned to a Carter robot
setup using NVIDIA MobilityGen-related tools and workflows.

The robot was integrated into the warehouse environment.

### 3. ROS 2 Nav2 Experiment

We explored ROS 2 Nav2 following NVIDIA's official tutorial:

https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html

During integration, we encountered:
- RViz2 freezing and performance issues
- Simulation clock synchronization problems
- Resource limitations when running Isaac Sim and RViz2 together

The ROS 2 navigation workflow was not fully validated.

### 4. Current Navigation Implementation

We implemented a custom Python-based navigation system
directly within Isaac Sim.

The system includes:
- Warehouse occupancy map
- A*-based path planning
- Goal-based navigation
- Linear and angular velocity control
- Waypoint navigation
- Navigation status monitoring
- Pause, resume, and emergency-stop controls
- Mission saving and loading

## Demonstrated Results

- Successful short-distance waypoint navigation
- Successful longer-distance navigation
- Route planning and movement between warehouse racks
- Live robot position and navigation visualization

## Navigation Workflow

Goal Selection
    ↓
Occupancy Map
    ↓
A* Path Planning
    ↓
Motion Controller
    ↓
Robot Movement
    ↓
Position Feedback

## Technology Stack

- NVIDIA Isaac Sim 5.1.0
- NVIDIA MobilityGen-related components
- Python
- Carter Robot
- A* Path Planning
- Occupancy Map

## Project Structure

- `isaac-codex/` — Python implementation and navigation assets
- `Team_Warehouse_MobilityGen.usd` — Isaac Sim warehouse scene

## Running the Project

1. Install and launch NVIDIA Isaac Sim 5.1.0.
2. Open `Team_Warehouse_MobilityGen.usd` in Isaac Sim.
3. Set up the Python dependencies required by the project.
4. Launch the navigation controller using the project scripts.

The current implementation may require adjusting local file paths
and asset references when running on another computer.

## Limitations

The current system uses a known occupancy map and simulator-provided
robot position.

Sensor-based localization, dynamic obstacle avoidance, and
full ROS 2 Nav2 integration are not part of the validated implementation.
