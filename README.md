# Humanoid Skill Sim

A Python project I built to explore humanoid robotics with behavior trees and skill development related to my interest in robotics, AI, and autonomous systems.

![Demo](assets/demo-r2d2.gif)

*Note: The GIF compression makes the visualization appear choppy - the actual simulation runs much smoother when viewed live or in PyBullet.*

## What it does

- Robot picks up 3 boxes from platforms and distributes them to 3 different bins
- Each task is controlled by behavior trees (hierarchical AI logic)
- Robot navigates around obstacles using path planning and collision avoidance
- Physics simulation handles realistic object manipulation and movement
- Tweakable settings for robot parameters, world setup, and task complexity

## Project structure

```
humanoid-skill-sim/
├── sim/
│   ├── robot.py               # humanoid robot simulation + movement
│   ├── world.py              # physics world setup with objects
│   └── teleop_override.py    # manual control override system
├── behavior_tree/
│   ├── node.py               # base behavior tree node classes
│   ├── sequence.py           # sequential task execution
│   ├── fallback.py           # backup behavior handling
│   └── action_nodes.py       # actual robot actions (walk, pick, place)
├── skills/
│   ├── walk.py               # walking and navigation logic
│   ├── pick.py               # object grasping and manipulation
│   ├── place.py              # object placement logic
│   └── custom_workflow.py    # complex composed behaviors
├── configs/
│   └── workflows.yaml        # different task configurations
├── tests/                    # various testing scripts
├── logs/                     # simulation run data and logs
├── assets/                   # demo gifs and media
├── run_sim.py               # main simulation runner
└── README.md                # you are here
```

## Building and Running

```bash
# install dependencies
pip install -r requirements.txt

# run the main demo (distribute 3 boxes to 3 bins)
python run_sim.py --workflow distribute_boxes --gui

# run without visualization (faster)
python run_sim.py --workflow distribute_boxes --no-gui

# run simple walking test
python run_sim.py --workflow simple_walk --gui
```

For debugging or development:
```bash
# run with detailed logging
python run_sim.py --workflow distribute_boxes --verbose --export-logs

# run interactive mode for manual control
python run_interactive.py
```

## How to use it

The robot shows up as a humanoid figure, boxes are on platforms, and bins are the target locations. The main workflow demonstrates the robot picking up box_01, box_02, and box_03 from their platforms and placing each one in a different bin.

Edit `configs/workflows.yaml` to change robot behavior, object positions, target locations, etc.

## The technical bits

### Behavior Trees
Using behavior trees for robust task control:
- Sequence nodes for step-by-step execution
- Fallback nodes for error recovery
- Action nodes for basic robot skills
- Easy to compose complex behaviors from simple building blocks

### Physics Simulation
PyBullet handles the physics and collision detection:
- Realistic robot movement and object interactions
- Collision avoidance with dynamic path planning
- Gravity and momentum for realistic object placement
- URDF robot models or built-in simplified robots

### Robot Skills
Modular skill system:
- Walk: Navigation with obstacle avoidance and multi-waypoint routing
- Pick: Object grasping with strategic positioning and constraint-based attachment
- Place: Precise object placement with physics-based dropping
- Each skill handles its own error conditions and recovery

### Multi-Agent Coordination
No central coordinator - robot reacts to environment changes and plans dynamically around obstacles.

## Recent Updates
Been improving the simulation based on testing:
- Enhanced collision avoidance with smart detour routing strategies
- Better object attachment during navigation to prevent dropping
- Improved strategic positioning for optimal manipulation reach
- Added multi-waypoint navigation for complex obstacle courses
- More robust constraint management for held objects during movement
- Better error recovery and fallback behaviors

The workflow system is flexible - you can define complex task sequences in YAML without touching the Python code.

## Lessons learned

### What went well:
- Behavior trees worked great for complex task sequencing
- PyBullet physics simulation was surprisingly realistic
- Modular skill design made debugging much easier
- Configuration-driven workflows are very flexible

### What was trickier than expected:
- Object attachment during robot movement - objects would lag behind or detach
- Multi-waypoint navigation around complex obstacle layouts
- Balancing realistic physics with reliable task execution
- Strategic robot positioning for optimal manipulation reach

### If I did this again:
- Would start with simpler object manipulation before complex navigation
- Add proper unit tests for each skill early on
- Better visualization of robot intentions and planned paths
- Should have implemented logging system from the beginning

### Weird issues I ran into:
- Objects would sometimes "lag" behind the robot during navigation despite constraints
- Robot would occasionally get stuck in infinite planning loops with certain obstacle configurations
- PyBullet constraint forces needed constant tuning for different scenarios
- Strategic positioning sometimes resulted in suboptimal robot orientations

The biggest insight was that "simple" manipulation tasks become complex when you add realistic physics and navigation constraints. Even basic fetch-and-place requires sophisticated coordination between movement, grasping, and spatial reasoning.

## TODO

- Machine learning for adaptive grasping based on object properties
- More sophisticated path planning algorithms (RRT*, etc.)
- Multi-robot coordination and task sharing
- Real robot deployment testing (sim-to-real transfer)
- Better performance metrics and success rate tracking
