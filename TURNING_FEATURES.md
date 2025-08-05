# Robot Turning and Orientation Features

## Overview

The humanoid robot simulation now includes realistic turning behavior that makes the robot orient itself optimally for walking and manipulation tasks. This makes the simulation more realistic and improves manipulation success rates.

## New Features

### 1. Robot Orientation Tracking
- The robot now tracks its current orientation (yaw angle) in radians
- Orientation is updated during turns and maintained during other actions
- PyBullet physics integration for realistic orientation changes

### 2. Turn Actions

#### `TurnToFace` Action Node
Turn the robot to face a specific position or angle:

```python
# Turn to face a specific position
TurnToFace(target=[2.0, 1.0, 0.0])

# Turn to a specific angle (in radians)
TurnToFace(angle=1.57)  # 90 degrees
```

#### Enhanced `WalkToTarget`
Now includes optional turning before walking:

```python
# Turn to face direction before walking (default)
WalkToTarget(target=[2.0, 1.0, 0.0], turn_first=True)

# Walk without turning first
WalkToTarget(target=[2.0, 1.0, 0.0], turn_first=False)
```

#### Enhanced `PickObject`
Automatically turns to face objects for optimal manipulation:

```python
# Robot will:
# 1. Navigate to object
# 2. Turn to face object
# 3. Attempt pick
PickObject(object_id="box_01")
```

#### Enhanced `PlaceObject`
Automatically turns to face placement locations:

```python
# Robot will:
# 1. Turn to face placement location
# 2. Attempt place
PlaceObject(location=[2.0, 1.0, 0.1])
```

## Benefits of Turning

### 1. Realistic Movement
- Humanoid robots naturally turn before walking forward
- Eliminates unrealistic sideways sliding motion
- More natural bipedal locomotion patterns

### 2. Improved Manipulation
- Gripper faces objects directly for better grasping
- Arms don't have to reach awkwardly across the body
- Higher success rates for pick and place operations

### 3. Better Spatial Awareness
- Robot maintains consistent forward-facing orientation
- Easier to predict robot behavior and motion
- More intuitive human-robot interaction

## Configuration Options

### Turn Speed
Adjust how fast the robot turns (in robot.py):
```python
self.turn_speed = 2.0  # radians per second
```

### Turn Tolerance
Adjust how precisely the robot needs to face the target:
```python
# In turning logic (currently 0.05 radians ≈ 3 degrees)
if abs(angle_diff) < 0.05:
    # Turn complete
```

## Example Workflows

### Basic Turning Demo
```yaml
turning_demo:
  root: "Sequence"
  nodes:
    - type: "TurnToFace"
      parameters:
        target: [2.0, 0.0, 0.0]
    - type: "WalkToTarget"
      parameters:
        target: [2.0, 0.0, 0.0]
        turn_first: false
```

### Precise Manipulation
```yaml
precise_manipulation:
  root: "Sequence"
  nodes:
    - type: "TurnToFace"
      parameters:
        target: [1.5, 0.8, 0.0]  # Face object
    - type: "PickObject"
      parameters:
        object_id: "box_01"
    - type: "TurnToFace"
      parameters:
        target: [2.5, 1.2, 0.0]  # Face target
    - type: "PlaceObject"
      parameters:
        location: [2.5, 1.2, 0.1]
```

## Testing

Run the turning behavior test:
```bash
python test_turning.py
```

This demonstrates the robot turning to face different positions and angles, showing the smooth progression of orientation changes.

## Implementation Details

### Robot Class Changes
- Added `current_orientation`, `target_orientation`, `turning` state
- Added `turn_speed` parameter for turn rate control
- Added turning methods: `turn_to_face()`, `turn_to_angle()`, `is_turning()`
- Added angle normalization for proper angle handling

### Action Node Changes
- `TurnToFace`: New action node for explicit turning
- `WalkToTarget`: Optional turning phase before walking
- `PickObject`: Automatic orientation toward objects
- `PlaceObject`: Automatic orientation toward placement locations

### Physics Integration
- Uses PyBullet's `resetBasePositionAndOrientation()` for smooth updates
- Quaternion conversion for proper 3D orientation
- Held objects follow robot orientation changes
