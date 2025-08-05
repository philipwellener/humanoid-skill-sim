"""
Action nodes for specific skills in the behavior tree
"""

from typing import Any, List, Optional, Dict
import time
import random
import numpy as np
try:
    from .node import Node, NodeStatus
except ImportError:
    from node import Node, NodeStatus


class ActionNode(Node):
    """Base class for action nodes that execute specific skills"""
    
    def __init__(self, name: str = None, **kwargs):
        super().__init__(name)
        self.parameters = kwargs
        self.robot = None  # Will be set by context
        self.world = None  # Will be set by context
        
    def setup_from_context(self, context: Any) -> None:
        """Extract robot and world from context"""
        if context:
            if isinstance(context, dict):
                self.robot = context.get('robot', None)
                self.world = context.get('world', None)
            else:
                self.robot = getattr(context, 'robot', None)
                self.world = getattr(context, 'world', None)


class TurnToFace(ActionNode):
    """Action node for turning to face a target position or direction"""
    
    def __init__(self, target: List[float] = None, angle: float = None, tolerance: float = 0.05, **kwargs):
        super().__init__(**kwargs)
        
        if target is None and angle is None:
            raise ValueError("Must specify either target position or angle")
        if target is not None and angle is not None:
            raise ValueError("Cannot specify both target position and angle")
            
        self.target = target
        self.angle = angle
        self.tolerance = tolerance
        self.turning = False
        
    def tick(self, context: Any = None) -> NodeStatus:
        """Execute turning behavior"""
        self.setup_from_context(context)
        
        if self.start_time is None:
            self._start_execution()
            
        if not self.robot:
            return self._end_execution(NodeStatus.FAILURE)
            
        # Start turning if not already turning
        if not self.turning:
            if self.target is not None:
                self.robot.turn_to_face(self.target)
            else:
                self.robot.turn_to_angle(self.angle)
            self.turning = True
            
        # Check if still turning
        if self.robot.is_turning():
            return NodeStatus.RUNNING
        else:
            # Turning completed
            return self._end_execution(NodeStatus.SUCCESS)
                
    def reset(self) -> None:
        """Reset turning state"""
        super().reset()
        self.turning = False


class WalkToTarget(ActionNode):
    """Action node for walking to a target location"""
    
    def __init__(self, target: List[float], tolerance: float = 0.1, turn_first: bool = True, **kwargs):
        super().__init__(**kwargs)
        
        # Validate target parameter
        if not isinstance(target, (list, tuple)) or len(target) < 2:
            raise ValueError("Target must be a list or tuple with at least 2 coordinates")
        if not all(isinstance(x, (int, float)) for x in target[:2]):
            raise TypeError("Target coordinates must be numeric")
            
        self.target = target
        self.tolerance = tolerance
        self.turn_first = turn_first
        self.walking = False
        self.turning = False
        self.turn_complete = False
        
    def tick(self, context: Any = None) -> NodeStatus:
        """Execute walking behavior with optional turning"""
        self.setup_from_context(context)
        
        if self.start_time is None:
            self._start_execution()
            
        if not self.robot:
            return self._end_execution(NodeStatus.FAILURE)
            
        # Get current position
        current_pos = self.robot.get_position()
        
        # Calculate distance to target
        distance = np.linalg.norm(np.array(self.target[:2]) - np.array(current_pos[:2]))
        
        if distance <= self.tolerance:
            # Reached target
            return self._end_execution(NodeStatus.SUCCESS)
        
        # Step 1: Turn to face target (if enabled and not already done)
        if self.turn_first and not self.turn_complete:
            if not self.turning:
                self.robot.turn_to_face(self.target)
                self.turning = True
                
            if self.robot.is_turning():
                return NodeStatus.RUNNING
            else:
                self.turn_complete = True
                self.turning = False
        
        # Step 2: Walk to target
        if not self.walking:
            self.robot.start_walking_to(self.target)
            self.walking = True
            
        # Check if still walking
        if self.robot.is_walking():
            return NodeStatus.RUNNING
        else:
            # Walking completed, check if we reached target
            final_pos = self.robot.get_position()
            final_distance = np.linalg.norm(np.array(self.target[:2]) - np.array(final_pos[:2]))
            
            if final_distance <= self.tolerance:
                return self._end_execution(NodeStatus.SUCCESS)
            else:
                return self._end_execution(NodeStatus.FAILURE)
                
    def reset(self) -> None:
        """Reset walking state"""
        super().reset()
        self.walking = False
        self.turning = False
        self.turn_complete = False


class PickObject(ActionNode):
    """Action node for picking up an object"""
    
    def __init__(self, object_id: str, approach_distance: float = 0.05, **kwargs):
        super().__init__(**kwargs)
        self.object_id = object_id
        self.approach_distance = approach_distance
        self.picking = False
        self.turning = False
        self.pick_attempt = 0
        self.max_attempts = 3
        self.navigation_complete = False
        self.orientation_complete = False
        self.navigation_attempt = 0
        self.max_navigation_attempts = 5
        
    def tick(self, context: Any = None) -> NodeStatus:
        """Execute picking behavior"""
        self.setup_from_context(context)
        
        if self.start_time is None:
            self._start_execution()
            
        if not self.robot or not self.world:
            return self._end_execution(NodeStatus.FAILURE)
            
        # Get object position
        obj_pos = self.world.get_object_position(self.object_id)
        if obj_pos is None:
            return self._end_execution(NodeStatus.FAILURE)
            
        # Step 1: Navigate to object if not already there
        if not self.navigation_complete:
            robot_pos = self.robot.get_position()
            distance = np.linalg.norm(np.array(obj_pos[:2]) - np.array(robot_pos[:2]))
            
            # Check if robot is in good position for picking
            print(f"  📏 Current distance to object: {distance:.3f}m (robot at {robot_pos[:2]}, object at {obj_pos[:2]})")
            if 0.30 <= distance <= 0.60:  # Expanded range for better positioning
                self.navigation_complete = True
                print(f"  ✅ Robot in good position for picking (distance: {distance:.3f}m)")
            else:
                # Check if robot is currently walking
                if self.robot.is_walking():
                    print(f"  🚶 Robot is walking, waiting for movement to complete...")
                    return NodeStatus.RUNNING
                
                # Need to navigate to better position
                self.navigation_attempt += 1
                if self.navigation_attempt > self.max_navigation_attempts:
                    print(f"  ❌ Too many navigation attempts, proceeding with pick anyway")
                    self.navigation_complete = True
                else:
                    # Calculate approach position for HEAD-ON approach
                    target_distance = 0.45  # Slightly closer than max reach for reliability
                    
                    # HEAD-ON APPROACH: Position robot so its +X axis (forward direction) faces the object
                    # In robot coordinates: +X = forward, +Y = left, angle 0 = facing east
                    # For head-on approach, robot should face object with its +X axis pointing toward object
                    
                    # We want robot positioned WEST of object so it faces EAST toward object
                    # This means robot's +X axis (forward) points toward object (+X direction)
                    target_pos = [
                        obj_pos[0] - target_distance,  # Position robot WEST of object
                        obj_pos[1],                    # Same Y position as object
                        robot_pos[2]                   # Keep original height
                    ]
                    
                    print(f"  🎯 Object at {obj_pos[:2]}, Robot at {robot_pos[:2]}")
                    print(f"  🎯 Head-on approach: Robot positioned WEST of object to face EAST")
                    print(f"  🎯 Target position: {target_pos[:2]} (robot's +X axis will point toward object)")
                    
                    print(f"  🎯 Navigation attempt {self.navigation_attempt}: moving to {target_pos}")
                    print(f"  🎯 Navigation details:")
                    print(f"      Object at {obj_pos[:2]}")
                    print(f"      Robot currently at {robot_pos[:2]}")
                    print(f"      Head-on approach target: {target_pos[:2]} (facing object directly)")
                    print(f"      Head-on approach: Robot faces EAST (+X direction) toward object")
                    
                    # Set world reference for robot
                    setattr(self.robot, '_world_ref', self.world)
                    
                    # Start walking to target position
                    self.robot.start_walking_to(target_pos)

                    return NodeStatus.RUNNING
                        
        # Step 2: Turn to face the object for optimal gripper alignment
        if not self.orientation_complete:
            if not self.turning:
                self.robot.turn_to_face(obj_pos)
                self.turning = True
                print(f"  🔄 Turning to face object at {obj_pos}")
                
            if self.robot.is_turning():
                return NodeStatus.RUNNING
            else:
                self.orientation_complete = True
                self.turning = False
                print(f"  ✅ Robot oriented toward object")
        
        # Step 3: Attempt to pick from current position
        if not self.picking:
            success = self.robot.attempt_pick(self.object_id, obj_pos)
            if success:
                self.picking = True
            else:
                self.pick_attempt += 1
                if self.pick_attempt >= self.max_attempts:
                    return self._end_execution(NodeStatus.FAILURE)
                    
        if self.picking:
            # Check if pick is complete
            if self.robot.is_holding(self.object_id):
                return self._end_execution(NodeStatus.SUCCESS)
            else:
                return NodeStatus.RUNNING
        else:
            return NodeStatus.RUNNING
            
    def reset(self) -> None:
        """Reset picking state"""
        super().reset()
        self.picking = False
        self.turning = False
        self.pick_attempt = 0
        self.navigation_complete = False
        self.orientation_complete = False


class PlaceObject(ActionNode):
    """Action node for placing an object at a location"""
    
    def __init__(self, location: List[float], object_id: str = None, **kwargs):
        super().__init__(**kwargs)
        self.location = location
        self.object_id = object_id  # If None, place whatever is held
        self.placing = False
        self.turning = False
        self.orientation_complete = False
        
    def tick(self, context: Any = None) -> NodeStatus:
        """Execute placing behavior"""
        self.setup_from_context(context)
        
        if self.start_time is None:
            self._start_execution()
            
        if not self.robot:
            return self._end_execution(NodeStatus.FAILURE)
            
        # Check if robot is holding something
        if not self.robot.is_holding_any():
            return self._end_execution(NodeStatus.FAILURE)
            
        # Check if we're close enough to place location
        robot_pos = self.robot.get_position()
        distance = np.linalg.norm(np.array(self.location[:2]) - np.array(robot_pos[:2]))
        
        if distance > self.robot.reach_distance:
            # Need to get closer
            return self._end_execution(NodeStatus.FAILURE)
        
        # Step 1: Turn to face the placement location for optimal gripper alignment
        if not self.orientation_complete:
            if not self.turning:
                self.robot.turn_to_face(self.location)
                self.turning = True
                print(f"  🔄 Turning to face placement location at {self.location}")
                
            if self.robot.is_turning():
                return NodeStatus.RUNNING
            else:
                self.orientation_complete = True
                self.turning = False
                print(f"  ✅ Robot oriented toward placement location")
            
        # Step 2: Attempt to place
        if not self.placing:
            success = self.robot.attempt_place(self.location, self.object_id)
            if success:
                self.placing = True
            else:
                return self._end_execution(NodeStatus.FAILURE)
                
        if self.placing:
            # Check if place is complete
            if not self.robot.is_holding_any() or (self.object_id and not self.robot.is_holding(self.object_id)):
                return self._end_execution(NodeStatus.SUCCESS)
            else:
                return NodeStatus.RUNNING
        else:
            return NodeStatus.RUNNING
            
    def reset(self) -> None:
        """Reset placing state"""
        super().reset()
        self.placing = False
        self.turning = False
        self.orientation_complete = False


class WaitForHuman(ActionNode):
    """Action node for waiting/idle behavior"""
    
    def __init__(self, duration: float = 1.0, **kwargs):
        super().__init__(**kwargs)
        
        # Validate duration parameter
        if not isinstance(duration, (int, float)):
            raise TypeError("Duration must be a number")
        if duration < 0:
            raise ValueError("Duration must be non-negative")
            
        self.duration = duration
        self.wait_start = None
        
    def tick(self, context: Any = None) -> NodeStatus:
        """Execute waiting behavior"""
        if self.start_time is None:
            self._start_execution()
            self.wait_start = time.time()
            
        elapsed = time.time() - self.wait_start
        
        if elapsed >= self.duration:
            return self._end_execution(NodeStatus.SUCCESS)
        else:
            return NodeStatus.RUNNING
            
    def reset(self) -> None:
        """Reset waiting state"""
        super().reset()
        self.wait_start = None
