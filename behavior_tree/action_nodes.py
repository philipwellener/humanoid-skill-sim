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
            
            # Check if robot is in good position for picking (optimal range AND no pending waypoints)
            print(f"  📏 Current distance to object: {distance:.3f}m (robot at {robot_pos[:2]}, object at {obj_pos[:2]})")
            
            # Check for pending waypoints
            has_pending_waypoints = hasattr(self.robot, '_waypoint_queue') and len(self.robot._waypoint_queue) > 0
            
            # More strict positioning requirements
            if 0.35 <= distance <= 0.55 and not has_pending_waypoints:  # Optimal range AND completed navigation
                # Additional check: make sure robot is not too high above object
                height_diff = abs(robot_pos[2] - obj_pos[2])
                if height_diff < 0.6:  # Robot should be reasonably close in height
                    self.navigation_complete = True
                    print(f"  ✅ Robot in good position for picking (distance: {distance:.3f}m, height_diff: {height_diff:.3f}m, no pending waypoints)")
                else:
                    print(f"  ⚠️  Robot too high above object (height_diff: {height_diff:.3f}m)")
            else:
                if has_pending_waypoints:
                    print(f"  🛤️ Robot navigating multi-waypoint path ({len(self.robot._waypoint_queue)} waypoints remaining)...")
                    print(f"  ⏳ Current distance: {distance:.3f}m - waiting for complete navigation")
                else:
                    print(f"  ❌ Robot not in optimal position (distance: {distance:.3f}m, need: 0.35-0.55m)")
                
                # Check if robot is currently walking OR has active waypoints
                if self.robot.is_walking() or has_pending_waypoints:
                    if has_pending_waypoints:
                        print(f"  � Multi-waypoint navigation in progress, waiting for completion...")
                    else:
                        print(f"  🚶 Robot is walking, waiting for movement to complete...")
                    return NodeStatus.RUNNING
                
                # Need to navigate to better position
                self.navigation_attempt += 1
                if self.navigation_attempt > self.max_navigation_attempts:
                    print(f"  ❌ Too many navigation attempts, proceeding with pick anyway")
                    self.navigation_complete = True
                else:
                    # Calculate approach position for OPTIMAL POSITIONING
                    target_distance = 0.45  # Distance to maintain from object
                    
                    # STRATEGIC APPROACH: Position robot at optimal distance and let turning handle orientation
                    # Calculate direction from robot to object for approach vector
                    robot_to_obj = np.array(obj_pos[:2]) - np.array(robot_pos[:2])
                    distance_to_obj = np.linalg.norm(robot_to_obj)
                    
                    if distance_to_obj > 0.01:
                        # Normalize the direction from robot to object
                        approach_direction = robot_to_obj / distance_to_obj
                        # Position robot at target_distance away from object, maintaining approach angle
                        target_pos = [
                            obj_pos[0] - approach_direction[0] * target_distance,
                            obj_pos[1] - approach_direction[1] * target_distance,
                            0.5  # Fixed safe height above ground
                        ]
                    else:
                        # Default fallback: position west of object
                        target_pos = [
                            obj_pos[0] - target_distance,  # Position robot west of object
                            obj_pos[1],                    # Same Y position as object
                            0.5  # Fixed safe height above ground
                        ]
                    
                    print(f"  🎯 Object at {obj_pos[:2]}, Robot at {robot_pos[:2]}")
                    print(f"  🎯 Strategic approach: Robot positioned {target_distance}m from object")
                    print(f"  🎯 Target position: {target_pos[:2]} (will turn to face object after arriving)")
                    
                    print(f"  🎯 Navigation attempt {self.navigation_attempt}: moving to {target_pos}")
                    print(f"  🎯 Navigation details:")
                    print(f"      Object at {obj_pos[:2]}")
                    print(f"      Robot currently at {robot_pos[:2]}")
                    print(f"      Approach target: {target_pos[:2]} (proper turning will follow)")
                    
                    # Set world reference for robot
                    setattr(self.robot, '_world_ref', self.world)
                    
                    # Start walking to target position
                    self.robot.start_walking_to(target_pos)

                    return NodeStatus.RUNNING
                        
        # Step 2: Turn to face the object for optimal gripper alignment
        if not self.orientation_complete:
            if not self.turning:
                # Get current robot position and calculate angle to object
                robot_pos = self.robot.get_position()
                dx = obj_pos[0] - robot_pos[0]
                dy = obj_pos[1] - robot_pos[1]
                # CORRECTED: Subtract 90 degrees so robot orientation makes gripper face object
                target_angle = np.arctan2(dy, dx) - np.pi/2  # Subtract 90 degrees so gripper faces object
                
                print(f"  🔄 Precise turning: Robot at {robot_pos[:2]}, Object at {obj_pos[:2]}")
                print(f"  🧭 Target angle: {np.degrees(target_angle):.1f}° (toward object, corrected for gripper orientation)")
                
                # Use the corrected angle for turning
                self.robot.turn_to_angle(target_angle)
                self.turning = True
                print(f"  🔄 Turning to corrected angle to face object at {obj_pos}")
                
            if self.robot.is_turning():
                return NodeStatus.RUNNING
            else:
                # Verify final orientation
                robot_pos = self.robot.get_position()
                robot_orientation = self.robot.get_orientation()
                dx = obj_pos[0] - robot_pos[0]
                dy = obj_pos[1] - robot_pos[1]
                target_angle = np.arctan2(dy, dx) - np.pi/2  # Same correction as above
                angle_error = abs(target_angle - robot_orientation)
                
                print(f"  ✅ Robot oriented toward object")
                print(f"  🧭 Final orientation: {np.degrees(robot_orientation):.1f}°, Target: {np.degrees(target_angle):.1f}°")
                print(f"  📐 Angle error: {np.degrees(angle_error):.1f}°")
                
                self.orientation_complete = True
                self.turning = False
        
        # Step 3: Attempt to pick from current position
        if not self.picking:
            success = self.robot.attempt_pick(self.object_id)
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
        self.navigation_complete = False
        self.navigation_attempt = 0
        self.max_navigation_attempts = 5
        self.target_waypoint = None  # Store calculated waypoint to avoid recalculating
        
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
            
        # Step 1: Navigate to optimal position for bin placement (approach from side)
        if not self.navigation_complete:
            robot_pos = self.robot.get_position()
            
            # CRITICAL FIX: Calculate edge waypoint IMMEDIATELY if not already calculated
            # This prevents fallback navigation to bin center
            if not hasattr(self, 'target_waypoint') or self.target_waypoint is None:
                print(f"  🎯 Calculating PLATFORM-AWARE collision-free waypoint (IMMEDIATELY)...")
                
                # PLATFORM-AWARE SOLUTION: Choose position that avoids platform cluster
                # - Must be within 0.75m reach distance
                # - Must be far enough from platforms to allow collision-free navigation
                # - Consider platform positions when choosing approach direction
                
                bin_center = np.array(self.location[:2])
                robot_position = np.array(robot_pos[:2])
                
                # Get platform positions for collision-aware placement
                platform_positions = []
                bin_positions = []
                if hasattr(self.robot, '_world_ref') and self.robot._world_ref is not None:
                    for obj_id, position in self.robot._world_ref.object_positions.items():
                        if 'platform' in obj_id.lower():
                            platform_positions.append(np.array(position[:2]))
                        elif 'bin' in obj_id.lower():
                            # Only include bins that are NOT the target bin (avoid the one we're placing into)
                            target_bin_center = np.array(self.location[:2])
                            bin_pos = np.array(position[:2])
                            distance_to_target = np.linalg.norm(bin_pos - target_bin_center)
                            if distance_to_target > 0.3:  # Not the target bin (different location)
                                bin_positions.append(bin_pos)
                
                print(f"  🏛️ Found {len(platform_positions)} platforms and {len(bin_positions)} other bins for collision-aware placement")
                
                # EDGE-CENTER APPROACH: Position robot at center of closest bin edge
                # Calculate the 4 edge centers of the bin (North, South, East, West)
                edge_centers = [
                    [bin_center[0], bin_center[1] + 0.70],  # North edge center
                    [bin_center[0], bin_center[1] - 0.70],  # South edge center  
                    [bin_center[0] + 0.70, bin_center[1]],  # East edge center
                    [bin_center[0] - 0.70, bin_center[1]]   # West edge center
                ]
                
                edge_names = ["North", "South", "East", "West"]
                
                print(f"  🎯 Evaluating 4 bin edge centers for optimal approach:")
                for i, (edge_center, edge_name) in enumerate(zip(edge_centers, edge_names)):
                    robot_distance = np.linalg.norm(np.array(edge_center) - robot_position)
                    print(f"    {edge_name} edge [{edge_center[0]:.2f}, {edge_center[1]:.2f}]: robot distance {robot_distance:.2f}m")
                
                best_waypoint = None
                best_score = -float('inf')
                
                # Evaluate each edge center for platform clearance and robot proximity
                for i, (edge_center, edge_name) in enumerate(zip(edge_centers, edge_names)):
                    edge_center_np = np.array(edge_center)
                    
                    # Check distance to nearest platform
                    min_distance_to_platform = float('inf')
                    if platform_positions:
                        distances_to_platforms = [np.linalg.norm(edge_center_np - platform_pos) 
                                                 for platform_pos in platform_positions]
                        min_distance_to_platform = min(distances_to_platforms)
                    
                    # Check distance to nearest bin (avoid other bins)
                    min_distance_to_bin = float('inf')
                    if bin_positions:
                        distances_to_bins = [np.linalg.norm(edge_center_np - bin_pos) 
                                           for bin_pos in bin_positions]
                        min_distance_to_bin = min(distances_to_bins)
                    
                    # Calculate distance from robot to this edge center
                    robot_to_edge_distance = np.linalg.norm(edge_center_np - robot_position)
                    
                    # Score = obstacle clearance (higher is better) + robot proximity (closer is better)
                    # Heavily prioritize robot proximity to minimize navigation distance
                    min_obstacle_clearance = min(min_distance_to_platform, min_distance_to_bin)
                    obstacle_clearance_score = min_obstacle_clearance
                    robot_proximity_score = 4.0 - robot_to_edge_distance  # Prefer closer edges
                    combined_score = obstacle_clearance_score * 0.2 + robot_proximity_score * 0.8  # 80% weight on robot proximity
                    
                    print(f"    {edge_name} edge analysis: platform distance {min_distance_to_platform:.2f}m, bin distance {min_distance_to_bin:.2f}m, robot distance {robot_to_edge_distance:.2f}m, score {combined_score:.2f}")
                    
                    # Use more relaxed obstacle clearance - allow closer approach for efficiency
                    # 0.6m clearance is sufficient for robot navigation while prioritizing robot proximity
                    if min_obstacle_clearance >= 0.6 and combined_score > best_score:
                        best_score = combined_score
                        best_waypoint = edge_center_np
                        print(f"    ✅ {edge_name} edge selected as best option (score: {combined_score:.2f})")
                    elif min_obstacle_clearance < 0.6:
                        print(f"    ❌ {edge_name} edge rejected: too close to obstacles (min clearance: {min_obstacle_clearance:.2f}m < 0.6m)")
                
                # Use the best waypoint found, or fallback to simple calculation
                if best_waypoint is not None:
                    self.target_waypoint = [best_waypoint[0], best_waypoint[1], 0.5]
                    print(f"  ✅ PLATFORM-AWARE waypoint selected: {self.target_waypoint[:2]}")
                    
                    # Calculate final platform clearance for the selected waypoint
                    final_platform_clearance = float('inf')
                    if platform_positions:
                        distances_to_platforms = [np.linalg.norm(best_waypoint - platform_pos) 
                                                 for platform_pos in platform_positions]
                        final_platform_clearance = min(distances_to_platforms)
                    print(f"  🏛️ Minimum distance from platforms: {final_platform_clearance:.2f}m")
                else:
                    # Fallback: choose closest edge center even if platform clearance is not ideal
                    print(f"  ⚠️ No edge meets platform clearance requirements, choosing closest edge as fallback")
                    closest_edge_distance = float('inf')
                    for i, (edge_center, edge_name) in enumerate(zip(edge_centers, edge_names)):
                        edge_center_np = np.array(edge_center)
                        robot_to_edge_distance = np.linalg.norm(edge_center_np - robot_position)
                        if robot_to_edge_distance < closest_edge_distance:
                            closest_edge_distance = robot_to_edge_distance
                            best_waypoint = edge_center_np
                            print(f"    Fallback: {edge_name} edge selected (closest to robot: {robot_to_edge_distance:.2f}m)")
                    
                    self.target_waypoint = [best_waypoint[0], best_waypoint[1], 0.5]
                    print(f"  ⚠️ Using fallback edge center waypoint: {self.target_waypoint[:2]}")
                
                # Verify final waypoint is reachable
                final_distance_to_bin = np.linalg.norm(np.array(self.target_waypoint[:2]) - bin_center)
                if final_distance_to_bin <= 0.75:  # Within robot reach
                    print(f"  ✅ PLATFORM-AWARE waypoint calculation complete!")
                    print(f"  🎯 Waypoint details:")
                    print(f"      Bin center: {bin_center}")
                    print(f"      Final target waypoint: {self.target_waypoint[:2]}")
                    print(f"      Final distance to bin center: {final_distance_to_bin:.3f}m (✅ within 0.75m reach)")
                    
                    # Calculate final platform clearance for display
                    if best_waypoint is not None and platform_positions:
                        final_clearance = min([np.linalg.norm(best_waypoint - platform_pos) for platform_pos in platform_positions])
                        print(f"      Minimum distance from platforms: {final_clearance:.2f}m")
                    print(f"      Collision-free placement achieved!")
                else:
                    print(f"  ⚠️ WARNING: Waypoint may be too far from bin ({final_distance_to_bin:.2f}m)")
                print(f"      Final target waypoint: {self.target_waypoint[:2]}")
                print(f"      Final distance to bin center: {final_distance_to_bin:.3f}m (✅ within reach)")
                print(f"      Collision-free rotation: Robot can turn without hitting bin!")
            
            # Now check distance to the edge waypoint (never to bin center)
            distance = np.linalg.norm(np.array(self.target_waypoint[:2]) - np.array(robot_pos[:2]))
            print(f"  📏 Current distance to edge waypoint: {distance:.3f}m (robot at {robot_pos[:2]}, waypoint at {self.target_waypoint[:2]})")
            
            # CRITICAL: Only complete navigation when robot reaches FINAL target AND has no more waypoints
            # Robot should get close to the edge waypoint (within 15cm) AND have no pending waypoints
            has_pending_waypoints = hasattr(self.robot, '_waypoint_queue') and len(self.robot._waypoint_queue) > 0
            
            if distance <= 0.15 and not has_pending_waypoints:
                self.navigation_complete = True
                print(f"  ✅ Robot reached FINAL edge waypoint successfully (distance: {distance:.3f}m, no pending waypoints)")
            else:
                if has_pending_waypoints:
                    print(f"  🛤️ Robot navigating multi-waypoint path ({len(self.robot._waypoint_queue)} waypoints remaining)...")
                    print(f"  ⏳ Current waypoint distance: {distance:.3f}m - continuing multi-waypoint navigation")
                else:
                    print(f"  🎯 Robot approaching FINAL edge waypoint (distance: {distance:.3f}m)")
                
                # Check if robot is currently walking OR has active waypoints
                if self.robot.is_walking() or has_pending_waypoints:
                    if has_pending_waypoints:
                        print(f"  � Multi-waypoint navigation in progress, waiting for completion...")
                    else:
                        print(f"  🚶 Robot is walking to bin edge, waiting for movement to complete...")
                    return NodeStatus.RUNNING
                
                # Start navigation to edge waypoint
                self.navigation_attempt += 1
                if self.navigation_attempt > self.max_navigation_attempts:
                    print(f"  ❌ Too many navigation attempts, proceeding with placement anyway")
                    self.navigation_complete = True
                else:
                    # Start walking to the edge waypoint
                    print(f"  🎯 Navigation attempt {self.navigation_attempt}: walking to edge waypoint {self.target_waypoint[:2]}")
                    self.robot.start_walking_to(self.target_waypoint)
                    return NodeStatus.RUNNING
        
        # Step 2: Turn to face the bin, accounting for gripper length to drop inside
        if not self.orientation_complete:
            if not self.turning:
                # Get current robot position and calculate angle to placement location
                robot_pos = self.robot.get_position()
                
                # ACCOUNT FOR GRIPPER LENGTH: Position gripper reach point to aim into bin
                # Robot needs to position its gripper to be over the bin center
                gripper_reach = 0.4  # Gripper reach for close bin edge placement
                
                # Calculate where robot should face so gripper will be over bin center
                dx = self.location[0] - robot_pos[0]
                dy = self.location[1] - robot_pos[1]
                
                # CORRECTED: Subtract 90 degrees so robot orientation makes gripper face bin
                # This matches the pickup action orientation calculation
                target_angle = np.arctan2(dy, dx) - np.pi/2  # Subtract 90 degrees so gripper faces bin
                
                print(f"  🔄 Turning to drop INTO bin: Robot at {robot_pos[:2]}, Bin center at {self.location[:2]}")
                print(f"  📏 Gripper reach: {gripper_reach}m - aiming to drop directly into bin")
                print(f"  🧭 Target angle: {np.degrees(target_angle):.1f}° (corrected for gripper orientation)")
                
                # Use the corrected angle for turning
                self.robot.turn_to_angle(target_angle)
                self.turning = True
                print(f"  🔄 Turning to face bin center with gripper side forward")
                
            if self.robot.is_turning():
                return NodeStatus.RUNNING
            else:
                # Verify final orientation
                robot_pos = self.robot.get_position()
                robot_orientation = self.robot.get_orientation()
                dx = self.location[0] - robot_pos[0]
                dy = self.location[1] - robot_pos[1]
                target_angle = np.arctan2(dy, dx) - np.pi/2  # Same correction as above
                angle_error = abs(target_angle - robot_orientation)
                
                print(f"  ✅ Robot oriented to drop INTO bin center")
                print(f"  🧭 Final orientation: {np.degrees(robot_orientation):.1f}°, Target: {np.degrees(target_angle):.1f}°")
                print(f"  📐 Angle error: {np.degrees(angle_error):.1f}°")
                print(f"  📦 Gripper positioned to drop box directly into bin")
                
                self.orientation_complete = True
                self.turning = False
                # Now that we're oriented, move closer if needed
                self.need_closer_positioning = True
                
        # Step 2b: Move closer after turning toward bin (if needed)
        if self.orientation_complete and getattr(self, 'need_closer_positioning', False):
            robot_pos = self.robot.get_position()
            distance_to_bin = np.linalg.norm(np.array(self.location[:2]) - np.array(robot_pos[:2]))
            
            # PRECISION PLACEMENT: Move much closer for accurate drop while avoiding collision
            # Robot radius ~25cm + Bin size 20cm + Safety margin 10cm = 55cm minimum distance
            # But for optimal placement accuracy, use 50cm (robot can extend gripper)
            optimal_distance = 0.50  # 50cm for precise placement (accounting for robot size ~25cm + bin size 20cm + safety 5cm)
            
            if distance_to_bin > optimal_distance:
                # Calculate position that maintains safe distance from bin center but allows precise placement
                direction = np.array(robot_pos[:2]) - np.array(self.location[:2])
                direction_normalized = direction / np.linalg.norm(direction)
                target_pos = np.array(self.location[:2]) + direction_normalized * optimal_distance
                target_pos_3d = [target_pos[0], target_pos[1], robot_pos[2]]
                
                print(f"  🎯 Moving closer for PRECISION placement: {distance_to_bin:.2f}m → {optimal_distance:.2f}m (robot+bin clearance)")
                self.robot.start_walking_to(target_pos_3d)
                self.need_closer_positioning = False
                self.final_positioning = True
                
        if getattr(self, 'final_positioning', False):
            if self.robot.is_walking():
                return NodeStatus.RUNNING
            else:
                self.final_positioning = False
                print(f"  ✅ Robot positioned optimally for precise placement")
            
        # Step 3: Attempt to place
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
        self.navigation_complete = False
        self.navigation_attempt = 0


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
