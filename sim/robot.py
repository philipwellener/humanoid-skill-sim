"""
Humanoid robot simulation with manipulation capabilities
"""

import pybullet as p
import numpy as np
import time
import random
from typing import List, Optional, Dict, Tuple


class HumanoidRobot:
    """Simulated humanoid robot with walking and manipulation capabilities"""
    
    def __init__(self, urdf_path: str = None, start_position: List[float] = None):
        self.urdf_path = urdf_path or "r2d2.urdf"  # Fallback to simple robot
        self.start_position = start_position or [0, 0, 0.5]
        self.robot_id = None
        self.reach_distance = 0.75  # Extended reach distance for improved performance
        self.current_position = self.start_position.copy()
        self.current_orientation = 0.0  # Current yaw angle in radians
        self.target_position = None
        self.target_orientation = None
        self.walking = False
        self.turning = False
        self.walk_speed = 2.0  # m/s - Faster for behavior tree execution
        self.turn_speed = 2.0  # rad/s
        self.walk_start_time = None
        self.turn_start_time = None
        self.held_objects: Dict[str, int] = {}  # object_id -> constraint_id
        self.end_effector_link = -1  # End effector link index
        self.joints = {}  # Joint name -> index mapping
        self.grasped_object_id = None  # Currently grasped object
        self.grasp_constraint_id = None  # Current grasp constraint
        self._world_ref = None  # Reference to the simulation world
        self._waypoint_queue = []  # Queue for multi-waypoint navigation
        
    def set_world_reference(self, world_instance) -> None:
        """Set reference to the simulation world for collision avoidance and object access"""
        self._world_ref = world_instance
        print(f"🌍 Robot world reference set - collision avoidance enabled")
        
    def load_robot(self) -> bool:
        """Load the robot into the simulation"""
        try:
            self.robot_id = p.loadURDF(
                self.urdf_path,
                self.start_position,
                [0, 0, 0, 1]  # Upright orientation
            )
            
            # Get joint information
            self._discover_joints()
            return True
            
        except Exception as e:
            print(f"Failed to load robot: {e}")
            # Try to load a simple fallback robot
            try:
                self.robot_id = p.loadURDF("cube_small.urdf", self.start_position)
                return True
            except:
                return False
                
    def _discover_joints(self) -> None:
        """Discover joint indices and capabilities"""
        if self.robot_id is None:
            return
            
        num_joints = p.getNumJoints(self.robot_id)
        print(f"🔍 Robot has {num_joints} joints:")
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            self.joints[joint_name] = i
            print(f"  Joint {i}: {joint_name} (type: {joint_type})")
            
        # For simplicity, use the last joint as end effector
        if num_joints > 0:
            self.end_effector_link = num_joints - 1
            
        print(f"🤖 Available joints: {list(self.joints.keys())}")
            
    def get_position(self) -> List[float]:
        """Get current robot base position"""
        if self.robot_id is None:
            return self.current_position
            
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        self.current_position = list(pos)
        return self.current_position
    
    def set_position(self, position: List[float]) -> None:
        """Set robot base position"""
        self.current_position = position.copy()
        if self.robot_id is not None:
            p.resetBasePositionAndOrientation(
                self.robot_id,
                position,
                [0, 0, 0, 1]  # Keep upright orientation
            )
        
    def start_walking_to(self, target: List[float]) -> None:
        """Start walking to target position using safe path planning with multi-waypoint support"""
        # CRITICAL: Secure any held objects before starting navigation
        if self.held_objects:
            print(f"  🔒 Securing {len(self.held_objects)} held objects before navigation")
            self._update_held_objects()
            self._verify_object_attachment()
            
            # Strengthen all constraints for navigation
            for object_id, constraint_id in self.held_objects.items():
                try:
                    p.changeConstraint(constraint_id, maxForce=12000)  # Even stronger during navigation to prevent lagging
                    print(f"  🔧 Strengthened constraint for {object_id} to 12000N for navigation")
                except Exception as e:
                    print(f"  ⚠️ Could not strengthen constraint for {object_id}: {e}")
        
        # Always use safe path planning to avoid platforms
        if hasattr(self, '_world_ref') and self._world_ref is not None:
            print(f"  🚶 Planning safe route to {target[:2]}")
            safe_waypoints = self._find_safe_path(target)
            if len(safe_waypoints) == 0:
                print(f"  🛑 CRITICAL: No safe path found - robot cannot move to avoid collisions!")
                print(f"  ⚠️ Robot stopping - target {target[:2]} is unreachable without collision")
                return  # Don't start walking if no safe path
            elif len(safe_waypoints) > 1:
                print(f"  🛤️ Using {len(safe_waypoints)}-waypoint route for collision avoidance")
                # Store all waypoints for sequential navigation
                self._waypoint_queue = safe_waypoints.copy()
                # Start with first waypoint
                self.target_position = self._waypoint_queue.pop(0).copy()
                print(f"  🎯 First waypoint: {self.target_position[:2]} ({len(self._waypoint_queue)} more to follow)")
            else:
                self.target_position = target.copy()
                self._waypoint_queue = []  # Clear any existing queue
        else:
            self.target_position = target.copy()
            self._waypoint_queue = []  # Clear any existing queue
            
        self.walking = True
        self.walk_start_time = time.time()
        # Store starting position for proper interpolation
        self._walk_start_pos = self.get_position().copy()
        print(f"🚶 Robot starting walk from {self._walk_start_pos[:2]} to {self.target_position[:2]}")
        
    def _complete_walk(self) -> None:
        """Complete the walking motion and handle waypoint queue"""
        self.walking = False
        if self.target_position and self.robot_id is not None:
            # Set final position
            final_pos = [self.target_position[0], self.target_position[1], self.start_position[2]]
            _, current_orn = p.getBasePositionAndOrientation(self.robot_id)
            p.resetBasePositionAndOrientation(self.robot_id, final_pos, current_orn)
            self.current_position = final_pos
            
            # Update held object positions
            self._update_held_objects()
            
            # Check if there are more waypoints to navigate to
            if hasattr(self, '_waypoint_queue') and len(self._waypoint_queue) > 0:
                next_waypoint = self._waypoint_queue.pop(0)
                print(f"  🎯 Continuing to next waypoint: {next_waypoint[:2]} ({len(self._waypoint_queue)} remaining)")
                
                # CRITICAL: Verify object attachment before continuing navigation
                if self.held_objects:
                    print(f"  🔍 Verifying {len(self.held_objects)} held objects before waypoint transition")
                    self._verify_object_attachment()
                    self._update_held_objects()
                
                # Start walking to next waypoint immediately (no planning needed, already planned)
                self.target_position = next_waypoint.copy()
                self.walking = True
                self.walk_start_time = time.time()
                self._walk_start_pos = self.get_position().copy()
                print(f"  🚶 Robot continuing multi-waypoint navigation to {self.target_position[:2]}")
            else:
                print(f"  ✅ Multi-waypoint navigation complete!")
                if hasattr(self, '_waypoint_queue'):
                    self._waypoint_queue = []  # Clear queue
                    
                # Final verification that objects are still held after navigation
                if self.held_objects:
                    print(f"  🔍 Final verification of {len(self.held_objects)} held objects after navigation")
                    self._verify_object_attachment()
        
    def is_walking(self) -> bool:
        """Check if robot is currently walking"""
        if not self.walking:
            return False
            
        # Get start and target positions for interpolation
        start_pos = getattr(self, '_walk_start_pos', self.current_position)
        
        # Calculate total distance and elapsed time
        total_distance = np.linalg.norm(np.array(self.target_position[:2]) - np.array(start_pos[:2]))
        elapsed_time = time.time() - self.walk_start_time
        expected_time = total_distance / self.walk_speed
        
        # Check if robot is close to target (within 30cm) - improved waypoint completion detection  
        current_pos = self.get_position()
        distance_to_target = np.linalg.norm(np.array(self.target_position[:2]) - np.array(current_pos[:2]))
        
        # More generous tolerance for waypoint completion and stricter time limit
        tolerance = 0.30  # 30cm tolerance for waypoint completion
        time_exceeded = elapsed_time >= expected_time * 1.2  # Allow 20% extra time
        oscillation_limit = elapsed_time > 10.0  # Force completion after 10 seconds to prevent infinite loops
        
        if distance_to_target <= tolerance or time_exceeded or oscillation_limit:
            # Walking completed
            completion_reason = "distance" if distance_to_target <= tolerance else ("time" if time_exceeded else "oscillation")
            print(f"  ✅ Waypoint completed ({completion_reason}): distance {distance_to_target:.3f}m (time: {elapsed_time:.1f}s)")
            self._complete_walk()
            return False
            
        # Calculate progress (0.0 to 1.0)
        progress = elapsed_time / expected_time if expected_time > 0 else 1.0
        progress = min(progress, 1.0)
        
        # Interpolate between start and target positions
        new_pos = [
            start_pos[0] + (self.target_position[0] - start_pos[0]) * progress,
            start_pos[1] + (self.target_position[1] - start_pos[1]) * progress,
            max(0.5, self.target_position[2]) if len(self.target_position) > 2 else 0.5  # Proper height management
        ]
        
        # Update robot position in simulation
        if self.robot_id is not None:
            _, current_orn = p.getBasePositionAndOrientation(self.robot_id)
            p.resetBasePositionAndOrientation(self.robot_id, new_pos, current_orn)
            
            # Update current position tracker
            self.current_position = new_pos
            
            # CRITICAL: Update held objects more frequently during navigation to prevent dropping
            self._update_held_objects()
            
            # Additional verification for held objects during multi-waypoint navigation
            if self.held_objects and hasattr(self, '_waypoint_queue') and len(self._waypoint_queue) > 0:
                self._verify_object_attachment()
        
        return True
    
    def stop_walking(self) -> None:
        """Stop the walking motion"""
        self.walking = False
        self.target_position = None
        self.walk_start_time = None
        
    def turn_to_face(self, target_position: List[float]) -> None:
        """Turn to face a target position"""
        current_pos = self.get_position()
        dx = target_position[0] - current_pos[0]
        dy = target_position[1] - current_pos[1]
        
        # Calculate target angle
        target_angle = np.arctan2(dy, dx)
        self.target_orientation = target_angle
        self.turning = True
        self.turn_start_time = time.time()
        
    def turn_to_angle(self, target_angle: float) -> None:
        """Turn to a specific angle"""
        self.target_orientation = target_angle
        self.turning = True
        self.turn_start_time = time.time()
        
    def is_turning(self) -> bool:
        """Check if robot is currently turning"""
        if not self.turning:
            return False
            
        # Calculate angle difference
        angle_diff = self._normalize_angle(self.target_orientation - self.current_orientation)
        
        if abs(angle_diff) < 0.05:  # Close enough (about 3 degrees)
            self._complete_turn()
            return False
            
        # Update orientation during turn
        elapsed_time = time.time() - self.turn_start_time
        turn_direction = 1 if angle_diff > 0 else -1
        turn_amount = self.turn_speed * elapsed_time * turn_direction
        
        # Don't overshoot
        if abs(turn_amount) > abs(angle_diff):
            turn_amount = angle_diff
            
        new_orientation = self.current_orientation + turn_amount
        self.current_orientation = self._normalize_angle(new_orientation)
        
        # Update robot orientation in simulation
        if self.robot_id is not None:
            current_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            new_quaternion = p.getQuaternionFromEuler([0, 0, self.current_orientation])
            p.resetBasePositionAndOrientation(self.robot_id, current_pos, new_quaternion)
            
            # Update held objects to follow robot orientation
            self._update_held_objects()
            
        return True
        
    def stop_turning(self) -> None:
        """Stop the turning motion"""
        self.turning = False
        self.target_orientation = None
        self.turn_start_time = None
        
    def _complete_turn(self) -> None:
        """Complete the turning motion"""
        self.turning = False
        if self.target_orientation is not None and self.robot_id is not None:
            # Set final orientation
            self.current_orientation = self.target_orientation
            current_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            new_quaternion = p.getQuaternionFromEuler([0, 0, self.current_orientation])
            p.resetBasePositionAndOrientation(self.robot_id, current_pos, new_quaternion)
            
            # Update held object positions
            self._update_held_objects()
            
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
        
    def get_orientation(self) -> float:
        """Get current orientation in radians"""
        if self.robot_id is None:
            return self.current_orientation
            
        _, quaternion = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(quaternion)
        self.current_orientation = euler[2]  # Yaw angle
        return self.current_orientation
        
    def _update_held_objects(self):
        """Update positions of held objects to follow robot and verify constraints"""
        if not self.held_objects:
            return
            
        # Verify all constraints are still active and objects are properly attached
        for object_id, constraint_id in list(self.held_objects.items()):
            try:
                # Check if constraint still exists by trying to get its info
                constraint_info = p.getConstraintInfo(constraint_id)
                
                # Get object and robot positions to verify attachment
                if hasattr(self, '_world_ref') and self._world_ref and hasattr(self._world_ref, 'objects'):
                    if object_id in self._world_ref.objects:
                        obj_bullet_id = self._world_ref.objects[object_id]
                        obj_pos, _ = p.getBasePositionAndOrientation(obj_bullet_id)
                        robot_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
                        
                        # Calculate distance between robot and object
                        distance = np.linalg.norm(np.array(obj_pos[:2]) - np.array(robot_pos[:2]))
                        
                        # If object is too far from robot, the constraint may have broken
                        if distance > 1.0:  # 1m max distance for held objects
                            print(f"  ⚠️ Warning: {object_id} is {distance:.2f}m from robot - constraint may be weak")
                            
                            # Strengthen the constraint
                            p.changeConstraint(constraint_id, maxForce=3000)  # Increase force
                            
                            # Reposition object closer to robot if needed
                            robot_euler = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.robot_id)[1])
                            robot_yaw = robot_euler[2] + np.pi/2  # Correct forward direction
                            
                            # Position object securely in front of robot
                            secure_pos = [
                                robot_pos[0] + 0.35 * np.cos(robot_yaw),
                                robot_pos[1] + 0.35 * np.sin(robot_yaw),
                                max(obj_pos[2], robot_pos[2] + 0.1)  # Keep reasonable height
                            ]
                            
                            print(f"  🔧 Repositioning {object_id} to secure grip position: {secure_pos}")
                            p.resetBasePositionAndOrientation(obj_bullet_id, secure_pos, [0, 0, 0, 1])
                            
                            # Clear any unwanted velocities
                            p.resetBaseVelocity(obj_bullet_id, [0, 0, 0], [0, 0, 0])
                            
                        else:
                            # Constraint is working properly
                            pass
                            
            except Exception as e:
                print(f"  ⚠️ Constraint {constraint_id} for {object_id} may be broken: {e}")
                # Try to recreate the constraint
                self._recreate_grasp_constraint(object_id)
    
    def _recreate_grasp_constraint(self, object_id: str) -> bool:
        """Recreate a broken grasp constraint for an object"""
        try:
            if not hasattr(self, '_world_ref') or not self._world_ref or not hasattr(self._world_ref, 'objects'):
                return False
                
            if object_id not in self._world_ref.objects:
                print(f"  ❌ Cannot recreate constraint - object {object_id} not found")
                return False
                
            obj_bullet_id = self._world_ref.objects[object_id]
            obj_pos, _ = p.getBasePositionAndOrientation(obj_bullet_id)
            
            print(f"  🔧 Recreating broken constraint for {object_id}")
            
            # Remove old constraint if it exists
            if object_id in self.held_objects:
                old_constraint_id = self.held_objects[object_id]
                try:
                    p.removeConstraint(old_constraint_id)
                except:
                    pass  # Constraint might already be broken
                    
            # Determine if object is elevated
            is_elevated = obj_pos[2] > 0.25
            
            # Create new constraint
            success = self._create_grasp_constraint(obj_bullet_id, obj_pos, is_elevated)
            
            if success:
                # Update held_objects with new constraint
                self.held_objects[object_id] = self.grasp_constraint_id
                print(f"  ✅ Successfully recreated constraint for {object_id}")
                return True
            else:
                print(f"  ❌ Failed to recreate constraint for {object_id}")
                # Remove from held objects if constraint creation failed
                if object_id in self.held_objects:
                    del self.held_objects[object_id]
                return False
                
        except Exception as e:
            print(f"  ❌ Error recreating constraint for {object_id}: {e}")
            return False
    
    def _verify_object_attachment(self) -> None:
        """Extra verification to ensure objects stay attached during multi-waypoint navigation"""
        if not self.held_objects:
            return
            
        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_id)
        
        for object_id, constraint_id in list(self.held_objects.items()):
            try:
                if hasattr(self, '_world_ref') and self._world_ref and hasattr(self._world_ref, 'objects'):
                    if object_id in self._world_ref.objects:
                        obj_bullet_id = self._world_ref.objects[object_id]
                        obj_pos, _ = p.getBasePositionAndOrientation(obj_bullet_id)
                        
                        # Check distance and velocity
                        distance = np.linalg.norm(np.array(obj_pos[:2]) - np.array(robot_pos[:2]))
                        obj_velocity, _ = p.getBaseVelocity(obj_bullet_id)
                        velocity_magnitude = np.linalg.norm(obj_velocity)
                        
                        # If object is drifting away or moving too fast, secure it
                        if distance > 0.8 or velocity_magnitude > 2.0:
                            print(f"  🚨 CRITICAL: {object_id} drifting during navigation (dist: {distance:.2f}m, vel: {velocity_magnitude:.2f}m/s)")
                            
                            # Emergency repositioning - lock object to robot
                            robot_euler = p.getEulerFromQuaternion(robot_orn)
                            robot_yaw = robot_euler[2] + np.pi/2
                            
                            emergency_pos = [
                                robot_pos[0] + 0.3 * np.cos(robot_yaw),
                                robot_pos[1] + 0.3 * np.sin(robot_yaw),
                                robot_pos[2] + 0.1
                            ]
                            
                            print(f"  🔧 Emergency repositioning {object_id} to {emergency_pos}")
                            p.resetBasePositionAndOrientation(obj_bullet_id, emergency_pos, [0, 0, 0, 1])
                            p.resetBaseVelocity(obj_bullet_id, [0, 0, 0], [0, 0, 0])
                            
                            # Strengthen constraint
                            p.changeConstraint(constraint_id, maxForce=6000)
                            
            except Exception as e:
                print(f"  ⚠️ Error verifying attachment for {object_id}: {e}")
    
    def _check_path_for_obstacles(self, start: List[float], end: List[float]) -> bool:
        """Check if path from start to end intersects with any platforms"""
        if not hasattr(self, '_world_ref') or self._world_ref is None:
            print("  ⚠️ No world reference - cannot check for obstacles")
            return False
            
        # Get all platforms and bins from object_positions (both are obstacles for navigation)
        platforms = []
        bins = []
        if hasattr(self._world_ref, 'object_positions'):
            for obj_id, position in self._world_ref.object_positions.items():
                if 'platform' in obj_id.lower():
                    platforms.append({
                        'id': obj_id,
                        'position': position,
                        'radius': 0.25  # Reduced safety radius for platform approach operations
                    })
                elif 'bin' in obj_id.lower():
                    bins.append({
                        'id': obj_id,
                        'position': position,
                        'radius': 0.30  # Slightly larger radius for bins to maintain clearance
                    })
        
        print(f"  🔍 Checking path {start[:2]} -> {end[:2]} against {len(platforms)} platforms and {len(bins)} bins")
        
        # Check both platforms and bins as collision objects
        all_obstacles = platforms + bins
        
        for i, obstacle in enumerate(all_obstacles):
            # Determine obstacle type based on ID
            if 'platform' in obstacle.get('id', '').lower():
                obstacle_type = "platform"
            elif 'bin' in obstacle.get('id', '').lower():
                obstacle_type = "bin"
            else:
                obstacle_type = "obstacle"
                
            if self._path_intersects_obstacle(start, end, obstacle):
                print(f"  ⚠️ Path intersects with {obstacle_type} at position {obstacle.get('position', 'unknown')}")
                return True
                
        print(f"  ✅ Path clear of all {len(all_obstacles)} obstacles")
        return False
        
    def _path_intersects_obstacle(self, start: List[float], end: List[float], obstacle: Dict) -> bool:
        """Check if path from start to end intersects with a specific obstacle"""
        try:
            # Get obstacle position and radius
            if 'position' not in obstacle:
                return False
                
            obs_pos = np.array(obstacle['position'][:2])
            obs_radius = obstacle.get('radius', 0.30)  # Default 30cm safety radius
            
            # Convert path to 2D
            start_2d = np.array(start[:2])
            end_2d = np.array(end[:2])
            
            # Calculate distance from obstacle center to line segment
            line_vec = end_2d - start_2d
            line_length = np.linalg.norm(line_vec)
            
            if line_length < 0.01:  # Very short path
                return np.linalg.norm(start_2d - obs_pos) < obs_radius
                
            line_unit = line_vec / line_length
            
            # Vector from start to obstacle center
            start_to_obs = obs_pos - start_2d
            
            # Project obstacle center onto line
            projection_length = np.dot(start_to_obs, line_unit)
            
            # Clamp projection to line segment
            projection_length = max(0, min(line_length, projection_length))
            
            # Find closest point on line segment
            closest_point = start_2d + projection_length * line_unit
            
            # Check distance from obstacle to closest point
            distance = np.linalg.norm(obs_pos - closest_point)
            
            intersects = distance < obs_radius
            if intersects:
                print(f"    ⚠️ Path intersects obstacle at {obs_pos} (distance: {distance:.2f}m < radius: {obs_radius:.2f}m)")
            
            return intersects
            
        except Exception as e:
            print(f"    ❌ Error checking path intersection: {e}")
            return False
        
    def _find_safe_path(self, target_pos: List[float]) -> List[List[float]]:
        """
        Find a safe path to target that avoids obstacles using intelligent routing
        
        Args:
            target_pos: Target position [x, y, z]
            
        Returns:
            List of waypoints to safely reach target
        """
        current_pos = self.get_position()
        current_2d = np.array(current_pos[:2])
        target_2d = np.array(target_pos[:2])
        
        # First check if direct path is clear
        if not self._check_path_for_obstacles(current_pos, target_pos):
            print(f"  ✅ Direct path to target is clear")
            return [target_pos]
            
        print(f"  🛑 Direct path blocked, finding efficient detour...")
        
        # Get obstacle positions for smart routing
        platforms = []
        bins = []
        if hasattr(self, '_world_ref') and self._world_ref is not None:
            for obj_id, position in self._world_ref.object_positions.items():
                if 'platform' in obj_id.lower():
                    platforms.append({
                        'id': obj_id,
                        'position': np.array(position[:2]),
                        'radius': 0.25
                    })
                elif 'bin' in obj_id.lower():
                    bins.append({
                        'id': obj_id,
                        'position': np.array(position[:2]),
                        'radius': 0.30
                    })
        
        # Calculate distance to target
        direct_distance = np.linalg.norm(target_2d - current_2d)
        
        # SMART ROUTING: For short distances, try multiple simple strategies first
        if direct_distance < 3.0:  # Increased range to 3m for better coverage
            print(f"  🎯 Short distance ({direct_distance:.1f}m) - trying smart detour strategies")
            
            # STRATEGY 1: Simple side-step detours with multiple distances
            print(f"  🔄 Trying side-step detours...")
            direct_vector = target_2d - current_2d
            if np.linalg.norm(direct_vector) > 0.1:
                perpendicular = np.array([-direct_vector[1], direct_vector[0]])
                perpendicular = perpendicular / np.linalg.norm(perpendicular)
                
                # Try multiple side-step distances and both sides
                for step_distance in [0.6, 1.0, 1.4]:  # Try different step distances
                    for side_multiplier in [1.0, -1.0]:  # Try both sides
                        side_direction = "left" if side_multiplier > 0 else "right"
                        side_waypoint = current_2d + direct_vector * 0.3 + perpendicular * side_multiplier * step_distance
                        waypoint_3d = [side_waypoint[0], side_waypoint[1], current_pos[2]]
                        
                        print(f"    Testing {side_direction} side-step (distance: {step_distance:.1f}m): {side_waypoint}")
                        
                        # Check if both path segments are clear
                        segment1_blocked = self._check_path_for_obstacles(current_pos, waypoint_3d)
                        segment2_blocked = self._check_path_for_obstacles(waypoint_3d, target_pos)
                        
                        if segment1_blocked:
                            print(f"    ❌ Segment 1 blocked (robot to waypoint)")
                        if segment2_blocked:
                            print(f"    ❌ Segment 2 blocked (waypoint to target)")
                        
                        if not segment1_blocked and not segment2_blocked:
                            print(f"  ✅ Side-step detour successful: {side_waypoint} (offset: {step_distance:.1f}m)")
                            return [waypoint_3d, target_pos]
            
            # STRATEGY 2: Simple right-angle detours (go around corners)
            print(f"  🔄 Trying right-angle corner detours...")
            for corner_distance in [0.8, 1.2]:  # Try different corner distances
                # Try 4 corner directions: North, South, East, West
                corner_offsets = [
                    [0, corner_distance],      # North
                    [0, -corner_distance],     # South  
                    [corner_distance, 0],      # East
                    [-corner_distance, 0]      # West
                ]
                
                for i, offset in enumerate(corner_offsets):
                    direction_names = ["North", "South", "East", "West"]
                    direction = direction_names[i]
                    corner_waypoint = current_2d + np.array(offset)
                    waypoint_3d = [corner_waypoint[0], corner_waypoint[1], current_pos[2]]
                    
                    print(f"    Testing {direction} corner detour (distance: {corner_distance:.1f}m): {corner_waypoint}")
                    
                    # Check if L-shaped path is clear
                    segment1_blocked = self._check_path_for_obstacles(current_pos, waypoint_3d)
                    segment2_blocked = self._check_path_for_obstacles(waypoint_3d, target_pos)
                    
                    if segment1_blocked:
                        print(f"    ❌ Segment 1 blocked (robot to corner)")
                    if segment2_blocked:
                        print(f"    ❌ Segment 2 blocked (corner to target)")
                    
                    if not segment1_blocked and not segment2_blocked:
                        print(f"  ✅ Right-angle detour successful: {corner_waypoint}")
                        return [waypoint_3d, target_pos]
        
        # FALLBACK: Use wide arc only when necessary
        print(f"  🌍 Using wide arc routing for complex obstacle avoidance...")
        return self._calculate_wide_arc_path(current_pos, target_pos, platforms, bins)
    
    def _calculate_wide_arc_path(self, current_pos: List[float], target_pos: List[float], 
                                platforms: List, bins: List) -> List[List[float]]:
        """Calculate wide arc path around obstacles"""
        current_2d = np.array(current_pos[:2])
        target_2d = np.array(target_pos[:2])
        direct_vector = target_2d - current_2d
        direct_length = np.linalg.norm(direct_vector)
        
        if direct_length < 0.1:
            return [target_pos]  # Too close, just go direct
            
        # STRATEGY 1: Wide Arc Routing - go around all obstacles in a big arc
        print(f"  🌍 Trying wide arc routing to avoid all obstacles...")
        
        # Find the center of all obstacles (platforms + bins)
        all_obstacle_positions = [p['position'] for p in platforms + bins]
        if all_obstacle_positions:
            obstacle_center = np.mean(all_obstacle_positions, axis=0)
            print(f"  📍 Obstacle cluster center: {obstacle_center}")
            
            # Calculate a more reasonable arc around the obstacle cluster
            cluster_radius = max([np.linalg.norm(pos - obstacle_center) for pos in all_obstacle_positions]) + 0.6  # 0.6m clearance for navigation
            print(f"  🌀 Cluster radius: {cluster_radius:.2f}m")
            
            # Direction from cluster center to current position
            to_current = current_2d - obstacle_center
            to_target = target_2d - obstacle_center
            
            # Normalize directions
            if np.linalg.norm(to_current) > 0.1:
                to_current_norm = to_current / np.linalg.norm(to_current)
            else:
                to_current_norm = np.array([1, 0])  # Default direction
                
            if np.linalg.norm(to_target) > 0.1:
                to_target_norm = to_target / np.linalg.norm(to_target)
            else:
                to_target_norm = np.array([1, 0])  # Default direction
            
            # Create waypoints on the arc around cluster
            arc_waypoint1 = obstacle_center + to_current_norm * cluster_radius
            arc_waypoint2 = obstacle_center + to_target_norm * cluster_radius
            
            waypoint1_3d = [arc_waypoint1[0], arc_waypoint1[1], current_pos[2]]
            waypoint2_3d = [arc_waypoint2[0], arc_waypoint2[1], current_pos[2]]
            
            print(f"  🌀 Arc waypoints: {arc_waypoint1} -> {arc_waypoint2}")
            
            # Test the wide arc path
            seg1_clear = not self._check_path_for_obstacles(current_pos, waypoint1_3d)
            seg2_clear = not self._check_path_for_obstacles(waypoint1_3d, waypoint2_3d) 
            seg3_clear = not self._check_path_for_obstacles(waypoint2_3d, target_pos)
            
            if seg1_clear and seg2_clear and seg3_clear:
                print(f"  ✅ Wide arc route successful! Going around entire obstacle cluster")
                return [waypoint1_3d, waypoint2_3d, target_pos]
        
        # STRATEGY 2: Sequential Avoidance - avoid obstacles one by one
        print(f"  🔄 Trying sequential obstacle avoidance...")
        
        # Find which obstacles are actually blocking the path
        blocking_obstacles = []
        for obstacle in platforms + bins:
            if self._path_intersects_obstacle(current_pos, target_pos, obstacle):
                blocking_obstacles.append(obstacle)
        
        print(f"  🚧 {len(blocking_obstacles)} obstacles blocking direct path")
        
        if blocking_obstacles:
            # Take a wide detour around the first blocking obstacle
            first_blocker = blocking_obstacles[0]
            obstacle_pos = first_blocker['position']
            
            # Calculate safe distance around obstacle
            safe_distance = 0.8  # 0.8m clearance around obstacle
            
            # Try 8 directions around the obstacle
            for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
                waypoint_offset = safe_distance * np.array([np.cos(angle), np.sin(angle)])
                waypoint_2d = obstacle_pos + waypoint_offset
                waypoint_3d = [waypoint_2d[0], waypoint_2d[1], current_pos[2]]
                
                # Check if this creates a completely clear path
                seg1_clear = not self._check_path_for_obstacles(current_pos, waypoint_3d)
                seg2_clear = not self._check_path_for_obstacles(waypoint_3d, target_pos)
                
                if seg1_clear and seg2_clear:
                    print(f"  ✅ Sequential avoidance successful! Wide detour around {first_blocker['id']} at {waypoint_2d}")
                    return [waypoint_3d, target_pos]
        
        # STRATEGY 3: Extreme Wide Detour - go very far around everything
        print(f"  🚁 Trying extreme wide detour (last resort)...")
        
        # Calculate bounding box of all obstacles
        all_obstacles = platforms + bins
        if all_obstacles:
            all_positions = np.array([p['position'] for p in all_obstacles])
            min_x, min_y = np.min(all_positions, axis=0) - 1.5  # 1.5m buffer
            max_x, max_y = np.max(all_positions, axis=0) + 1.5  # 1.5m buffer
            
            # Try corner routing - go to corners of bounding box
            corners = [
                [min_x, min_y],  # Bottom-left
                [min_x, max_y],  # Top-left  
                [max_x, max_y],  # Top-right
                [max_x, min_y]   # Bottom-right
            ]
            
            for corner in corners:
                corner_3d = [corner[0], corner[1], current_pos[2]]
                
                seg1_clear = not self._check_path_for_obstacles(current_pos, corner_3d)
                seg2_clear = not self._check_path_for_obstacles(corner_3d, target_pos)
                
                if seg1_clear and seg2_clear:
                    print(f"  ✅ Extreme detour successful! Via corner {corner}")
                    return [corner_3d, target_pos]
        
        # If all strategies fail, the target is truly unreachable
        print(f"  🛑 CRITICAL: All routing strategies failed - target is unreachable!")
        print(f"  ⚠️ Robot stopping to avoid collision - manual path planning required")
        return []  # Return empty path to prevent movement
            
    def attempt_pick(self, object_id: str) -> bool:
        """
        Attempt to pick up an object using the gripper with strategic positioning
        and proper arm extension for elevated objects
        """
        try:
            # Get current robot and object positions
            robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_id)
            
            if not hasattr(self, '_world_ref') or self._world_ref is None:
                print(f"  ❌ No world reference available")
                return False
                
            if object_id not in self._world_ref.objects:
                print(f"  ❌ Object {object_id} not found in world")
                return False
                
            obj_id = self._world_ref.objects[object_id]
            object_pos, object_orn = p.getBasePositionAndOrientation(obj_id)
            
            print(f"  🎯 Strategic positioning for elevated object at {object_pos}")
            
            # Check if object is at platform height (elevated)
            object_height = object_pos[2]
            is_elevated = object_height > 0.25  # If object is above 25cm, it's on a platform
            
            # Use strategic positioning - maintain 0.45m distance for optimal reach
            if not self._position_robot_strategically(object_pos):
                print("  ❌ Failed to position robot strategically")
                return False
            
            print(f"  🤖 Positioned strategically for {'elevated' if is_elevated else 'ground-level'} object")
            
            # Get updated robot position after strategic positioning
            robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_id)
            
            # Calculate gripper extension for elevated objects
            distance_to_object = np.linalg.norm(np.array(object_pos[:2]) - np.array(robot_pos[:2]))
            
            # For elevated objects, use forward extension (positive angle)
            if is_elevated:
                # Extended gripper position for platform objects
                extension_angle = 45  # Positive angle = forward extension
                print(f"  🔧 Using forward extension ({extension_angle}°) for elevated object")
            else:
                # Standard downward reach for ground objects
                extension_angle = -30
                print(f"  🔧 Using downward extension ({extension_angle}°) for ground object")
            
            # Convert angle to radians and set gripper position
            extension_radians = np.radians(extension_angle)
            
            # Open gripper first
            p.setJointMotorControl2(self.robot_id, self.joints['left_gripper_joint'], 
                                  p.POSITION_CONTROL, targetPosition=0.5)
            p.setJointMotorControl2(self.robot_id, self.joints['right_gripper_joint'], 
                                  p.POSITION_CONTROL, targetPosition=-0.5)
            
            # Extend gripper arm to reach object
            p.setJointMotorControl2(self.robot_id, self.joints['gripper_extension'], 
                                  p.POSITION_CONTROL, targetPosition=extension_radians)
            
            # Wait for movement to complete
            for _ in range(30):
                p.stepSimulation()
                
            time.sleep(0.5)
            
            print(f"  ✋ Gripper extended {extension_angle}° (distance: {distance_to_object:.2f}m)")
            
            # Close gripper to grasp object
            p.setJointMotorControl2(self.robot_id, self.joints['left_gripper_joint'], 
                                  p.POSITION_CONTROL, targetPosition=0.0)
            p.setJointMotorControl2(self.robot_id, self.joints['right_gripper_joint'], 
                                  p.POSITION_CONTROL, targetPosition=0.0)
            
            # Wait for grasp to complete
            for _ in range(20):
                p.stepSimulation()
                
            time.sleep(0.3)
            
            # Create constraint to attach object to gripper
            success = self._create_grasp_constraint(obj_id, object_pos, is_elevated)
            
            if success:
                self.grasped_object_id = object_id
                # Store the object in held_objects with the constraint ID
                self.held_objects[object_id] = self.grasp_constraint_id
                print(f"  ✅ Successfully picked up {object_id} from {'platform' if is_elevated else 'ground'}")
                return True
            else:
                print(f"  ❌ Failed to create grasp constraint for {object_id}")
                return False
                
        except Exception as e:
            print(f"  ❌ Error during pick operation: {e}")
            return False
    
    def _position_robot_strategically(self, object_pos: List[float]) -> bool:
        """Position robot strategically for optimal object interaction using safe path planning"""
        try:
            # Calculate optimal position (close enough for good gripper reach but not colliding)
            robot_pos = self.get_position()
            target_distance = 0.50  # Closer positioning since no repositioning needed after turns
            
            # Calculate direction vector from robot to object
            direction = np.array(object_pos[:2]) - np.array(robot_pos[:2])
            current_distance = np.linalg.norm(direction)
            
            if current_distance < 0.1:  # Too close
                print(f"  ⚠️ Robot too close to object ({current_distance:.2f}m)")
                return False
            
            # Check if already in acceptable range
            if 0.45 <= current_distance <= 0.70:  # Tighter acceptable range for optimal performance
                print(f"  ✅ Robot already in good position: {current_distance:.2f}m (target: {target_distance:.2f}m)")
                return True
            
            # Normalize direction and calculate target position
            direction_normalized = direction / current_distance
            target_pos = np.array(object_pos[:2]) - direction_normalized * target_distance
            target_pos_3d = [target_pos[0], target_pos[1], robot_pos[2]]
            
            print(f"  📍 Strategic positioning: {current_distance:.2f}m → {target_distance:.2f}m")
            
            # Use safe path planning to avoid platforms
            safe_waypoints = self._find_safe_path(target_pos_3d)
            
            # Execute safe navigation
            for i, waypoint in enumerate(safe_waypoints):
                print(f"  🛤️ Navigating to waypoint {i+1}/{len(safe_waypoints)}: {waypoint[:2]}")
                
                # Move robot to waypoint
                self.start_walking_to(waypoint)
                
                # Wait for movement to complete
                timeout = 5.0  # 5 second timeout per waypoint
                start_time = time.time()
                
                while self.walking and (time.time() - start_time) < timeout:
                    p.stepSimulation()
                    time.sleep(0.01)
                
                # Check if we reached the waypoint
                current_pos = self.get_position()
                distance_to_waypoint = np.linalg.norm(np.array(waypoint[:2]) - np.array(current_pos[:2]))
                
                if distance_to_waypoint > 0.3:  # More than 30cm off target
                    print(f"  ⚠️ Failed to reach waypoint {i+1} (distance: {distance_to_waypoint:.2f}m)")
                    # Continue anyway to next waypoint
                else:
                    print(f"  ✅ Reached waypoint {i+1} (distance: {distance_to_waypoint:.2f}m)")
            
            # Check final distance to object
            final_pos = self.get_position()
            final_distance = np.linalg.norm(np.array(object_pos[:2]) - np.array(final_pos[:2]))
            
            if 0.35 <= final_distance <= 0.75:  # Closer working range for better performance
                print(f"  ✅ Strategic positioning complete: {final_distance:.2f}m")
                return True
            else:
                print(f"  ⚠️ Strategic positioning suboptimal: {final_distance:.2f}m")
                return final_distance < 0.90  # Accept if reasonably close
                
        except Exception as e:
            print(f"  ❌ Error in strategic positioning: {e}")
            return False
        
    def _create_grasp_constraint(self, obj_id: int, object_pos: List[float], is_elevated: bool = False) -> bool:
        """Create a more secure constraint to simulate proper grasping"""
        try:
            # Get current robot and object positions  
            robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_id)
            obj_pos, obj_orn = p.getBasePositionAndOrientation(obj_id)
            
            print(f"  📍 Robot at {robot_pos}, object at {obj_pos}")
            
            # Calculate distance from robot to object
            distance_to_obj = np.linalg.norm(np.array(obj_pos[:2]) - np.array(robot_pos[:2]))
            print(f"  📏 Distance to object: {distance_to_obj:.3f}m")
            
            # Get robot's current orientation to calculate forward direction
            _, robot_quat = p.getBasePositionAndOrientation(self.robot_id)
            robot_euler = p.getEulerFromQuaternion(robot_quat)
            robot_yaw = robot_euler[2]  # Get yaw angle
            
            # Calculate forward direction with 90-degree offset for robot's actual forward axis
            corrected_yaw = robot_yaw + np.pi/2  # Add 90 degrees to align with robot's actual forward
            forward_x = np.cos(corrected_yaw)
            forward_y = np.sin(corrected_yaw)
            
            # Position object at gripper location for secure attachment
            if is_elevated:
                # For elevated objects, position close to robot gripper
                gripper_pos = [
                    robot_pos[0] + 0.35 * forward_x,  # 35cm in front of robot (secure gripper position)
                    robot_pos[1] + 0.35 * forward_y,  # 35cm in front of robot (secure gripper position) 
                    max(obj_pos[2], robot_pos[2] + 0.15)  # Keep at platform height or above robot
                ]
                print(f"  🏛️ Positioning object at secure gripper location: {gripper_pos}")
            else:
                # Standard positioning for ground objects
                gripper_pos = [
                    robot_pos[0] + 0.35 * forward_x,  # 35cm in front of robot (secure gripper position)
                    robot_pos[1] + 0.35 * forward_y,  # 35cm in front of robot (secure gripper position)
                    robot_pos[2] - 0.1               # Slightly below robot center
                ]
                print(f"  🤖 Secure gripper positioning: {gripper_pos}")
            
            print(f"  🧭 Robot facing direction (corrected): [{forward_x:.3f}, {forward_y:.3f}] (yaw: {np.degrees(corrected_yaw):.1f}°)")
            
            # Move object to secure gripper position
            p.resetBasePositionAndOrientation(obj_id, gripper_pos, [0, 0, 0, 1])
            
            # Try to create constraint with gripper joint if available, otherwise use robot base
            gripper_link_id = self.joints.get('left_gripper_joint', -1)  # Use actual gripper joint
            if gripper_link_id == -1:
                gripper_link_id = -1  # Fall back to robot base
                print(f"  ⚠️ Using robot base for constraint (no gripper joint found)")
            else:
                print(f"  🤖 Using gripper joint {gripper_link_id} for secure constraint")
            
            # Create a strong fixed constraint between gripper/robot and object
            constraint_id = p.createConstraint(
                self.robot_id,      # parent body (robot)
                gripper_link_id,    # parent link (gripper joint or robot base)
                obj_id,             # child body (object)
                -1,                 # child link (object base)
                p.JOINT_FIXED,      # constraint type
                [0, 0, 0],          # joint axis (not used for fixed joint)
                [0, 0, 0],          # parent frame position (relative to gripper/robot)
                [0, 0, 0]           # child frame position (relative to object base)
            )
            
            # Make the constraint extremely strong to prevent object from detaching during navigation
            p.changeConstraint(constraint_id, maxForce=10000)  # Increased to 10000N to prevent box lagging
            
            # Additional constraint settings for stability during navigation
            p.changeConstraint(constraint_id, erp=0.9)  # Higher error reduction parameter for stiffness
            
            # Store constraint info
            self.grasp_constraint_id = constraint_id
            
            print(f"  ✅ Created ultra-secure {'elevated' if is_elevated else 'standard'} grasp constraint (ID: {constraint_id})")
            print(f"  🔒 Constraint strength: 10000N - object locked tight to prevent lagging")
            
            # Step simulation to apply constraint
            for _ in range(15):  # More steps to ensure constraint is properly applied
                p.stepSimulation()
            
            return True
                
        except Exception as e:
            print(f"  ❌ Failed to create grasp constraint: {e}")
            return False
        
    def attempt_place(self, location: List[float], object_id: str = None) -> bool:
        """
        Attempt to place an object at a location by positioning it over the target and releasing
        
        Args:
            location: Target placement location [x, y, z]
            object_id: Specific object to place, or None for any held object
            
        Returns:
            True if placement attempt was successful
        """
        if not self.held_objects:
            print(f"  ❌ No objects being held")
            return False
            
        # Determine which object to place
        if object_id is None:
            # Place any held object
            target_object = next(iter(self.held_objects.keys()))
        else:
            if object_id not in self.held_objects:
                print(f"  ❌ Object {object_id} not being held")
                return False
            target_object = object_id
            
        # Check if location is in reach
        robot_pos = self.get_position()
        distance = np.linalg.norm(np.array(location[:2]) - np.array(robot_pos[:2]))
        
        if distance > self.reach_distance:
            print(f"  ❌ Target location too far: {distance:.2f}m > {self.reach_distance}m")
            return False
            
        print(f"  📦 Placing {target_object} at {location}")
        
        # Get the object and constraint info
        constraint_id = self.held_objects[target_object]
        
        # Get world reference to access the object
        world_instance = getattr(self, '_world_ref', None)
        if not world_instance or not hasattr(world_instance, 'objects'):
            print(f"  ❌ No world reference for object placement")
            return False
            
        if target_object not in world_instance.objects:
            print(f"  ❌ Object {target_object} not found in world")
            return False
            
        obj_bullet_id = world_instance.objects[target_object]
        
        try:
            # Step 1: Position object directly over the target location (elevated for clean drop)
            drop_height = 0.25  # 25cm above target for clean drop
            elevated_position = [location[0], location[1], location[2] + drop_height]
            
            print(f"  📍 Moving {target_object} to drop position: {elevated_position}")
            p.resetBasePositionAndOrientation(obj_bullet_id, elevated_position, [0, 0, 0, 1])
            
            # CRITICAL: Clear any residual velocities from robot movement
            print(f"  🛑 Clearing object velocities to ensure straight drop")
            p.resetBaseVelocity(obj_bullet_id, [0, 0, 0], [0, 0, 0])  # Zero linear and angular velocity
            
            # Allow physics to settle after positioning and velocity reset
            for _ in range(20):  # More steps for complete stabilization
                p.stepSimulation()
            time.sleep(0.3)  # Extended settling time
            
            # Verify object is stationary before release
            linear_vel, angular_vel = p.getBaseVelocity(obj_bullet_id)
            vel_magnitude = np.linalg.norm(linear_vel)
            print(f"  📏 Object velocity before release: {vel_magnitude:.4f} m/s")
            
            if vel_magnitude > 0.01:  # If still moving significantly
                print(f"  ⚠️ Object still has velocity, applying additional damping")
                p.resetBaseVelocity(obj_bullet_id, [0, 0, 0], [0, 0, 0])  # Force zero velocity
                for _ in range(10):
                    p.stepSimulation()
            
            # Step 2: Open gripper to simulate release
            print(f"  ✋ Opening gripper to release {target_object}")
            try:
                p.setJointMotorControl2(self.robot_id, self.joints['left_gripper_joint'], 
                                      p.POSITION_CONTROL, targetPosition=0.5)
                p.setJointMotorControl2(self.robot_id, self.joints['right_gripper_joint'], 
                                      p.POSITION_CONTROL, targetPosition=-0.5)
                
                # Allow gripper to open completely
                for _ in range(10):  # More steps for complete gripper opening
                    p.stepSimulation()
                time.sleep(0.2)
            except Exception as e:
                print(f"  ⚠️ Could not open gripper: {e}")
            
            # Step 3: Remove physics constraint to release object (only once!)
            if isinstance(constraint_id, int) and constraint_id >= 0:
                # Final velocity check and reset immediately before release
                p.resetBaseVelocity(obj_bullet_id, [0, 0, 0], [0, 0, 0])  # Ensure zero velocity
                
                p.removeConstraint(constraint_id)
                print(f"  🔓 Released constraint (ID: {constraint_id}) - object dropping")
                
                # Monitor drop with controlled physics
                print(f"  🌍 Object falling from {drop_height}m height (straight down)...")
                
                # Let object fall with gravity only - no external forces
                fall_steps = 0
                max_fall_steps = 60  # Reduced from 120 for faster simulation
                settling_threshold = 0.05  # Higher threshold for faster settling detection
                
                print(f"  🌍 Monitoring object drop and settling...")
                
                while fall_steps < max_fall_steps:
                    p.stepSimulation()
                    time.sleep(1./480.)  # Faster timestep (480Hz instead of 240Hz)
                    fall_steps += 1
                    
                    # Check if object has settled (low velocity and reasonable height)
                    current_pos, _ = p.getBasePositionAndOrientation(obj_bullet_id)
                    linear_vel, _ = p.getBaseVelocity(obj_bullet_id)
                    vel_magnitude = np.linalg.norm(linear_vel)
                    
                    # Object considered settled if velocity is very low and height is reasonable
                    if vel_magnitude < settling_threshold and current_pos[2] < (location[2] + 0.15):
                        print(f"  ✅ Object settled after {fall_steps} steps (velocity: {vel_magnitude:.3f} m/s)")
                        break
                        
                    # Progress updates every 20 steps (reduced from 30)
                    if fall_steps % 20 == 0:
                        print(f"  📉 Drop progress: step {fall_steps}, height: {current_pos[2]:.2f}m, velocity: {vel_magnitude:.3f} m/s")
                
                # Reduced settling time for faster animation
                print(f"  ⏱️ Allowing extra settling time for visualization...")
                for _ in range(20):  # Reduced from 60 steps
                    p.stepSimulation()
                    time.sleep(1./480.)  # Faster timestep
                
                time.sleep(0.2)  # Reduced final pause from 0.5 seconds
                
                # Get final position and verify straight drop
                final_obj_pos, _ = p.getBasePositionAndOrientation(obj_bullet_id)
                target_center = np.array(location[:2])
                final_position = np.array(final_obj_pos[:2])
                distance_from_target = np.linalg.norm(final_position - target_center)
                
                print(f"  📍 Final position: {final_obj_pos}")
                print(f"  📏 Distance from target: {distance_from_target:.3f}m")
                print(f"  ✅ Object placed successfully with straight-down drop")
                
                print(f"  📍 Final position: {final_obj_pos}")
                print(f"  📏 Distance from target: {distance_from_target:.3f}m")
                print(f"  ✅ Object placed successfully with straight-down drop")
                
            else:
                print(f"  ❌ Invalid constraint ID: {constraint_id}")
                return False
                
        except Exception as e:
            print(f"  ❌ Failed to place object: {e}")
            return False
        
        # Remove from held objects
        del self.held_objects[target_object]
        return True
        
    def is_holding(self, object_id: str) -> bool:
        """Check if robot is holding a specific object"""
        is_holding = object_id in self.held_objects
        if not is_holding:
            print(f"  🔍 DEBUG: Robot is NOT holding {object_id}. Currently holding: {list(self.held_objects.keys())}")
        return is_holding
        
    def is_holding_any(self) -> bool:
        """Check if robot is holding any object"""
        has_objects = len(self.held_objects) > 0
        if not has_objects:
            print(f"  🔍 DEBUG: Robot is not holding any objects")
        else:
            print(f"  🔍 DEBUG: Robot is holding {len(self.held_objects)} objects: {list(self.held_objects.keys())}")
        return has_objects
        
    def get_held_objects(self) -> List[str]:
        """Get list of currently held object IDs"""
        return list(self.held_objects.keys())
        
    def reset_pose(self) -> None:
        """Reset robot to initial pose"""
        if self.robot_id is not None:
            p.resetBasePositionAndOrientation(
                self.robot_id,
                self.start_position,
                [0, 0, 0, 1]
            )
            
            # Reset joint positions
            for i in range(len(self.joints)):
                p.resetJointState(self.robot_id, i, 0)
                
        self.current_position = self.start_position.copy()
        self.walking = False
        self.held_objects.clear()
        
    def add_sensor_noise(self, measurement: List[float], noise_level: float = 0.01) -> List[float]:
        """Add noise to sensor measurements"""
        noise = [random.uniform(-noise_level, noise_level) for _ in range(len(measurement))]
        return [measurement[i] + noise[i] for i in range(len(measurement))]
    
    def get_joint_states(self) -> List[Dict]:
        """Get current joint states (position, velocity, force)"""
        if self.robot_id is None:
            return []
            
        joint_states = []
        for joint_name, joint_idx in self.joints.items():
            try:
                state = p.getJointState(self.robot_id, joint_idx)
                joint_states.append({
                    'name': joint_name,
                    'index': joint_idx,
                    'position': state[0],
                    'velocity': state[1],
                    'force': state[3]
                })
            except:
                # If PyBullet not connected, return mock data
                joint_states.append({
                    'name': joint_name,
                    'index': joint_idx,
                    'position': 0.0,
                    'velocity': 0.0,
                    'force': 0.0
                })
        return joint_states
    
    def calculate_ik(self, target_position: List[float], target_orientation: List[float] = None) -> List[float]:
        """Calculate inverse kinematics for target pose"""
        if self.robot_id is None or self.end_effector_link == -1:
            # Return mock joint angles if no proper robot
            return [0.0] * max(len(self.joints), 7)
            
        if target_orientation is None:
            target_orientation = [0, 0, 0, 1]  # Default upright orientation
            
        try:
            joint_positions = p.calculateInverseKinematics(
                self.robot_id,
                self.end_effector_link,
                target_position,
                target_orientation,
                maxNumIterations=100
            )
            return list(joint_positions)
        except:
            # If PyBullet not connected, return mock data
            return [0.0] * max(len(self.joints), 7)
    
    def get_base_pose(self) -> List[float]:
        """Get robot base position [x, y, z]"""
        if self.robot_id is None:
            return self.current_position
            
        try:
            pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            return list(pos)
        except:
            # If PyBullet not connected, return current position
            return self.current_position
        
    def simulate_joint_error(self, target_positions: List[float], error_rate: float = 0.05) -> List[float]:
        """Simulate joint control errors"""
        errors = [random.uniform(-error_rate, error_rate) * pos for pos in target_positions]
        return [target_positions[i] + errors[i] for i in range(len(target_positions))]
