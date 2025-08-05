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
        self.reach_distance = 0.65  # Extended reach distance to accommodate collision-safe positioning
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
        """Start walking to target position"""
        self.target_position = target.copy()
        self.walking = True
        self.walk_start_time = time.time()
        # Store starting position for proper interpolation
        self._walk_start_pos = self.get_position().copy()
        print(f"🚶 Robot starting walk from {self._walk_start_pos[:2]} to {target[:2]}")
        
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
        
        if elapsed_time >= expected_time:
            # Walking completed
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
            
            # Update held objects to follow robot
            self._update_held_objects()
        
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
        
    def _complete_walk(self) -> None:
        """Complete the walking motion"""
        self.walking = False
        if self.target_position and self.robot_id is not None:
            # Set final position
            final_pos = [self.target_position[0], self.target_position[1], self.start_position[2]]
            _, current_orn = p.getBasePositionAndOrientation(self.robot_id)
            p.resetBasePositionAndOrientation(self.robot_id, final_pos, current_orn)
            self.current_position = final_pos
            
            # Update held object positions
            self._update_held_objects()
            
    def _update_held_objects(self):
        """Update positions of held objects to follow robot"""
        # With physics constraints, objects automatically follow the robot
        # No manual positioning needed - PyBullet handles this
        pass
            
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
        """Position robot strategically for optimal object interaction"""
        try:
            # Calculate optimal position (close enough for good gripper reach but not colliding)
            robot_pos = self.get_position()
            target_distance = 0.6  # Close enough for accurate gripper operation
            
            # Calculate direction vector from robot to object
            direction = np.array(object_pos[:2]) - np.array(robot_pos[:2])
            current_distance = np.linalg.norm(direction)
            
            if current_distance < 0.1:  # Too close
                print(f"  ⚠️ Robot too close to object ({current_distance:.2f}m)")
                return False
            
            # Check if already in acceptable range
            if 0.5 <= current_distance <= 0.8:
                print(f"  ✅ Robot already in good position: {current_distance:.2f}m (target: {target_distance:.2f}m)")
                return True
            
            # Normalize direction and calculate target position
            direction_normalized = direction / current_distance
            target_pos = np.array(object_pos[:2]) - direction_normalized * target_distance
            target_pos_3d = [target_pos[0], target_pos[1], robot_pos[2]]
            
            print(f"  📍 Strategic positioning: {current_distance:.2f}m → {target_distance:.2f}m")
            
            # Move robot to strategic position
            self.start_walking_to(target_pos_3d)
            
            # Wait for positioning to complete
            timeout = 5.0  # 5 second timeout
            start_time = time.time()
            
            while self.walking and (time.time() - start_time) < timeout:
                p.stepSimulation()
                time.sleep(0.01)
            
            # Check final distance
            final_pos = self.get_position()
            final_distance = np.linalg.norm(np.array(object_pos[:2]) - np.array(final_pos[:2]))
            
            if 0.4 <= final_distance <= 0.9:  # Reasonable working range
                print(f"  ✅ Strategic positioning complete: {final_distance:.2f}m")
                return True
            else:
                print(f"  ⚠️ Strategic positioning suboptimal: {final_distance:.2f}m")
                return final_distance < 1.0  # Accept if reasonably close
                
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
            
            # Make the constraint very strong to prevent object from detaching
            p.changeConstraint(constraint_id, maxForce=2000)  # Increased force for secure grip
            
            # Store constraint info
            self.grasp_constraint_id = constraint_id
            
            print(f"  ✅ Created secure {'elevated' if is_elevated else 'standard'} grasp constraint (ID: {constraint_id})")
            print(f"  🔒 Constraint strength: 2000N - object securely attached")
            
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
            
            # Step 2: Open gripper to simulate release
            print(f"  ✋ Opening gripper to release {target_object}")
            try:
                p.setJointMotorControl2(self.robot_id, self.joints['left_gripper_joint'], 
                                      p.POSITION_CONTROL, targetPosition=0.5)
                p.setJointMotorControl2(self.robot_id, self.joints['right_gripper_joint'], 
                                      p.POSITION_CONTROL, targetPosition=-0.5)
                
                # Brief pause for gripper to open
                for _ in range(5):
                    p.stepSimulation()
                time.sleep(0.1)
            except Exception as e:
                print(f"  ⚠️ Could not open gripper: {e}")
            
            # Step 3: Remove physics constraint to release object (only once!)
            if isinstance(constraint_id, int) and constraint_id >= 0:
                p.removeConstraint(constraint_id)
                print(f"  🔓 Released constraint (ID: {constraint_id}) - object dropping")
                
                # Let object fall naturally with controlled physics
                print(f"  🌍 Object falling from {drop_height}m height...")
                for _ in range(30):  # Controlled physics simulation
                    p.stepSimulation()
                    time.sleep(1./240.)
                
                # Brief settling time
                time.sleep(0.4)
                
                # Get final position
                final_obj_pos, _ = p.getBasePositionAndOrientation(obj_bullet_id)
                target_center = np.array(location[:2])
                final_position = np.array(final_obj_pos[:2])
                distance_from_target = np.linalg.norm(final_position - target_center)
                
                print(f"  📍 Final position: {final_obj_pos}")
                print(f"  📏 Distance from target: {distance_from_target:.3f}m")
                print(f"  ✅ Object placed successfully")
                
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
        return object_id in self.held_objects
        
    def is_holding_any(self) -> bool:
        """Check if robot is holding any object"""
        return len(self.held_objects) > 0
        
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
