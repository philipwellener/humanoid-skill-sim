"""
PickObject skill implementation with IK and manipulation logic
"""

import time
import numpy as np
import random
from typing import List, Optional, Dict, Any


class PickObjectSkill:
    """Skill for picking up objects using inverse kinematics"""
    
    def __init__(self, object_id: str, approach_distance: float = 0.05,
                 max_attempts: int = 3, timeout: float = 15.0, test_mode: bool = False):
        self.object_id = object_id
        self.approach_distance = approach_distance
        self.max_attempts = max_attempts
        self.timeout = timeout
        self.test_mode = test_mode  # Disable random failures in test mode
        self.attempts = 0
        self.start_time = None
        self.status = "IDLE"
        self.grasp_pose = None
        
    def execute(self, robot, world=None, teleop=None) -> Dict[str, Any]:
        """
        Execute the picking skill
        
        Returns:
            Dict with execution results
        """
        self.start_time = time.time()
        self.attempts += 1
        self.status = "EXECUTING"
        
        result = {
            "skill": "PickObject",
            "object_id": self.object_id,
            "attempt": self.attempts,
            "start_time": self.start_time,
            "status": "FAILURE",
            "error": None,
            "duration": 0.0,
            "ik_success": False,
            "grasp_success": False
        }
        
        try:
            # Check for teleop override
            if teleop and teleop.is_active():
                result["status"] = "OVERRIDE"
                result["error"] = "Manual override active"
                return result
                
            # Validate inputs
            if not world:
                result["error"] = "World reference required"
                return result
                
            # Get object position
            obj_pos = world.get_object_position(self.object_id)
            if obj_pos is None:
                result["error"] = f"Object {self.object_id} not found"
                return result
                
            # Check if robot is in range
            robot_pos = robot.get_position()
            distance = np.linalg.norm(np.array(obj_pos[:2]) - np.array(robot_pos[:2]))
            
            if distance > robot.reach_distance:
                result["error"] = f"Object out of reach: {distance:.3f} > {robot.reach_distance}"
                return result
                
            # Check if object is already held
            if robot.is_holding(self.object_id):
                result["status"] = "SUCCESS"
                result["error"] = "Object already held"
                result["duration"] = time.time() - self.start_time
                return result
                
            # Plan grasp approach
            grasp_success = self._plan_and_execute_grasp(robot, obj_pos, result)
            
            if grasp_success:
                # Verify grasp
                if robot.is_holding(self.object_id):
                    result["status"] = "SUCCESS"
                    result["grasp_success"] = True
                    self.status = "SUCCESS"
                else:
                    result["error"] = "Grasp verification failed"
                    if self.attempts < self.max_attempts:
                        result["status"] = "RETRY"
                        self.status = "RETRY"
            else:
                if self.attempts < self.max_attempts:
                    result["status"] = "RETRY"
                    self.status = "RETRY"
                else:
                    result["status"] = "FAILURE"
                    self.status = "FAILURE"
                    
        except Exception as e:
            result["error"] = str(e)
            result["status"] = "ERROR"
            self.status = "ERROR"
            
        result["duration"] = time.time() - self.start_time
        return result
        
    def _plan_and_execute_grasp(self, robot, obj_pos: List[float], result: Dict) -> bool:
        """Plan and execute the grasping motion"""
        
        # 1. Navigate robot to object first
        navigation_success = self._navigate_to_object(robot, obj_pos)
        if not navigation_success:
            result["error"] = "Failed to navigate to object"
            return False
        
        # 2. Calculate approach pose
        approach_pose = self._calculate_approach_pose(obj_pos)
        self.grasp_pose = approach_pose
        
        # 3. Solve inverse kinematics
        ik_success = self._solve_ik_for_grasp(robot, approach_pose, result)
        result["ik_success"] = ik_success
        
        if not ik_success:
            result["error"] = result.get("error", "IK solution failed")
            return False
            
        # 4. Execute approach motion
        approach_success = self._execute_approach_motion(robot, approach_pose)
        
        if not approach_success:
            result["error"] = "Approach motion failed"
            return False
            
        # 5. Close gripper and create grasp
        grasp_success = self._execute_grasp(robot, obj_pos)
        
        if not grasp_success:
            result["error"] = "Grasp execution failed"
            return False
            
        return True
        
    def _navigate_to_object(self, robot, obj_pos: List[float]) -> bool:
        """Navigate robot to within grasping range of the object"""
        
        robot_pos = robot.get_position()
        
        # Calculate target position that allows gripper to reach object
        # Position robot so its front gripper (which extends 0.4m forward) can reach the object
        # We want to be far enough away that the extended gripper reaches the object
        approach_distance = 0.6  # Robot needs to be 0.6m away so gripper extension can reach
        
        # Calculate direction from object to robot (to determine approach direction)
        dx = robot_pos[0] - obj_pos[0]
        dy = robot_pos[1] - obj_pos[1]
        distance = np.sqrt(dx*dx + dy*dy)
        
        # If robot is too close, move it away first
        if distance < 0.2:
            # Move robot away from object before approaching properly
            if abs(dx) > abs(dy):
                # Move along X axis
                offset_x = approach_distance if dx > 0 else -approach_distance
                offset_y = 0
            else:
                # Move along Y axis
                offset_x = 0
                offset_y = approach_distance if dy > 0 else -approach_distance
        else:
            # Normalize direction and calculate approach position
            direction_x = dx / distance if distance > 0 else 1.0
            direction_y = dy / distance if distance > 0 else 0.0
            
            offset_x = direction_x * approach_distance
            offset_y = direction_y * approach_distance
        
        target_pos = [
            obj_pos[0] + offset_x,  # Position robot away from object
            obj_pos[1] + offset_y,  # Position robot away from object
            robot_pos[2]            # Keep same height
        ]
        
        print(f"  📍 Navigating robot from {robot_pos} to {target_pos} (to approach object at {obj_pos})")
        print(f"      Approach distance: {approach_distance}m, offset: [{offset_x:.2f}, {offset_y:.2f}]")
        
        # Start walking to target
        robot.start_walking_to(target_pos)
        
        # Wait for robot to reach target
        max_walk_time = 5.0  # Maximum time to wait for navigation
        start_time = time.time()
        
        while robot.is_walking() and (time.time() - start_time) < max_walk_time:
            time.sleep(0.1)
            
        # Check if robot reached target
        final_pos = robot.get_position()
        distance_to_target = np.linalg.norm(np.array(target_pos[:2]) - np.array(final_pos[:2]))
        
        if distance_to_target < 0.15:  # Within 15cm of target
            print(f"  ✅ Robot reached approach position (distance: {distance_to_target:.3f}m)")
            return True
        else:
            print(f"  ❌ Robot failed to reach target (distance: {distance_to_target:.3f}m)")
            return False

    def _calculate_approach_pose(self, obj_pos: List[float]) -> List[float]:
        """Calculate the approach pose for grasping"""
        # Simple top-down approach
        approach_pose = [
            obj_pos[0],
            obj_pos[1], 
            obj_pos[2] + 0.1  # Approach from above
        ]
        return approach_pose
        
    def _solve_ik_for_grasp(self, robot, target_pose: List[float], result: Dict) -> bool:
        """Solve inverse kinematics for the grasp pose"""
        
        # In test mode, always succeed with IK
        if self.test_mode:
            return True
        
        # Check if robot has IK capability
        if robot.robot_id is None or len(robot.joints) == 0:
            # Fallback for simple robots
            return self._simple_reachability_check(robot, target_pose)
            
        try:
            # Import pybullet for IK
            import pybullet as p
            
            # Calculate IK solution
            joint_positions = p.calculateInverseKinematics(
                robot.robot_id,
                robot.end_effector_link,
                target_pose,
                maxNumIterations=100,
                residualThreshold=0.01
            )
            
            # Check if solution is valid
            if len(joint_positions) == 0:
                result["error"] = "No IK solution found"
                return False
                
            # Check joint limits (simplified)
            for i, pos in enumerate(joint_positions[:len(robot.joints)]):
                if abs(pos) > np.pi:  # Simple joint limit check
                    result["error"] = f"Joint {i} out of limits: {pos}"
                    return False
                    
            return True
            
        except Exception as e:
            result["error"] = f"IK calculation error: {str(e)}"
            return False
            
    def _simple_reachability_check(self, robot, target_pose: List[float]) -> bool:
        """Simple reachability check for robots without IK"""
        robot_pos = robot.get_position()
        distance = np.linalg.norm(np.array(target_pose) - np.array(robot_pos))
        
        # Check if within reach
        return distance <= robot.reach_distance
        
    def _execute_approach_motion(self, robot, target_pose: List[float]) -> bool:
        """Execute the approach motion to the grasp pose"""
        
        # Simulate approach motion time
        approach_time = random.uniform(0.5, 1.5)  # Variable approach time
        time.sleep(approach_time)
        
        # Add some failure probability based on object position complexity
        if self.test_mode:
            # In test mode, always succeed unless position is invalid
            success_rate = 1.0
        else:
            success_rate = 0.9  # 90% success rate for approach
            
            # Reduce success rate if object is in difficult position
            if target_pose[2] < 0.05:  # Very low objects are harder
                success_rate *= 0.7
            if target_pose[0] < 0 or target_pose[1] < 0:  # Negative positions are harder
                success_rate *= 0.8

        return random.random() < success_rate
        
    def _execute_grasp(self, robot, obj_pos: List[float]) -> bool:
        """Execute the actual grasping action"""
        
        # Simulate gripper closing time
        grasp_time = random.uniform(0.2, 0.5)
        time.sleep(grasp_time)
        
        # Attempt grasp with robot
        return robot.attempt_pick(self.object_id, obj_pos)
        
    def _calculate_grasp_quality(self, obj_pos: List[float], robot_pos: List[float], reach_distance: float = 1.0) -> float:
        """Calculate a quality metric for the grasp (0.0 to 1.0)"""
        
        # Distance factor (closer is better)
        distance = np.linalg.norm(np.array(obj_pos[:2]) - np.array(robot_pos[:2]))
        distance_factor = max(0, 1.0 - distance / reach_distance)
        
        # Height factor (waist height is optimal)
        optimal_height = 0.8  # Assume waist height
        height_diff = abs(obj_pos[2] - optimal_height)
        height_factor = max(0, 1.0 - height_diff / 0.5)
        
        # Angle factor (front approach is best)
        # Simplified - assume front approach is always good
        angle_factor = 1.0
        
        # Combine factors
        quality = (distance_factor * 0.4 + height_factor * 0.3 + angle_factor * 0.3)
        return max(0.0, min(1.0, quality))
        
    def can_retry(self) -> bool:
        """Check if skill can be retried"""
        return self.attempts < self.max_attempts and self.status in ["FAILURE", "RETRY"]
        
    def reset(self) -> None:
        """Reset skill state for retry"""
        self.attempts = 0
        self.start_time = None
        self.status = "IDLE"
        self.grasp_pose = None
        
    def get_estimated_duration(self) -> float:
        """Get estimated duration for this skill"""
        # Base time + IK time + approach time + grasp time
        return 0.5 + 0.3 + 1.0 + 0.3
        
    def get_success_probability(self, robot, world) -> float:
        """Estimate success probability based on current conditions"""
        
        # Get object position
        obj_pos = world.get_object_position(self.object_id)
        if obj_pos is None:
            return 0.0
            
        # Check reachability
        robot_pos = robot.get_position()
        distance = np.linalg.norm(np.array(obj_pos[:2]) - np.array(robot_pos[:2]))
        
        if distance > robot.reach_distance:
            return 0.0
            
        # Calculate base success rate
        base_success = 0.8
        
        # Modify based on grasp quality
        quality = self._calculate_grasp_quality(obj_pos, robot_pos, robot.reach_distance)
        success_prob = base_success * quality
        
        # Account for previous attempts
        retry_penalty = 0.9 ** self.attempts
        
        return success_prob * retry_penalty
