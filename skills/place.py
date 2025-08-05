"""
PlaceObject skill implementation
"""

import time
import numpy as np
import random
from typing import List, Dict, Any, Optional

try:
    import yaml
except ImportError:
    yaml = None


class PlaceObjectSkill:
    """Skill for placing objects at target locations"""
    
    def __init__(self, location: List[float], object_id: Optional[str] = None,
                 placement_tolerance: float = 0.05, max_attempts: int = 3, 
                 timeout: float = 15.0, test_mode: bool = False):
        self.location = location
        self.object_id = object_id  # None means place any held object
        self.placement_tolerance = placement_tolerance
        self.max_attempts = max_attempts
        self.timeout = timeout
        self.test_mode = test_mode  # Disable random failures in test mode
        self.attempts = 0
        self.start_time = None
        self.status = "IDLE"
        self.place_pose = None
        
    def execute(self, robot, world=None, teleop=None) -> Dict[str, Any]:
        """
        Execute the placing skill
        
        Returns:
            Dict with execution results
        """
        self.start_time = time.time()
        self.attempts += 1
        self.status = "EXECUTING"
        
        result = {
            "skill": "PlaceObject",
            "location": self.location,
            "object_id": self.object_id,
            "attempt": self.attempts,
            "start_time": self.start_time,
            "status": "FAILURE",
            "error": None,
            "duration": 0.0,
            "placement_success": False
        }
        
        try:
            # Check for teleop override
            if teleop and teleop.is_active():
                result["status"] = "OVERRIDE"
                result["error"] = "Manual override active"
                return result
                
            # Determine which object to place
            if self.object_id:
                # Place specific object
                if not robot.is_holding(self.object_id):
                    result["error"] = f"Robot is not holding {self.object_id}"
                    return result
                target_object = self.object_id
            else:
                # Place any held object
                if not robot.is_holding_any():
                    result["error"] = "Robot is not holding any objects"
                    return result
                target_object = robot.get_held_objects()[0]
                
            result["placed_object_id"] = target_object
            
            # Check if robot is in range of placement location
            robot_pos = robot.get_position()
            distance = np.linalg.norm(np.array(self.location[:2]) - np.array(robot_pos[:2]))
            
            if distance > robot.reach_distance:
                result["error"] = f"Location out of reach: {distance:.3f} > {robot.reach_distance}"
                return result
                
            # Plan and execute placement
            placement_success = self._plan_and_execute_placement(robot, target_object, result)
            
            if placement_success:
                # Verify placement
                if not robot.is_holding(target_object):
                    result["status"] = "SUCCESS"
                    result["placement_success"] = True
                    self.status = "SUCCESS"
                else:
                    result["error"] = "Placement verification failed"
                    if self.attempts < self.max_attempts:
                        result["status"] = "RETRY"
                        self.status = "RETRY"
            else:
                if self.attempts < self.max_attempts:
                    result["status"] = "RETRY"
                    self.status = "RETRY"
                else:
                    result["status"] = "FAILURE"
                    
        except Exception as e:
            result["error"] = f"Execution error: {str(e)}"
            result["status"] = "ERROR"
            self.status = "ERROR"
            
        finally:
            result["duration"] = time.time() - self.start_time
            
        return result
    
    def _plan_and_execute_placement(self, robot, object_id: str, result: Dict) -> bool:
        """Plan and execute the object placement"""
        
        # 1. Calculate placement pose
        placement_pose = self._calculate_placement_pose()
        
        # 2. Check for potential collisions
        collision_risk = self._check_placement_collisions(placement_pose)
        if collision_risk:
            result["error"] = "Collision risk detected at placement location"
            return False
            
        # 3. Execute placement motion
        motion_success = self._execute_placement_motion(robot, placement_pose)
        if not motion_success:
            result["error"] = "Placement motion failed"
            return False
            
        # 4. Release object
        release_success = self._execute_release(robot, object_id)
        if not release_success:
            result["error"] = "Object release failed"
            return False
            
        return True
        
    def _calculate_placement_pose(self) -> List[float]:
        """Calculate the pose for placing the object"""
        # Simple placement - slightly above target location
        placement_pose = [
            self.location[0],
            self.location[1],
            self.location[2] + 0.05  # Slightly above target
        ]
        return placement_pose
        
    def _check_placement_collisions(self, target_pose: List[float]) -> bool:
        """Check for potential collisions during placement"""
        # Simplified collision checking
        # In a real system, this would check against known obstacles
        
        # Check if placement is too low (below ground)
        if target_pose[2] < 0.01:
            return True
            
        # Add random collision probability for realism (disabled in test mode)
        if not self.test_mode:
            collision_probability = 0.1  # 10% chance of collision detection
            return random.random() < collision_probability
        
        return False
        
    def _execute_placement_motion(self, robot, target_pose: List[float]) -> bool:
        """Execute the motion to place the object"""
        
        # Simulate placement motion time
        motion_time = random.uniform(0.5, 1.2)
        time.sleep(motion_time)
        
        # Calculate success probability based on placement complexity
        if self.test_mode:
            # In test mode, always succeed
            success_rate = 1.0
        else:
            success_rate = 0.9  # Base success rate
            
            # Reduce success for difficult placements
            if target_pose[2] < 0.1:  # Low placements are harder
                success_rate *= 0.8
            if self.location[0] < 0 or self.location[1] < 0:  # Negative coordinates are harder
                success_rate *= 0.9

        return random.random() < success_rate
        
    def _execute_release(self, robot, object_id: str) -> bool:
        """Execute the object release"""
        
        # Simulate gripper opening time
        release_time = random.uniform(0.1, 0.3)
        time.sleep(release_time)
        
        # Attempt to place with robot
        return robot.attempt_place(self.location, object_id)
        
    def can_retry(self) -> bool:
        """Check if skill can be retried"""
        return self.attempts < self.max_attempts and self.status in ["FAILURE", "RETRY"]
        
    def reset(self) -> None:
        """Reset skill state for retry"""
        self.attempts = 0
        self.start_time = None
        self.status = "IDLE"
        self.place_pose = None
        
    def get_estimated_duration(self) -> float:
        """Get estimated duration for this skill"""
        # Base time + approach time + placement time + release time
        return 0.3 + 0.8 + 0.3
