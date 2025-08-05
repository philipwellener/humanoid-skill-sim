"""
WalkToTarget skill implementation
"""

import time
import numpy as np
from typing import List, Optional, Dict, Any


class WalkToTargetSkill:
    """Skill for walking to a target location with path planning"""
    
    def __init__(self, target: List[float], tolerance: float = 0.1, 
                 max_attempts: int = 3, timeout: float = 30.0):
        self.target = target
        self.tolerance = tolerance
        self.max_attempts = max_attempts
        self.timeout = timeout
        self.attempts = 0
        self.start_time = None
        self.status = "IDLE"
        self.path_points: List[List[float]] = []
        
    def execute(self, robot, world=None, teleop=None) -> Dict[str, Any]:
        """
        Execute the walking skill
        
        Returns:
            Dict with execution results including status, duration, errors
        """
        self.start_time = time.time()
        self.attempts += 1
        self.status = "EXECUTING"
        
        result = {
            "skill": "WalkToTarget",
            "target": self.target,
            "attempt": self.attempts,
            "start_time": self.start_time,
            "status": "FAILURE",
            "error": None,
            "duration": 0.0,
            "path_length": 0.0
        }
        
        try:
            # Check for teleop override
            if teleop and teleop.is_active():
                result["status"] = "OVERRIDE"
                result["error"] = "Manual override active"
                return result
                
            # Get current robot position
            current_pos = robot.get_position()
            
            # Calculate initial distance
            initial_distance = np.linalg.norm(
                np.array(self.target[:2]) - np.array(current_pos[:2])
            )
            
            # Check if already at target
            if initial_distance <= self.tolerance:
                result["status"] = "SUCCESS"
                result["duration"] = time.time() - self.start_time
                self.status = "SUCCESS"
                return result
                
            # Plan path (simplified - direct path)
            self.path_points = self._plan_path(current_pos, self.target, world)
            result["path_length"] = self._calculate_path_length()
            
            # Start walking
            robot.start_walking_to(self.target)
            
            # Monitor walking progress
            while robot.is_walking():
                # Check timeout
                if time.time() - self.start_time > self.timeout:
                    result["error"] = "TIMEOUT"
                    result["duration"] = time.time() - self.start_time
                    self.status = "TIMEOUT"
                    return result
                    
                # Check for teleop override
                if teleop and teleop.is_active():
                    result["status"] = "OVERRIDE"
                    result["error"] = "Manual override during execution"
                    result["duration"] = time.time() - self.start_time
                    return result
                    
                # Small delay to prevent tight loop
                time.sleep(0.1)
                
            # Check final position
            final_pos = robot.get_position()
            final_distance = np.linalg.norm(
                np.array(self.target[:2]) - np.array(final_pos[:2])
            )
            
            if final_distance <= self.tolerance:
                result["status"] = "SUCCESS"
                self.status = "SUCCESS"
            else:
                result["error"] = f"Final distance {final_distance:.3f} > tolerance {self.tolerance}"
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
        
    def _plan_path(self, start: List[float], goal: List[float], world=None) -> List[List[float]]:
        """
        Plan path from start to goal (simplified)
        In a real implementation, this would use A* or RRT
        """
        # Simple direct path for now
        path = [start[:2], goal[:2]]
        
        # Add obstacle avoidance if world is provided
        if world:
            path = self._avoid_obstacles(path, world)
            
        return path
        
    def _avoid_obstacles(self, path: List[List[float]], world) -> List[List[float]]:
        """Basic obstacle avoidance (simplified)"""
        # In a real implementation, this would check for collisions
        # and add waypoints to avoid obstacles
        
        # For now, just add a waypoint if there are obstacles
        obstacles = [obj for obj in world.get_all_objects() if "obstacle" in obj.lower()]
        
        if obstacles:
            # Add a simple detour waypoint
            midpoint = [
                (path[0][0] + path[1][0]) / 2 + 0.5,  # Offset to avoid obstacles
                (path[0][1] + path[1][1]) / 2
            ]
            path.insert(1, midpoint)
            
        return path
        
    def _calculate_path_length(self) -> float:
        """Calculate total path length"""
        if len(self.path_points) < 2:
            return 0.0
            
        total_length = 0.0
        for i in range(1, len(self.path_points)):
            segment_length = np.linalg.norm(
                np.array(self.path_points[i]) - np.array(self.path_points[i-1])
            )
            total_length += segment_length
            
        return total_length
        
    def can_retry(self) -> bool:
        """Check if skill can be retried"""
        return self.attempts < self.max_attempts and self.status in ["FAILURE", "RETRY"]
        
    def reset(self) -> None:
        """Reset skill state for retry"""
        self.attempts = 0
        self.start_time = None
        self.status = "IDLE"
        self.path_points.clear()
        
    def get_progress(self, robot) -> float:
        """Get current progress (0.0 to 1.0)"""
        if not self.path_points or self.start_time is None:
            return 0.0
            
        current_pos = robot.get_position()
        
        # Find closest point on path
        min_distance = float('inf')
        closest_segment = 0
        
        for i in range(len(self.path_points) - 1):
            # Distance to line segment
            dist = self._point_to_line_distance(
                current_pos[:2], 
                self.path_points[i], 
                self.path_points[i+1]
            )
            if dist < min_distance:
                min_distance = dist
                closest_segment = i
                
        # Calculate progress based on closest segment
        total_length = self._calculate_path_length()
        if total_length == 0:
            return 1.0
            
        # Length covered up to closest segment
        covered_length = 0.0
        for i in range(closest_segment):
            covered_length += np.linalg.norm(
                np.array(self.path_points[i+1]) - np.array(self.path_points[i])
            )
            
        return min(covered_length / total_length, 1.0)
        
    def _point_to_line_distance(self, point: List[float], line_start: List[float], 
                              line_end: List[float]) -> float:
        """Calculate distance from point to line segment"""
        # Vector from line_start to line_end
        line_vec = np.array(line_end) - np.array(line_start)
        # Vector from line_start to point
        point_vec = np.array(point) - np.array(line_start)
        
        # Project point onto line
        line_len = np.linalg.norm(line_vec)
        if line_len == 0:
            return np.linalg.norm(point_vec)
            
        line_unitvec = line_vec / line_len
        proj_length = np.dot(point_vec, line_unitvec)
        
        # Clamp to line segment
        proj_length = max(0, min(proj_length, line_len))
        
        # Find closest point on line segment
        closest_point = np.array(line_start) + proj_length * line_unitvec
        
        # Return distance
        return np.linalg.norm(np.array(point) - closest_point)
