"""
Teleop override system for manual intervention and data collection
"""

import time
from typing import Dict, List, Optional, Any, Callable
import json


class TeleopOverride:
    """System for manual override and teleop data collection during simulation"""
    
    def __init__(self):
        self.active_override = False
        self.override_start_time = None
        self.override_log: List[Dict] = []
        self.manual_commands: List[Dict] = []
        self.callbacks: Dict[str, Callable] = {}
        
    def start_override(self, reason: str = "manual_intervention") -> None:
        """Start a teleop override session"""
        self.active_override = True
        self.override_start_time = time.time()
        
        override_entry = {
            "type": "override_start",
            "timestamp": self.override_start_time,
            "reason": reason
        }
        self.override_log.append(override_entry)
        print(f"Teleop override started: {reason}")
        
    def end_override(self) -> float:
        """End the teleop override session and return duration"""
        if not self.active_override:
            return 0.0
            
        duration = time.time() - self.override_start_time
        self.active_override = False
        
        override_entry = {
            "type": "override_end", 
            "timestamp": time.time(),
            "duration": duration
        }
        self.override_log.append(override_entry)
        print(f"Teleop override ended. Duration: {duration:.2f}s")
        
        return duration
        
    def is_active(self) -> bool:
        """Check if override is currently active"""
        return self.active_override
    
    def activate(self) -> None:
        """Activate teleop override (alias for start_override)"""
        self.start_override("activation")
    
    def deactivate(self) -> None:
        """Deactivate teleop override (alias for end_override)"""
        self.end_override()
    
    def set_movement_command(self, command: List[float]) -> bool:
        """Set movement command for teleop control"""
        if not self.active_override:
            return False
        
        command_entry = {
            "type": "movement_command",
            "timestamp": time.time(),
            "command": command
        }
        self.manual_commands.append(command_entry)
        
        return True
    
    def set_manipulation_command(self, action: str, object_id: str) -> bool:
        """Set manipulation command for teleop control"""
        if not self.active_override:
            return False
        
        command_entry = {
            "type": "manipulation_command",
            "timestamp": time.time(),
            "action": action,
            "object_id": object_id
        }
        self.manual_commands.append(command_entry)
        
        return True
    
    def end_intervention(self, intervention_id: str, success: bool = True) -> bool:
        """End a manual intervention"""
        intervention_entry = {
            "type": "intervention_end",
            "intervention_id": intervention_id,
            "success": success,
            "timestamp": time.time()
        }
        self.manual_commands.append(intervention_entry)
        
        return True
    
    def reset(self) -> bool:
        """Reset teleop system"""
        self.active_override = False
        self.override_start_time = None
        self.override_log.clear()
        self.manual_commands.clear()
        return True
    
    def check_override_needed(self, context: Dict) -> bool:
        """Check if override is needed based on context"""
        # Simple heuristic - override needed if there are failures
        return context.get("failure_detected", False)
    
    def start_intervention(self, intervention_type: str) -> str:
        """Start a manual intervention and return intervention ID"""
        intervention_id = f"intervention_{len(self.manual_commands):03d}"
        
        intervention_entry = {
            "type": "intervention_start",
            "intervention_id": intervention_id,
            "intervention_type": intervention_type,
            "timestamp": time.time()
        }
        self.manual_commands.append(intervention_entry)
        
        return intervention_id
    
    def emergency_stop(self) -> bool:
        """Trigger emergency stop"""
        stop_entry = {
            "type": "emergency_stop",
            "timestamp": time.time()
        }
        self.manual_commands.append(stop_entry)
        
        return True
    
    def get_state(self) -> Dict:
        """Get current teleop state"""
        return {
            "active": self.active_override,
            "start_time": self.override_start_time,
            "num_commands": len(self.manual_commands),
            "num_overrides": len(self.override_log)
        }
    
    def update_feedback(self, feedback: Dict) -> bool:
        """Update feedback from teleop interface"""
        feedback_entry = {
            "type": "feedback_update",
            "timestamp": time.time(),
            "feedback": feedback
        }
        self.manual_commands.append(feedback_entry)
        
        return True
    
    def get_feedback(self) -> Dict:
        """Get current feedback state"""
        # Get the latest feedback entry
        for command in reversed(self.manual_commands):
            if command.get("type") == "feedback_update":
                return command.get("feedback", {})
        return {}
    
    def validate_command(self, command: List[float]) -> bool:
        """Validate if a command is within safety bounds"""
        if not isinstance(command, list) or len(command) < 2:
            return False
        
        # Check for reasonable movement bounds (max 10 units in any direction)
        for val in command:
            if abs(val) > 10.0:
                return False
        
        return True
        
    def manual_move_robot(self, robot, target_position: List[float]) -> None:
        """Manually move robot to target position during override"""
        if not self.active_override:
            print("Cannot move robot: teleop override not active")
            return
            
        command_entry = {
            "type": "manual_robot_move",
            "timestamp": time.time(),
            "target_position": target_position,
            "robot_id": getattr(robot, 'robot_id', 'unknown')
        }
        self.manual_commands.append(command_entry)
        
        # Execute the movement
        robot.set_position(target_position)
        print(f"Robot manually moved to {target_position}")
        
    def manual_move_object(self, world, object_id: str, target_position: List[float]) -> None:
        """Manually move an object during override"""
        if not self.active_override:
            print("Cannot move object: teleop override not active")
            return
            
        command_entry = {
            "type": "manual_object_move",
            "timestamp": time.time(),
            "object_id": object_id,
            "target_position": target_position
        }
        self.manual_commands.append(command_entry)
        
        # Execute the movement through world interface
        if hasattr(world, 'move_object'):
            world.move_object(object_id, target_position)
            print(f"Object {object_id} manually moved to {target_position}")
        else:
            print(f"World does not support object movement")
            
    def force_skill_success(self, skill_name: str) -> None:
        """Force a skill to succeed during override"""
        command_entry = {
            "type": "force_skill_success",
            "timestamp": time.time(),
            "skill_name": skill_name
        }
        self.manual_commands.append(command_entry)
        print(f"Skill {skill_name} forced to succeed")
        
    def force_skill_failure(self, skill_name: str, reason: str = "manual_failure") -> None:
        """Force a skill to fail during override"""
        command_entry = {
            "type": "force_skill_failure",
            "timestamp": time.time(),
            "skill_name": skill_name,
            "reason": reason
        }
        self.manual_commands.append(command_entry)
        print(f"Skill {skill_name} forced to fail: {reason}")
        
    def inject_grasp_success(self, robot, object_id: str) -> None:
        """Inject a successful grasp during override"""
        command_entry = {
            "type": "inject_grasp_success", 
            "timestamp": time.time(),
            "object_id": object_id,
            "robot_id": getattr(robot, 'robot_id', 'unknown')
        }
        self.manual_commands.append(command_entry)
        
        # Simulate the grasp
        if hasattr(robot, 'attempt_pick'):
            robot.attempt_pick(object_id, [0, 0, 0])  # Position doesn't matter for injection
            print(f"Grasp success injected for object {object_id}")
            
    def inject_grasp_failure(self, robot, object_id: str) -> None:
        """Inject a grasp failure during override"""
        command_entry = {
            "type": "inject_grasp_failure",
            "timestamp": time.time(), 
            "object_id": object_id,
            "robot_id": getattr(robot, 'robot_id', 'unknown')
        }
        self.manual_commands.append(command_entry)
        print(f"Grasp failure injected for object {object_id}")
        
    def collect_correction_data(self, skill_name: str, failure_type: str, 
                              correction_action: str) -> None:
        """Collect data about manual corrections for training"""
        correction_entry = {
            "type": "correction_data",
            "timestamp": time.time(),
            "skill_name": skill_name,
            "failure_type": failure_type,
            "correction_action": correction_action
        }
        self.manual_commands.append(correction_entry)
        print(f"Correction data collected for {skill_name}")
        
    def simulate_human_intervention(self, intervention_type: str, duration: float = 2.0) -> None:
        """Simulate a human intervention for testing"""
        intervention_entry = {
            "type": "simulated_intervention",
            "timestamp": time.time(),
            "intervention_type": intervention_type,
            "duration": duration
        }
        self.manual_commands.append(intervention_entry)
        
        print(f"Simulating {intervention_type} intervention for {duration}s")
        time.sleep(duration)
        print(f"Intervention {intervention_type} completed")
        
    def register_callback(self, event_type: str, callback: Callable) -> None:
        """Register callback for teleop events"""
        self.callbacks[event_type] = callback
        
    def trigger_callback(self, event_type: str, *args, **kwargs) -> None:
        """Trigger registered callback"""
        if event_type in self.callbacks:
            self.callbacks[event_type](*args, **kwargs)
            
    def get_override_summary(self) -> Dict:
        """Get summary of override session"""
        manual_interventions = len([cmd for cmd in self.manual_commands 
                                  if cmd.get("type", "").startswith("manual_")])
        forced_outcomes = len([cmd for cmd in self.manual_commands 
                             if cmd.get("type", "").startswith("force_skill_")])
        
        total_duration = 0.0
        for entry in self.override_log:
            if entry.get("type") == "override_end":
                total_duration += entry.get("duration", 0.0)
                
        return {
            "total_overrides": len(self.override_log) // 2,  # start/end pairs
            "total_duration": total_duration,
            "manual_interventions": manual_interventions,
            "forced_outcomes": forced_outcomes,
            "total_commands": len(self.manual_commands),
            "currently_active": self.active_override
        }
        
    def export_teleop_data(self, filename: str) -> None:
        """Export all teleop data to a JSON file for analysis"""
        data = {
            "override_log": self.override_log,
            "manual_commands": self.manual_commands,
            "summary": self.get_override_summary(),
            "export_timestamp": time.time()
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
            
        print(f"Teleop data exported to {filename}")
        
    def clear_logs(self) -> None:
        """Clear all logged data"""
        self.override_log.clear()
        self.manual_commands.clear()
        print("Teleop logs cleared")
        
    def suggest_dataset_corrections(self) -> List[Dict]:
        """Analyze override data and suggest dataset corrections for retraining"""
        suggestions = []
        
        # Analyze failure patterns
        failure_types = {}
        for cmd in self.manual_commands:
            if cmd["type"] == "force_skill_failure":
                skill = cmd["skill_name"]
                reason = cmd.get("reason", "unknown")
                key = f"{skill}_{reason}"
                failure_types[key] = failure_types.get(key, 0) + 1
                
        # Suggest corrections based on patterns
        for failure_pattern, count in failure_types.items():
            if count > 1:  # Multiple occurrences suggest systematic issue
                skill, reason = failure_pattern.rsplit('_', 1)
                suggestion = {
                    "type": "dataset_correction",
                    "skill": skill,
                    "issue": reason,
                    "frequency": count,
                    "suggested_action": f"Collect more training data for {skill} with {reason} scenarios"
                }
                suggestions.append(suggestion)
                
        return suggestions
