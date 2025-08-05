"""
Custom workflow skill that combines basic behaviors into complex robot workflows.
This allows users to define custom sequences of actions for specific scenarios.
"""

import time
import json
from typing import List, Dict, Any, Optional

try:
    import numpy as np
except ImportError:
    print("Warning: numpy not available, using mock implementation")
    np = None

try:
    from behavior_tree.sequence import Sequence
    from behavior_tree.fallback import Fallback
    from behavior_tree.action_nodes import WalkToTarget, PickObject, PlaceObject, WaitForHuman
except ImportError:
    # Fallback imports for when modules don't exist
    print("Warning: behavior_tree modules not available, using mock implementations")
    
    class MockNode:
        def __init__(self, *args, **kwargs):
            pass
        def tick(self):
            return "SUCCESS"
    
    Sequence = Fallback = WalkToTarget = PickObject = PlaceObject = WaitForHuman = MockNode


class CustomWorkflowSkill:
    """Skill that composes multiple atomic skills into complex workflows"""
    
    def __init__(self, workflow_name: str, workflow_config: Dict = None):
        self.workflow_name = workflow_name
        self.workflow_config = workflow_config or {}
        self.behavior_tree = None
        self.execution_log: List[Dict] = []
        self.start_time = None
        self.status = "IDLE"
        self.current_step = 0
        self.total_steps = 0
        
    def load_workflow_from_config(self, config_path: str) -> bool:
        """Load workflow configuration from file"""
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
                
            self.workflow_config = config
            self.workflow_name = config.get('name', self.workflow_name)
            return self._build_behavior_tree_from_config(config)
            
        except Exception as e:
            print(f"Failed to load workflow config: {e}")
            return False
            
    def create_fetch_and_place_workflow(self, object_id: str, object_location: List[float],
                                      target_location: List[float]) -> None:
        """Create a standard fetch and place workflow"""
        
        # Create behavior tree for fetch and place
        self.behavior_tree = Sequence([
            WalkToTarget(target=object_location, tolerance=0.5),
            PickObject(object_id=object_id),
            WalkToTarget(target=target_location, tolerance=0.5), 
            PlaceObject(location=target_location)
        ])
        
        self.total_steps = 4
        self.workflow_config = {
            "name": "fetch_and_place",
            "object_id": object_id,
            "object_location": object_location,
            "target_location": target_location
        }
        
    def create_multi_object_workflow(self, tasks: List[Dict]) -> None:
        """Create workflow for handling multiple objects"""
        
        # Build sequence of fetch-and-place operations
        operations = []
        
        for task in tasks:
            obj_id = task['object_id']
            obj_loc = task['object_location']
            target_loc = task['target_location']
            
            # Add fetch and place sequence for each object
            fetch_place = Sequence([
                WalkToTarget(target=obj_loc, tolerance=0.5),
                PickObject(object_id=obj_id),
                WalkToTarget(target=target_loc, tolerance=0.5),
                PlaceObject(location=target_loc)
            ])
            operations.append(fetch_place)
            
        self.behavior_tree = Sequence(operations)
        self.total_steps = len(tasks) * 4
        self.workflow_config = {
            "name": "multi_object_workflow",
            "tasks": tasks
        }
        
    def create_patrol_and_inspect_workflow(self, waypoints: List[List[float]], 
                                         inspection_duration: float = 2.0) -> None:
        """Create a patrol and inspection workflow"""
        
        operations = []
        
        for waypoint in waypoints:
            # Walk to waypoint and wait (simulate inspection)
            patrol_step = Sequence([
                WalkToTarget(target=waypoint, tolerance=0.3),
                WaitForHuman(duration=inspection_duration)  # Inspection pause
            ])
            operations.append(patrol_step)
            
        self.behavior_tree = Sequence(operations)
        self.total_steps = len(waypoints) * 2
        self.workflow_config = {
            "name": "patrol_and_inspect",
            "waypoints": waypoints,
            "inspection_duration": inspection_duration
        }
        
    def create_delivery_workflow(self, pickup_location: List[float], 
                                delivery_locations: List[List[float]],
                                package_ids: List[str] = None) -> None:
        """Create a delivery workflow with multiple stops"""
        
        operations = []
        
        # Go to pickup location first
        operations.append(WalkToTarget(target=pickup_location, tolerance=0.3))
        
        # Pick up packages
        if package_ids:
            for pkg_id in package_ids:
                operations.append(PickObject(object_id=pkg_id))
        
        # Deliver to each location
        for i, delivery_loc in enumerate(delivery_locations):
            operations.append(WalkToTarget(target=delivery_loc, tolerance=0.3))
            
            # If we have specific package IDs, place the corresponding package
            if package_ids and i < len(package_ids):
                operations.append(PlaceObject(location=delivery_loc, object_id=package_ids[i]))
            else:
                operations.append(PlaceObject(location=delivery_loc))
                
        self.behavior_tree = Sequence(operations)
        self.total_steps = 1 + len(package_ids or []) + len(delivery_locations) * 2
        self.workflow_config = {
            "name": "delivery_workflow",
            "pickup_location": pickup_location,
            "delivery_locations": delivery_locations,
            "package_ids": package_ids
        }
        
    def create_robust_workflow_with_fallbacks(self, primary_plan: List[Dict],
                                            fallback_plan: List[Dict]) -> None:
        """Create workflow with fallback behaviors"""
        
        # Build primary sequence
        primary_ops = self._build_operations_from_plan(primary_plan)
        primary_sequence = Sequence(primary_ops)
        
        # Build fallback sequence  
        fallback_ops = self._build_operations_from_plan(fallback_plan)
        fallback_sequence = Sequence(fallback_ops)
        
        # Use fallback node to try primary first, then fallback
        self.behavior_tree = Fallback([primary_sequence, fallback_sequence])
        
        self.total_steps = len(primary_plan) + len(fallback_plan)
        self.workflow_config = {
            "name": "robust_workflow",
            "primary_plan": primary_plan,
            "fallback_plan": fallback_plan
        }
        
    def _build_operations_from_plan(self, plan: List[Dict]) -> List:
        """Build behavior tree operations from plan specification"""
        operations = []
        
        for step in plan:
            action = step.get('action')
            params = step.get('parameters', {})
            
            if action == 'walk':
                operations.append(WalkToTarget(target=params['target'], 
                                             tolerance=params.get('tolerance', 0.3)))
            elif action == 'pick':
                operations.append(PickObject(object_id=params['object_id']))
            elif action == 'place':
                operations.append(PlaceObject(location=params['location'],
                                            object_id=params.get('object_id')))
            elif action == 'wait':
                operations.append(WaitForHuman(duration=params.get('duration', 1.0)))
                
        return operations
        
    def _build_behavior_tree_from_config(self, config: Dict) -> bool:
        """Build behavior tree from configuration dictionary"""
        try:
            root_type = config.get('root', 'Sequence')
            nodes_config = config.get('nodes', [])
            
            if root_type == 'Sequence':
                self.behavior_tree = Sequence(self._parse_nodes(nodes_config))
            elif root_type == 'Fallback':
                self.behavior_tree = Fallback(self._parse_nodes(nodes_config))
            else:
                print(f"Unknown root type: {root_type}")
                return False
                
            self.total_steps = len(nodes_config)
            return True
            
        except Exception as e:
            print(f"Failed to build behavior tree from config: {e}")
            return False
            
    def _parse_nodes(self, nodes_config: List[Dict]) -> List:
        """Parse node configurations into behavior tree nodes"""
        nodes = []
        
        for node_config in nodes_config:
            node_type = node_config.get('type')
            params = node_config.get('parameters', {})
            
            if node_type == 'WalkToTarget':
                nodes.append(WalkToTarget(target=params['target'],
                                        tolerance=params.get('tolerance', 0.3)))
            elif node_type == 'PickObject':
                nodes.append(PickObject(object_id=params['object_id']))
            elif node_type == 'PlaceObject':
                nodes.append(PlaceObject(location=params['location'],
                                       object_id=params.get('object_id')))
            elif node_type == 'WaitForHuman':
                nodes.append(WaitForHuman(duration=params.get('duration', 1.0)))
                
        return nodes
        
    def execute(self, robot, world=None, teleop=None) -> Dict[str, Any]:
        """Execute the complete workflow"""
        
        if not self.behavior_tree:
            return {
                "workflow": self.workflow_name,
                "status": "ERROR",
                "error": "No behavior tree defined",
                "duration": 0.0
            }
            
        self.start_time = time.time()
        self.status = "EXECUTING"
        self.execution_log.clear()
        
        # Create context for behavior tree
        context = type('Context', (), {
            'robot': robot,
            'world': world,
            'teleop': teleop,
            'workflow': self
        })()
        
        # Execute behavior tree
        result = self.behavior_tree.tick(context)
        
        execution_time = time.time() - self.start_time
        
        # Compile final result
        final_result = {
            "workflow": self.workflow_name,
            "status": result.value if hasattr(result, 'value') else str(result),
            "duration": execution_time,
            "steps_completed": self.current_step,
            "total_steps": self.total_steps,
            "execution_log": self.execution_log.copy(),
            "config": self.workflow_config
        }
        
        if result.value == "SUCCESS":
            self.status = "SUCCESS"
        else:
            self.status = "FAILURE"
            final_result["error"] = "Workflow execution failed"
            
        return final_result
        
    def log_step_execution(self, step_name: str, step_result: Dict) -> None:
        """Log the execution of a workflow step"""
        log_entry = {
            "step": self.current_step,
            "step_name": step_name,
            "timestamp": time.time(),
            "result": step_result
        }
        self.execution_log.append(log_entry)
        self.current_step += 1
        
    def get_progress(self) -> float:
        """Get current progress (0.0 to 1.0)"""
        if self.total_steps == 0:
            return 0.0
        return min(self.current_step / self.total_steps, 1.0)
        
    def get_estimated_remaining_time(self) -> float:
        """Estimate remaining execution time"""
        if self.start_time is None or self.current_step == 0:
            return 0.0
            
        elapsed_time = time.time() - self.start_time
        avg_step_time = elapsed_time / self.current_step
        remaining_steps = self.total_steps - self.current_step
        
        return avg_step_time * remaining_steps
        
    def pause_execution(self) -> None:
        """Pause workflow execution"""
        self.status = "PAUSED"
        
    def resume_execution(self) -> None:
        """Resume workflow execution"""
        if self.status == "PAUSED":
            self.status = "EXECUTING"
            
    def abort_execution(self) -> None:
        """Abort workflow execution"""
        self.status = "ABORTED"
        
    def reset(self) -> None:
        """Reset workflow state"""
        self.current_step = 0
        self.start_time = None
        self.status = "IDLE"
        self.execution_log.clear()
        if self.behavior_tree:
            self.behavior_tree.reset()
