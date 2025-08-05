#!/usr/bin/env python3
"""
Main simulation runner for humanoid skill testing.
Combines behavior tree execution with physics simulation for comprehensive testing.
"""

import argparse
import logging
import time
import sys
import os
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional, List

try:
    import yaml
except ImportError:
    print("Warning: PyYAML not available, using fallback configuration")
    yaml = None

try:
    import numpy as np
except ImportError:
    print("Warning: numpy not available, using mock implementation")
    np = None

try:
    import pybullet as p
except ImportError:
    print("Warning: PyBullet not available, using simulation mock")
    p = None

# Import our modules
from sim.world import SimulationWorld
from sim.robot import HumanoidRobot
from sim.teleop_override import TeleopOverride
from behavior_tree.sequence import Sequence
from behavior_tree.fallback import Fallback
from behavior_tree.node import NodeStatus
from behavior_tree.action_nodes import WalkToTarget, PickObject, PlaceObject, WaitForHuman
from skills.walk import WalkToTargetSkill
from skills.pick import PickObjectSkill
from skills.place import PlaceObjectSkill
from skills.custom_workflow import CustomWorkflowSkill


class SimulationLogger:
    """Logging system for simulation runs"""
    
    def __init__(self, run_id: str = None):
        self.run_id = run_id or f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.log_data = {
            "run_id": self.run_id,
            "start_time": datetime.now().isoformat(),
            "workflow": None,
            "steps": [],
            "total_duration": 0.0,
            "failures": 0,
            "recovery_steps": 0,
            "teleop_interventions": 0,
            "final_status": "UNKNOWN"
        }
        
    def log_step(self, step_data: Dict[str, Any]) -> None:
        """Log a single step execution"""
        step_entry = {
            "timestamp": datetime.now().isoformat(),
            **step_data
        }
        self.log_data["steps"].append(step_entry)
        
        # Update counters
        if step_data.get("status") == "FAILURE":
            self.log_data["failures"] += 1
        if "recovery" in step_data.get("skill", "").lower():
            self.log_data["recovery_steps"] += 1
            
    def log_teleop_intervention(self, intervention_data: Dict[str, Any]) -> None:
        """Log teleop intervention"""
        self.log_data["teleop_interventions"] += 1
        self.log_step({
            "skill": "TeleopIntervention",
            "status": "OVERRIDE",
            **intervention_data
        })
        
    def finalize_log(self, final_status: str, total_duration: float) -> None:
        """Finalize the log with summary data"""
        self.log_data["final_status"] = final_status
        self.log_data["total_duration"] = total_duration
        self.log_data["end_time"] = datetime.now().isoformat()
        
    def export_log(self, filename: str = None) -> str:
        """Export log to JSON file"""
        if filename is None:
            filename = f"logs/{self.run_id}.json"
            
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        with open(filename, 'w') as f:
            json.dump(self.log_data, f, indent=2)
            
        return filename
        
    def get_summary(self) -> Dict[str, Any]:
        """Get summary statistics"""
        total_steps = len(self.log_data["steps"])
        success_steps = len([s for s in self.log_data["steps"] if s.get("status") == "SUCCESS"])
        failure_steps = len([s for s in self.log_data["steps"] if s.get("status") == "FAILURE"])
        running_steps = len([s for s in self.log_data["steps"] if s.get("status") == "RUNNING"])
        
        # Calculate task completion rate (SUCCESS vs FAILURE, excluding RUNNING)
        completed_tasks = success_steps + failure_steps
        task_success_rate = success_steps / completed_tasks if completed_tasks > 0 else 0.0
        
        return {
            "run_id": self.run_id,
            "total_steps": total_steps,
            "successful_steps": success_steps,
            "failed_steps": failure_steps,
            "running_steps": running_steps,
            "completed_tasks": completed_tasks,
            "task_success_rate": task_success_rate,
            "failures": self.log_data["failures"],
            "recovery_steps": self.log_data["recovery_steps"],
            "teleop_interventions": self.log_data["teleop_interventions"],
            "duration": self.log_data["total_duration"],
            "final_status": self.log_data["final_status"]
        }


class SimulationRunner:
    """Main simulation runner"""
    
    def __init__(self, use_gui: bool = True, log_level: str = "INFO"):
        self.use_gui = use_gui
        self.log_level = log_level
        self.world = None
        self.robot = None
        self.teleop = None
        self.logger = None
        
    def initialize_simulation(self) -> bool:
        """Initialize the simulation environment"""
        try:
            # Create world
            self.world = SimulationWorld(use_gui=self.use_gui)
            self.world.initialize()
            
            # Create robot
            self.robot = HumanoidRobot()
            success = self.robot.load_robot()
            
            # Give robot reference to world for object manipulation
            self.robot.set_world_reference(self.world)
            
            if not success:
                print("Warning: Failed to load robot URDF, using simplified robot")
                
            # Create teleop system
            self.teleop = TeleopOverride()
            
            # Set up test scenario
            self.world.create_fetch_and_place_scenario()
            
            print(f"Simulation initialized (GUI: {self.use_gui})")
            return True
            
        except Exception as e:
            print(f"Failed to initialize simulation: {e}")
            return False
            
    def shutdown_simulation(self) -> None:
        """Clean shutdown of simulation"""
        if self.world:
            self.world.shutdown()
        print("Simulation shutdown complete")
        
    def load_workflow_config(self, config_path: str) -> Dict[str, Any]:
        """Load workflow configuration from YAML file"""
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Failed to load workflow config: {e}")
            return {}
            
    def create_behavior_tree_from_config(self, workflow_config: Dict[str, Any]):
        """Create behavior tree from workflow configuration"""
        root_type = workflow_config.get('root', 'Sequence')
        nodes_config = workflow_config.get('nodes', [])
        
        nodes = []
        for node_config in nodes_config:
            node = self.create_node_from_config(node_config)
            if node:
                nodes.append(node)
                
        if root_type == 'Sequence':
            return Sequence(nodes)
        elif root_type == 'Fallback':
            return Fallback(nodes)
        else:
            raise ValueError(f"Unknown root type: {root_type}")
    
    def create_node_from_config(self, node_config: Dict[str, Any]):
        """Create a single node from configuration (handles both action nodes and composite nodes)"""
        node_type = node_config.get('type')
        params = node_config.get('parameters', {})
        nested_nodes = node_config.get('nodes', [])
        
        # Handle action nodes
        if node_type == 'WalkToTarget':
            return WalkToTarget(
                target=params['target'],
                tolerance=params.get('tolerance', 0.3)
            )
        elif node_type == 'PickObject':
            return PickObject(object_id=params['object_id'])
        elif node_type == 'PlaceObject':
            return PlaceObject(
                location=params['location'],
                object_id=params.get('object_id')
            )
        elif node_type == 'WaitForHuman':
            return WaitForHuman(duration=params.get('duration', 1.0))
        
        # Handle composite nodes (Sequence, Fallback)
        elif node_type == 'Sequence':
            child_nodes = []
            for child_config in nested_nodes:
                child_node = self.create_node_from_config(child_config)
                if child_node:
                    child_nodes.append(child_node)
            return Sequence(child_nodes)
        elif node_type == 'Fallback':
            child_nodes = []
            for child_config in nested_nodes:
                child_node = self.create_node_from_config(child_config)
                if child_node:
                    child_nodes.append(child_node)
            return Fallback(child_nodes)
        else:
            print(f"Warning: Unknown node type: {node_type}")
            return None
            
    def run_workflow(self, workflow_name: str, config_path: str = None) -> Dict[str, Any]:
        """Run a specific workflow"""
        
        # Initialize logger
        self.logger = SimulationLogger()
        self.logger.log_data["workflow"] = workflow_name
        
        start_time = time.time()
        
        try:
            # Load workflow configuration
            if config_path:
                all_configs = self.load_workflow_config(config_path)
                workflow_config = all_configs.get(workflow_name)
            else:
                workflow_config = self._get_default_workflow(workflow_name)
                
            if not workflow_config:
                raise ValueError(f"Workflow '{workflow_name}' not found")
                
            print(f"Running workflow: {workflow_config.get('name', workflow_name)}")
            print(f"Description: {workflow_config.get('description', 'No description')}")
            
            # Create behavior tree
            behavior_tree = self.create_behavior_tree_from_config(workflow_config)
            
            # Create execution context
            context = type('Context', (), {
                'robot': self.robot,
                'world': self.world,
                'teleop': self.teleop,
                'logger': self.logger
            })()
            
            # Execute behavior tree
            print("Starting workflow execution...")
            result = self._execute_behavior_tree(behavior_tree, context)
            
            total_duration = time.time() - start_time
            
            # Finalize logging
            self.logger.finalize_log(result.value, total_duration)
            
            # Print summary
            summary = self.logger.get_summary()
            self._print_execution_summary(summary)
            
            return summary
            
        except Exception as e:
            print(f"Workflow execution failed: {e}")
            total_duration = time.time() - start_time
            if self.logger:
                self.logger.finalize_log("ERROR", total_duration)
            return {"error": str(e), "duration": total_duration}
            
    def _execute_behavior_tree(self, behavior_tree, context) -> NodeStatus:
        """Execute behavior tree with monitoring"""
        
        max_iterations = 1000  # Prevent infinite loops
        iteration = 0
        
        while iteration < max_iterations:
            # Step simulation
            if self.world:
                self.world.step_simulation()
                
            # Tick behavior tree
            result = behavior_tree.tick(context)
            
            # Log if we have a specific node result
            if hasattr(behavior_tree, 'children'):
                self._log_node_execution(behavior_tree, result)
                
            # Check if complete
            if result in [NodeStatus.SUCCESS, NodeStatus.FAILURE]:
                return result
                
            # Check for teleop override
            if self.teleop and self.teleop.is_active():
                self.logger.log_teleop_intervention({
                    "intervention_type": "manual_override",
                    "iteration": iteration
                })
                
            iteration += 1
            time.sleep(0.01)  # Small delay for real-time execution
            
        # Timeout
        print(f"Behavior tree execution timeout after {max_iterations} iterations")
        return NodeStatus.FAILURE
        
    def _log_node_execution(self, node, result: NodeStatus) -> None:
        """Log execution of behavior tree nodes"""
        if self.logger:
            self.logger.log_step({
                "skill": node.__class__.__name__,
                "status": result.value,
                "duration": node.get_duration() or 0.0
            })
            
    def _get_default_workflow(self, workflow_name: str) -> Dict[str, Any]:
        """Get default workflow configurations"""
        default_workflows = {
            "simple_walk": {
                "name": "simple_walk",
                "description": "Simple walking test",
                "root": "Sequence",
                "nodes": [
                    {
                        "type": "WalkToTarget",
                        "parameters": {"target": [1.0, 1.0, 0.0], "tolerance": 0.3}
                    }
                ]
            },
            "fetch_and_place": {
                "name": "fetch_and_place",
                "description": "Basic fetch and place operation",
                "root": "Sequence", 
                "nodes": [
                    {
                        "type": "WalkToTarget",
                        "parameters": {"target": [1.5, 0.8, 0.0], "tolerance": 0.3}
                    },
                    {
                        "type": "PickObject",
                        "parameters": {"object_id": "box_01"}
                    },
                    {
                        "type": "WalkToTarget", 
                        "parameters": {"target": [2.5, 1.0, 0.0], "tolerance": 0.3}
                    },
                    {
                        "type": "PlaceObject",
                        "parameters": {"location": [2.5, 1.0, 0.1]}
                    }
                ]
            }
        }
        return default_workflows.get(workflow_name)
        
    def _print_execution_summary(self, summary: Dict[str, Any]) -> None:
        """Print execution summary"""
        print("\n" + "="*50)
        print("EXECUTION SUMMARY")
        print("="*50)
        print(f"Run ID: {summary['run_id']}")
        print(f"Total Steps: {summary['total_steps']}")
        print(f"  - Running Steps: {summary['running_steps']}")
        print(f"  - Successful Steps: {summary['successful_steps']}")
        print(f"  - Failed Steps: {summary['failed_steps']}")
        print(f"Completed Tasks: {summary['completed_tasks']}")
        print(f"Success Rate: {summary['task_success_rate']:.1%}")
        print(f"Failures: {summary['failures']}")
        print(f"Recovery Steps: {summary['recovery_steps']}")
        print(f"Teleop Interventions: {summary['teleop_interventions']}")
        print(f"Duration: {summary['duration']:.2f} seconds")
        print(f"Final Status: {summary['final_status']}")
        print("="*50)
        
    def run_interactive_mode(self) -> None:
        """Run in interactive mode with teleop capabilities"""
        print("Starting interactive mode...")
        print("Available commands:")
        print("  walk <x> <y> - Walk to position")
        print("  pick <object_id> - Pick up object")
        print("  place <x> <y> <z> - Place object")
        print("  override - Start teleop override")
        print("  end_override - End teleop override") 
        print("  status - Show robot status")
        print("  quit - Exit simulation")
        
        while True:
            try:
                command = input("\n> ").strip().lower()
                
                if command == "quit":
                    break
                elif command == "status":
                    self._print_robot_status()
                elif command == "override":
                    self.teleop.start_override("manual_command")
                elif command == "end_override":
                    self.teleop.end_override()
                elif command.startswith("walk"):
                    parts = command.split()
                    if len(parts) == 3:
                        x, y = float(parts[1]), float(parts[2])
                        self._manual_walk([x, y, 0.0])
                elif command.startswith("pick"):
                    parts = command.split()
                    if len(parts) == 2:
                        self._manual_pick(parts[1])
                elif command.startswith("place"):
                    parts = command.split()
                    if len(parts) == 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        self._manual_place([x, y, z])
                        
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
                
        print("Exiting interactive mode...")
        
    def _print_robot_status(self) -> None:
        """Print current robot status"""
        pos = self.robot.get_position()
        held = self.robot.get_held_objects()
        
        print(f"Robot Position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        print(f"Held Objects: {held if held else 'None'}")
        print(f"Walking: {self.robot.is_walking()}")
        print(f"Teleop Active: {self.teleop.is_active()}")
        
    def _manual_walk(self, target: List[float]) -> None:
        """Manual walk command"""
        print(f"Walking to {target}")
        self.robot.start_walking_to(target)
        
    def _manual_pick(self, object_id: str) -> None:
        """Manual pick command"""
        obj_pos = self.world.get_object_position(object_id)
        if obj_pos:
            success = self.robot.attempt_pick(object_id, obj_pos)
            print(f"Pick {object_id}: {'Success' if success else 'Failed'}")
        else:
            print(f"Object {object_id} not found")
            
    def _manual_place(self, location: List[float]) -> None:
        """Manual place command"""
        success = self.robot.attempt_place(location)
        print(f"Place at {location}: {'Success' if success else 'Failed'}")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Humanoid Skill Simulator")
    parser.add_argument("--workflow", type=str, default="fetch_and_place",
                       help="Workflow to execute")
    parser.add_argument("--config", type=str, default="configs/workflows.yaml",
                       help="Path to workflow configuration file")
    parser.add_argument("--gui", action="store_true", default=True,
                       help="Enable GUI visualization")
    parser.add_argument("--no-gui", action="store_true",
                       help="Disable GUI visualization")
    parser.add_argument("--interactive", action="store_true",
                       help="Run in interactive mode")
    parser.add_argument("--export-logs", action="store_true",
                       help="Export execution logs to file")
    parser.add_argument("--log-level", type=str, default="INFO",
                       choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                       help="Logging level")
    
    args = parser.parse_args()
    
    # Handle GUI settings
    use_gui = args.gui and not args.no_gui
    
    # Create simulation runner
    runner = SimulationRunner(use_gui=use_gui, log_level=args.log_level)
    
    try:
        # Initialize simulation
        if not runner.initialize_simulation():
            print("Failed to initialize simulation")
            return 1
            
        if args.interactive:
            # Run interactive mode
            runner.run_interactive_mode()
        else:
            # Run specified workflow
            config_path = args.config if os.path.exists(args.config) else None
            summary = runner.run_workflow(args.workflow, config_path)
            
            # Export logs if requested
            if args.export_logs and runner.logger:
                log_file = runner.logger.export_log()
                print(f"\nLogs exported to: {log_file}")
                
        return 0
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
        return 0
    except Exception as e:
        print(f"Simulation error: {e}")
        return 1
    finally:
        runner.shutdown_simulation()


if __name__ == "__main__":
    sys.exit(main())
