"""
Scenario tests for randomized simulation-to-deployment testing
"""

import random
import time
import json
import os
import sys
import argparse
from typing import Dict, List, Any, Tuple
from datetime import datetime

# Add parent directory for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.world import SimulationWorld
from sim.robot import HumanoidRobot
from sim.teleop_override import TeleopOverride
from behavior_tree import Sequence, Fallback, WalkToTarget, PickObject, PlaceObject, WaitForHuman, NodeStatus


class ScenarioGenerator:
    """Generates randomized test scenarios for deployment testing"""
    
    def __init__(self, seed: int = None):
        if seed is not None:
            random.seed(seed)
            
    def generate_object_placement_scenario(self, num_objects: int = 3) -> Dict[str, Any]:
        """Generate scenario with randomly placed objects"""
        
        objects = []
        for i in range(num_objects):
            obj_id = f"test_obj_{i:02d}"
            
            # Random position within reachable area
            position = [
                random.uniform(0.5, 2.5),  # X: 0.5m to 2.5m
                random.uniform(0.2, 1.8),  # Y: 0.2m to 1.8m
                0.05                       # Z: on ground
            ]
            
            objects.append({
                'id': obj_id,
                'position': position,
                'size': [0.05, 0.05, 0.05],
                'color': [random.random(), random.random(), random.random(), 1.0]
            })
            
        return {
            'scenario_type': 'object_placement',
            'objects': objects,
            'description': f'Random placement of {num_objects} objects'
        }
        
    def generate_fetch_task_scenario(self, num_tasks: int = 2) -> Dict[str, Any]:
        """Generate fetch and place task scenario"""
        
        tasks = []
        used_positions = set()
        
        for i in range(num_tasks):
            # Ensure unique positions
            while True:
                pick_pos = [
                    round(random.uniform(1.0, 2.0), 1),
                    round(random.uniform(0.5, 1.5), 1),
                    0.05
                ]
                place_pos = [
                    round(random.uniform(2.5, 3.5), 1),
                    round(random.uniform(0.5, 1.5), 1),
                    0.1
                ]
                
                pick_key = (pick_pos[0], pick_pos[1])
                place_key = (place_pos[0], place_pos[1])
                
                if pick_key not in used_positions and place_key not in used_positions:
                    used_positions.add(pick_key)
                    used_positions.add(place_key)
                    break
                    
            tasks.append({
                'object_id': f"task_obj_{i:02d}",
                'pick_location': pick_pos,
                'place_location': place_pos,
                'priority': random.randint(1, 5)
            })
            
        # Sort by priority
        tasks.sort(key=lambda x: x['priority'], reverse=True)
        
        return {
            'scenario_type': 'fetch_tasks',
            'tasks': tasks,
            'description': f'Fetch and place {num_tasks} objects by priority'
        }
        
    def generate_obstacle_scenario(self, num_obstacles: int = 2) -> Dict[str, Any]:
        """Generate scenario with obstacles"""
        
        obstacles = []
        for i in range(num_obstacles):
            obstacle = {
                'id': f"obstacle_{i:02d}",
                'position': [
                    random.uniform(1.0, 2.0),
                    random.uniform(0.5, 1.5),
                    0.2
                ],
                'size': [
                    random.uniform(0.1, 0.3),
                    random.uniform(0.1, 0.3),
                    random.uniform(0.2, 0.4)
                ],
                'color': [0.5, 0.3, 0.1, 1.0]  # Brown obstacles
            }
            obstacles.append(obstacle)
            
        # Add target object
        target_object = {
            'id': 'target_object',
            'position': [random.uniform(1.5, 2.5), random.uniform(0.8, 1.2), 0.05],
            'size': [0.05, 0.05, 0.05],
            'color': [1.0, 0.0, 0.0, 1.0]  # Red target
        }
        
        return {
            'scenario_type': 'obstacle_navigation',
            'obstacles': obstacles,
            'target_object': target_object,
            'description': f'Navigate around {num_obstacles} obstacles to reach target'
        }
        
    def generate_patrol_scenario(self, num_waypoints: int = 4) -> Dict[str, Any]:
        """Generate patrol scenario with random waypoints"""
        
        waypoints = []
        
        # Start at origin
        waypoints.append([0.0, 0.0, 0.0])
        
        # Generate random waypoints
        for i in range(num_waypoints - 1):
            waypoint = [
                random.uniform(-1.0, 3.0),
                random.uniform(-1.0, 2.0),
                0.0
            ]
            waypoints.append(waypoint)
            
        # Return to start
        waypoints.append([0.0, 0.0, 0.0])
        
        return {
            'scenario_type': 'patrol',
            'waypoints': waypoints,
            'inspection_duration': random.uniform(1.0, 3.0),
            'description': f'Patrol {num_waypoints} waypoints with inspection'
        }
        
    def generate_failure_injection_scenario(self) -> Dict[str, Any]:
        """Generate scenario with predefined failure points"""
        
        failure_types = [
            'sensor_noise',
            'object_drift',
            'ik_failure',
            'grasp_failure',
            'timeout',
            'collision'
        ]
        
        # Select random failure types
        num_failures = random.randint(1, 3)
        selected_failures = random.sample(failure_types, num_failures)
        
        failures = []
        for failure_type in selected_failures:
            failure = {
                'type': failure_type,
                'trigger_step': random.randint(1, 5),
                'severity': random.uniform(0.1, 1.0),
                'description': f'Inject {failure_type} at step {random.randint(1, 5)}'
            }
            failures.append(failure)
            
        return {
            'scenario_type': 'failure_injection',
            'failures': failures,
            'description': f'Test resilience with {num_failures} injected failures'
        }


class ScenarioTestRunner:
    """Runs scenario tests and collects deployment readiness metrics"""
    
    def __init__(self, use_gui: bool = False):
        self.use_gui = use_gui
        self.world = None
        self.robot = None
        self.teleop = None
        self.generator = ScenarioGenerator()
        self.test_results = []
        
    def setup_simulation(self) -> bool:
        """Initialize simulation environment"""
        try:
            self.world = SimulationWorld(use_gui=self.use_gui)
            self.world.initialize()
            
            self.robot = HumanoidRobot()
            self.robot.load_robot()
            
            self.teleop = TeleopOverride()
            
            return True
        except Exception as e:
            print(f"Failed to setup simulation: {e}")
            return False
            
    def teardown_simulation(self) -> None:
        """Clean up simulation"""
        if self.world:
            self.world.shutdown()
            
    def setup_scenario(self, scenario: Dict[str, Any]) -> bool:
        """Setup simulation environment for specific scenario"""
        
        # Reset environment
        self.world.reset_scene()
        self.robot.reset_pose()
        
        scenario_type = scenario['scenario_type']
        
        try:
            if scenario_type == 'object_placement':
                return self._setup_object_placement_scenario(scenario)
            elif scenario_type == 'fetch_tasks':
                return self._setup_fetch_task_scenario(scenario)
            elif scenario_type == 'obstacle_navigation':
                return self._setup_obstacle_scenario(scenario)
            elif scenario_type == 'patrol':
                return self._setup_patrol_scenario(scenario)
            elif scenario_type == 'failure_injection':
                return self._setup_failure_injection_scenario(scenario)
            else:
                print(f"Unknown scenario type: {scenario_type}")
                return False
                
        except Exception as e:
            print(f"Error setting up scenario: {e}")
            return False
            
    def _setup_object_placement_scenario(self, scenario: Dict[str, Any]) -> bool:
        """Setup object placement scenario"""
        
        for obj in scenario['objects']:
            self.world.add_simple_box(
                obj['id'],
                obj['position'],
                obj['size'],
                obj['color']
            )
            
        return True
        
    def _setup_fetch_task_scenario(self, scenario: Dict[str, Any]) -> bool:
        """Setup fetch task scenario"""
        
        for task in scenario['tasks']:
            self.world.add_simple_box(
                task['object_id'],
                task['pick_location'],
                [0.05, 0.05, 0.05],
                [random.random(), random.random(), random.random(), 1.0]
            )
            
        return True
        
    def _setup_obstacle_scenario(self, scenario: Dict[str, Any]) -> bool:
        """Setup obstacle navigation scenario"""
        
        # Add obstacles
        for obstacle in scenario['obstacles']:
            self.world.add_simple_box(
                obstacle['id'],
                obstacle['position'],
                obstacle['size'],
                obstacle['color']
            )
            
        # Add target object
        target = scenario['target_object']
        self.world.add_simple_box(
            target['id'],
            target['position'],
            target['size'],
            target['color']
        )
        
        return True
        
    def _setup_patrol_scenario(self, scenario: Dict[str, Any]) -> bool:
        """Setup patrol scenario"""
        # No special objects needed for patrol
        return True
        
    def _setup_failure_injection_scenario(self, scenario: Dict[str, Any]) -> bool:
        """Setup failure injection scenario"""
        
        # Add basic objects for manipulation testing
        self.world.add_simple_box("test_obj", [1.5, 0.8, 0.05])
        
        # Configure failure conditions
        for failure in scenario['failures']:
            if failure['type'] == 'sensor_noise':
                self.world.set_noise_config(sensor_noise=failure['severity'] * 0.1)
            elif failure['type'] == 'object_drift':
                # Will be triggered during execution
                pass
                
        return True
        
    def create_behavior_tree_for_scenario(self, scenario: Dict[str, Any]):
        """Create appropriate behavior tree for scenario"""
        
        scenario_type = scenario['scenario_type']
        
        if scenario_type == 'object_placement':
            # Simple walk to each object
            nodes = []
            for obj in scenario['objects']:
                nodes.append(WalkToTarget(target=obj['position'], tolerance=0.3))
                
            return Sequence(nodes)
            
        elif scenario_type == 'fetch_tasks':
            # Fetch and place each task
            nodes = []
            for task in scenario['tasks']:
                nodes.extend([
                    WalkToTarget(target=task['pick_location'], tolerance=0.3),
                    PickObject(object_id=task['object_id']),
                    WalkToTarget(target=task['place_location'], tolerance=0.3),
                    PlaceObject(location=task['place_location'])
                ])
                
            return Sequence(nodes)
            
        elif scenario_type == 'obstacle_navigation':
            # Navigate to target object
            target = scenario['target_object']
            return Sequence([
                WalkToTarget(target=target['position'], tolerance=0.3),
                PickObject(object_id=target['id'])
            ])
            
        elif scenario_type == 'patrol':
            # Walk to each waypoint with inspection pause
            nodes = []
            for waypoint in scenario['waypoints']:
                nodes.extend([
                    WalkToTarget(target=waypoint, tolerance=0.3),
                    WaitForHuman(duration=scenario.get('inspection_duration', 2.0))
                ])
                
            return Sequence(nodes)
            
        elif scenario_type == 'failure_injection':
            # Basic fetch and place with failure injection
            return Sequence([
                WalkToTarget(target=[1.5, 0.8, 0.0], tolerance=0.3),
                PickObject(object_id="test_obj"),
                WalkToTarget(target=[2.5, 1.0, 0.0], tolerance=0.3),
                PlaceObject(location=[2.5, 1.0, 0.1])
            ])
            
        else:
            # Default simple behavior
            return Sequence([WalkToTarget(target=[1.0, 1.0, 0.0], tolerance=0.3)])
            
    def inject_scenario_failures(self, scenario: Dict[str, Any], current_step: int) -> None:
        """Inject failures based on scenario configuration"""
        
        if scenario['scenario_type'] != 'failure_injection':
            return
            
        for failure in scenario['failures']:
            if failure['trigger_step'] == current_step:
                failure_type = failure['type']
                severity = failure['severity']
                
                if failure_type == 'object_drift':
                    # Drift objects
                    for obj_id in self.world.get_all_objects():
                        self.world.inject_object_drift(obj_id, severity * 0.1)
                        
                elif failure_type == 'grasp_failure':
                    # Force grasp to fail (would need robot integration)
                    pass
                    
                elif failure_type == 'sensor_noise':
                    # Increase sensor noise
                    self.world.set_noise_config(sensor_noise=severity * 0.2)
                    
    def run_scenario_test(self, scenario: Dict[str, Any], timeout: float = 60.0) -> Dict[str, Any]:
        """Run a single scenario test"""
        
        print(f"Running scenario: {scenario['description']}")
        
        # Setup scenario
        if not self.setup_scenario(scenario):
            return {
                'scenario': scenario,
                'status': 'SETUP_FAILED',
                'error': 'Failed to setup scenario',
                'duration': 0.0
            }
            
        # Create behavior tree
        behavior_tree = self.create_behavior_tree_for_scenario(scenario)
        
        # Create execution context
        context = type('Context', (), {
            'robot': self.robot,
            'world': self.world,
            'teleop': self.teleop
        })()
        
        # Execute test
        start_time = time.time()
        step_count = 0
        max_steps = 1000
        
        try:
            while step_count < max_steps:
                # Check timeout
                if time.time() - start_time > timeout:
                    return {
                        'scenario': scenario,
                        'status': 'TIMEOUT',
                        'duration': time.time() - start_time,
                        'steps_completed': step_count
                    }
                    
                # Inject failures if needed
                self.inject_scenario_failures(scenario, step_count)
                
                # Step simulation
                self.world.step_simulation()
                
                # Tick behavior tree
                result = behavior_tree.tick(context)
                
                if result in [NodeStatus.SUCCESS, NodeStatus.FAILURE]:
                    duration = time.time() - start_time
                    
                    return {
                        'scenario': scenario,
                        'status': result.value,
                        'duration': duration,
                        'steps_completed': step_count,
                        'success': result == NodeStatus.SUCCESS
                    }
                    
                step_count += 1
                time.sleep(0.01)  # Small delay for real-time execution
                
            # Max steps reached
            return {
                'scenario': scenario,
                'status': 'MAX_STEPS',
                'duration': time.time() - start_time,
                'steps_completed': step_count
            }
            
        except Exception as e:
            return {
                'scenario': scenario,
                'status': 'ERROR',
                'error': str(e),
                'duration': time.time() - start_time,
                'steps_completed': step_count
            }
            
    def run_scenario_test_suite(self, num_tests: int = 10, random_seed: int = None) -> Dict[str, Any]:
        """Run a comprehensive scenario test suite"""
        
        if random_seed is not None:
            self.generator = ScenarioGenerator(seed=random_seed)
            
        print(f"Running scenario test suite with {num_tests} tests...")
        
        # Generate test scenarios
        scenarios = []
        scenario_types = [
            'object_placement',
            'fetch_tasks', 
            'obstacle_navigation',
            'patrol',
            'failure_injection'
        ]
        
        for i in range(num_tests):
            scenario_type = random.choice(scenario_types)
            
            if scenario_type == 'object_placement':
                scenario = self.generator.generate_object_placement_scenario(random.randint(2, 5))
            elif scenario_type == 'fetch_tasks':
                scenario = self.generator.generate_fetch_task_scenario(random.randint(1, 3))
            elif scenario_type == 'obstacle_navigation':
                scenario = self.generator.generate_obstacle_scenario(random.randint(1, 3))
            elif scenario_type == 'patrol':
                scenario = self.generator.generate_patrol_scenario(random.randint(3, 6))
            elif scenario_type == 'failure_injection':
                scenario = self.generator.generate_failure_injection_scenario()
                
            scenarios.append(scenario)
            
        # Run tests
        results = []
        successful_tests = 0
        
        for i, scenario in enumerate(scenarios):
            print(f"Test {i+1}/{num_tests}: {scenario['scenario_type']}")
            
            result = self.run_scenario_test(scenario)
            results.append(result)
            
            if result.get('success', False):
                successful_tests += 1
                
        # Compile summary
        summary = {
            'total_tests': num_tests,
            'successful_tests': successful_tests,
            'success_rate': successful_tests / num_tests if num_tests > 0 else 0.0,
            'results': results,
            'random_seed': random_seed,
            'timestamp': datetime.now().isoformat()
        }
        
        self._print_test_suite_summary(summary)
        
        return summary
        
    def _print_test_suite_summary(self, summary: Dict[str, Any]) -> None:
        """Print test suite summary"""
        
        print("\n" + "="*60)
        print("SCENARIO TEST SUITE SUMMARY")
        print("="*60)
        print(f"Total Tests: {summary['total_tests']}")
        print(f"Successful Tests: {summary['successful_tests']}")
        print(f"Success Rate: {summary['success_rate']:.1%}")
        
        # Analyze by scenario type
        type_stats = {}
        for result in summary['results']:
            scenario_type = result['scenario']['scenario_type']
            if scenario_type not in type_stats:
                type_stats[scenario_type] = {'total': 0, 'success': 0}
                
            type_stats[scenario_type]['total'] += 1
            if result.get('success', False):
                type_stats[scenario_type]['success'] += 1
                
        print(f"\nBy Scenario Type:")
        for scenario_type, stats in type_stats.items():
            success_rate = stats['success'] / stats['total'] if stats['total'] > 0 else 0.0
            print(f"  {scenario_type}: {stats['success']}/{stats['total']} ({success_rate:.1%})")
            
        print("="*60)
        
    def export_results(self, summary: Dict[str, Any], filename: str) -> None:
        """Export test results to file"""
        
        with open(filename, 'w') as f:
            json.dump(summary, f, indent=2)
            
        print(f"Test results exported to {filename}")


def main():
    """Main entry point for scenario testing"""
    
    parser = argparse.ArgumentParser(description="Scenario Test Suite")
    parser.add_argument("--num-tests", type=int, default=10,
                       help="Number of tests to run")
    parser.add_argument("--random-seed", type=int,
                       help="Random seed for reproducible tests")
    parser.add_argument("--gui", action="store_true",
                       help="Enable GUI visualization")
    parser.add_argument("--export", type=str,
                       help="Export results to file")
    parser.add_argument("--scenario-type", type=str,
                       choices=['object_placement', 'fetch_tasks', 'obstacle_navigation', 'patrol', 'failure_injection'],
                       help="Run only specific scenario type")
    
    args = parser.parse_args()
    
    # Create test runner
    runner = ScenarioTestRunner(use_gui=args.gui)
    
    try:
        # Setup simulation
        if not runner.setup_simulation():
            print("Failed to setup simulation")
            return 1
            
        if args.scenario_type:
            # Run single scenario type
            generator = ScenarioGenerator(seed=args.random_seed)
            
            if args.scenario_type == 'object_placement':
                scenario = generator.generate_object_placement_scenario()
            elif args.scenario_type == 'fetch_tasks':
                scenario = generator.generate_fetch_task_scenario()
            elif args.scenario_type == 'obstacle_navigation':
                scenario = generator.generate_obstacle_scenario()
            elif args.scenario_type == 'patrol':
                scenario = generator.generate_patrol_scenario()
            elif args.scenario_type == 'failure_injection':
                scenario = generator.generate_failure_injection_scenario()
                
            result = runner.run_scenario_test(scenario)
            print(f"\nTest Result: {result}")
        else:
            # Run full test suite
            summary = runner.run_scenario_test_suite(args.num_tests, args.random_seed)
            
            # Export results if requested
            if args.export:
                runner.export_results(summary, args.export)
                
        return 0
        
    except KeyboardInterrupt:
        print("\nScenario testing interrupted")
        return 0
    except Exception as e:
        print(f"Scenario testing failed: {e}")
        return 1
    finally:
        runner.teardown_simulation()


if __name__ == "__main__":
    sys.exit(main())
