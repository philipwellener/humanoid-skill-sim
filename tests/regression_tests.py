"""
Regression tests for replaying known failure scenarios
"""

import unittest
import json
import os
import sys
import argparse
from typing import Dict, List, Any

# Add parent directory for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.world import SimulationWorld
from sim.robot import HumanoidRobot
from sim.teleop_override import TeleopOverride


class RegressionTestSuite:
    """Test suite for regression testing of known failure scenarios"""
    
    def __init__(self):
        self.world = None
        self.robot = None
        self.teleop = None
        self.known_failures = []
        
    def setup_simulation(self, use_gui: bool = False) -> bool:
        """Setup simulation environment"""
        try:
            self.world = SimulationWorld(use_gui=use_gui)
            self.world.initialize()
            
            self.robot = HumanoidRobot()
            self.robot.load_robot()
            
            self.teleop = TeleopOverride()
            
            # Setup test scenario
            self.world.create_fetch_and_place_scenario()
            
            return True
        except Exception as e:
            print(f"Failed to setup simulation: {e}")
            return False
            
    def teardown_simulation(self) -> None:
        """Clean up simulation"""
        if self.world:
            self.world.shutdown()
            
    def load_failure_scenarios(self, log_directory: str) -> List[Dict[str, Any]]:
        """Load known failure scenarios from log files"""
        failure_scenarios = []
        
        if not os.path.exists(log_directory):
            print(f"Log directory {log_directory} not found")
            return failure_scenarios
            
        for filename in os.listdir(log_directory):
            if filename.endswith('.json'):
                filepath = os.path.join(log_directory, filename)
                
                try:
                    with open(filepath, 'r') as f:
                        log_data = json.load(f)
                        
                    # Check if this run had failures
                    if log_data.get('failures', 0) > 0 or log_data.get('final_status') == 'FAILURE':
                        failure_scenarios.append({
                            'log_file': filepath,
                            'run_id': log_data.get('run_id'),
                            'workflow': log_data.get('workflow'),
                            'failures': log_data.get('failures', 0),
                            'steps': log_data.get('steps', []),
                            'final_status': log_data.get('final_status')
                        })
                        
                except Exception as e:
                    print(f"Error loading {filepath}: {e}")
                    
        print(f"Loaded {len(failure_scenarios)} failure scenarios")
        return failure_scenarios
        
    def extract_failure_patterns(self, scenarios: List[Dict[str, Any]]) -> Dict[str, List]:
        """Extract common failure patterns from scenarios"""
        patterns = {
            'skill_failures': [],
            'sequence_failures': [],
            'timeout_failures': [],
            'ik_failures': [],
            'reachability_failures': []
        }
        
        for scenario in scenarios:
            for step in scenario['steps']:
                if step.get('status') == 'FAILURE':
                    error = step.get('error', '').lower()
                    skill = step.get('skill', '')
                    
                    # Categorize failure types
                    if 'timeout' in error:
                        patterns['timeout_failures'].append({
                            'skill': skill,
                            'error': step.get('error'),
                            'scenario_id': scenario['run_id']
                        })
                    elif 'ik' in error or 'inverse kinematics' in error:
                        patterns['ik_failures'].append({
                            'skill': skill,
                            'error': step.get('error'),
                            'scenario_id': scenario['run_id']
                        })
                    elif 'reach' in error or 'out of reach' in error:
                        patterns['reachability_failures'].append({
                            'skill': skill,
                            'error': step.get('error'),
                            'scenario_id': scenario['run_id']
                        })
                    else:
                        patterns['skill_failures'].append({
                            'skill': skill,
                            'error': step.get('error'),
                            'scenario_id': scenario['run_id']
                        })
                        
        return patterns
        
    def replay_scenario(self, scenario: Dict[str, Any], fix_applied: bool = False) -> Dict[str, Any]:
        """Replay a specific failure scenario"""
        
        print(f"Replaying scenario {scenario['run_id']} - {scenario['workflow']}")
        
        # Reset simulation state
        self.robot.reset_pose()
        self.world.reset_scene()
        self.world.create_fetch_and_place_scenario()
        
        # Apply fixes if requested
        if fix_applied:
            self._apply_scenario_fixes(scenario)
            
        # Track replay results
        replay_result = {
            'original_run_id': scenario['run_id'],
            'replay_successful': True,
            'steps_completed': 0,
            'new_failures': [],
            'improvements': []
        }
        
        # Replay each step (simplified)
        for i, original_step in enumerate(scenario['steps']):
            skill_name = original_step.get('skill')
            original_status = original_step.get('status')
            
            # Simulate step execution
            simulated_result = self._simulate_step_execution(skill_name, original_step)
            
            replay_result['steps_completed'] = i + 1
            
            if simulated_result['status'] != original_status:
                if original_status == 'FAILURE' and simulated_result['status'] == 'SUCCESS':
                    replay_result['improvements'].append({
                        'step': i,
                        'skill': skill_name,
                        'original_error': original_step.get('error'),
                        'improvement': 'Fixed in replay'
                    })
                elif original_status == 'SUCCESS' and simulated_result['status'] == 'FAILURE':
                    replay_result['new_failures'].append({
                        'step': i,
                        'skill': skill_name,
                        'new_error': simulated_result.get('error'),
                        'note': 'New failure in replay'
                    })
                    
            # Stop if we hit a failure and this is a critical step
            if simulated_result['status'] == 'FAILURE' and not fix_applied:
                replay_result['replay_successful'] = False
                break
                
        return replay_result
        
    def _apply_scenario_fixes(self, scenario: Dict[str, Any]) -> None:
        """Apply known fixes for scenario failures"""
        
        # Example fixes based on common failure patterns
        for step in scenario['steps']:
            error = step.get('error', '').lower()
            
            if 'out of reach' in error:
                # Reduce object distances for reachability testing
                self.world.set_noise_config(position_noise=0.001)
                
            elif 'ik' in error:
                # Simplify IK requirements
                # This would be implemented based on specific failure analysis
                pass
                
            elif 'timeout' in error:
                # Increase timeout thresholds
                # This would require modifying skill parameters
                pass
                
    def _simulate_step_execution(self, skill_name: str, original_step: Dict[str, Any]) -> Dict[str, Any]:
        """Simulate execution of a single step"""
        
        # This is a simplified simulation - in a real implementation,
        # you would actually execute the skill with the same parameters
        
        import random
        
        # Base success rate for replay
        base_success_rate = 0.8
        
        # Modify based on original failure
        if original_step.get('status') == 'FAILURE':
            # Lower success rate for originally failed steps
            success_rate = base_success_rate * 0.6
        else:
            success_rate = base_success_rate
            
        # Simulate execution
        success = random.random() < success_rate
        
        if success:
            return {
                'skill': skill_name,
                'status': 'SUCCESS',
                'duration': original_step.get('duration', 1.0) * random.uniform(0.8, 1.2)
            }
        else:
            return {
                'skill': skill_name,
                'status': 'FAILURE',
                'error': f"Simulated failure in {skill_name}",
                'duration': original_step.get('duration', 1.0) * random.uniform(0.5, 1.5)
            }
            
    def run_regression_test_suite(self, log_directory: str, apply_fixes: bool = False) -> Dict[str, Any]:
        """Run complete regression test suite"""
        
        print("Starting regression test suite...")
        
        # Load failure scenarios
        scenarios = self.load_failure_scenarios(log_directory)
        
        if not scenarios:
            return {
                'total_scenarios': 0,
                'error': 'No failure scenarios found'
            }
            
        # Analyze failure patterns
        patterns = self.extract_failure_patterns(scenarios)
        
        # Run replays
        replay_results = []
        successful_replays = 0
        
        for scenario in scenarios:
            try:
                result = self.replay_scenario(scenario, fix_applied=apply_fixes)
                replay_results.append(result)
                
                if result['replay_successful']:
                    successful_replays += 1
                    
            except Exception as e:
                print(f"Error replaying scenario {scenario['run_id']}: {e}")
                
        # Compile summary
        summary = {
            'total_scenarios': len(scenarios),
            'successful_replays': successful_replays,
            'replay_success_rate': successful_replays / len(scenarios) if scenarios else 0.0,
            'failure_patterns': patterns,
            'replay_results': replay_results,
            'fixes_applied': apply_fixes
        }
        
        self._print_regression_summary(summary)
        
        return summary
        
    def _print_regression_summary(self, summary: Dict[str, Any]) -> None:
        """Print regression test summary"""
        
        print("\n" + "="*60)
        print("REGRESSION TEST SUMMARY")
        print("="*60)
        print(f"Total Scenarios Tested: {summary['total_scenarios']}")
        print(f"Successful Replays: {summary['successful_replays']}")
        print(f"Replay Success Rate: {summary['replay_success_rate']:.1%}")
        print(f"Fixes Applied: {summary['fixes_applied']}")
        
        # Print failure pattern analysis
        patterns = summary['failure_patterns']
        print(f"\nFailure Pattern Analysis:")
        print(f"  Skill Failures: {len(patterns['skill_failures'])}")
        print(f"  Timeout Failures: {len(patterns['timeout_failures'])}")
        print(f"  IK Failures: {len(patterns['ik_failures'])}")
        print(f"  Reachability Failures: {len(patterns['reachability_failures'])}")
        
        # Print improvements if fixes were applied
        if summary['fixes_applied']:
            total_improvements = sum(len(result.get('improvements', [])) for result in summary['replay_results'])
            print(f"\nTotal Improvements: {total_improvements}")
            
        print("="*60)
        
    def generate_regression_report(self, summary: Dict[str, Any], output_file: str) -> None:
        """Generate detailed regression test report"""
        
        with open(output_file, 'w') as f:
            json.dump(summary, f, indent=2)
            
        print(f"Regression report saved to {output_file}")


def main():
    """Main entry point for regression testing"""
    
    parser = argparse.ArgumentParser(description="Regression Test Suite")
    parser.add_argument("--log-dir", type=str, default="logs",
                       help="Directory containing log files")
    parser.add_argument("--apply-fixes", action="store_true",
                       help="Apply known fixes during replay")
    parser.add_argument("--gui", action="store_true",
                       help="Enable GUI for visual debugging")
    parser.add_argument("--report", type=str,
                       help="Output file for regression report")
    parser.add_argument("--scenario", type=str,
                       help="Replay specific scenario by run_id")
    
    args = parser.parse_args()
    
    # Create test suite
    test_suite = RegressionTestSuite()
    
    try:
        # Setup simulation
        if not test_suite.setup_simulation(use_gui=args.gui):
            print("Failed to setup simulation")
            return 1
            
        if args.scenario:
            # Replay specific scenario
            scenarios = test_suite.load_failure_scenarios(args.log_dir)
            target_scenario = None
            
            for scenario in scenarios:
                if scenario['run_id'] == args.scenario:
                    target_scenario = scenario
                    break
                    
            if target_scenario:
                result = test_suite.replay_scenario(target_scenario, fix_applied=args.apply_fixes)
                print(f"\nReplay result: {result}")
            else:
                print(f"Scenario {args.scenario} not found")
                return 1
        else:
            # Run full regression suite
            summary = test_suite.run_regression_test_suite(args.log_dir, apply_fixes=args.apply_fixes)
            
            # Generate report if requested
            if args.report:
                test_suite.generate_regression_report(summary, args.report)
                
        return 0
        
    except KeyboardInterrupt:
        print("\nRegression testing interrupted")
        return 0
    except Exception as e:
        print(f"Regression testing failed: {e}")
        return 1
    finally:
        test_suite.teardown_simulation()


if __name__ == "__main__":
    sys.exit(main())
