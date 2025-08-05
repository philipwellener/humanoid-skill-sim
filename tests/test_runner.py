"""
Comprehensive test runner for the humanoid skill simulator
"""

import unittest
import sys
import os
import time
from datetime import datetime
from typing import Dict, List, Any

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import all test modules
from tests.skill_tests import TestWalkToTargetSkill, TestPickObjectSkill, TestPlaceObjectSkill, TestSkillMetrics
from tests.behavior_tree_tests import TestBehaviorTreeNodes, TestBehaviorTreeExecution, TestActionNodes
from tests.simulation_tests import TestSimulationWorld, TestHumanoidRobot, TestTeleopOverride


class ComprehensiveTestSuite:
    """Main test suite runner for all simulator components"""
    
    def __init__(self):
        self.results = {
            'start_time': None,
            'end_time': None,
            'total_tests': 0,
            'passed_tests': 0,
            'failed_tests': 0,
            'error_tests': 0,
            'skipped_tests': 0,
            'suite_results': {}
        }
    
    def run_test_suite(self, suite_name: str, test_classes: List) -> Dict[str, Any]:
        """Run a specific test suite"""
        print(f"\n{'='*60}")
        print(f"Running {suite_name}")
        print(f"{'='*60}")
        
        suite = unittest.TestSuite()
        for test_class in test_classes:
            suite.addTests(unittest.TestLoader().loadTestsFromTestCase(test_class))
        
        # Run tests with detailed output
        runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
        result = runner.run(suite)
        
        suite_result = {
            'tests_run': result.testsRun,
            'failures': len(result.failures),
            'errors': len(result.errors),
            'skipped': len(result.skipped) if hasattr(result, 'skipped') else 0,
            'success_rate': ((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100) if result.testsRun > 0 else 0
        }
        
        return suite_result
    
    def run_scenario_tests(self) -> Dict[str, Any]:
        """Run scenario-based tests"""
        print(f"\n{'='*60}")
        print("Running Scenario Tests")
        print(f"{'='*60}")
        
        try:
            # Import and run scenario tests
            import subprocess
            result = subprocess.run([
                sys.executable, 'tests/scenario_tests.py'
            ], capture_output=True, text=True, cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            
            # Parse output for success rate
            output = result.stdout
            if 'Success Rate:' in output:
                success_line = [line for line in output.split('\n') if 'Success Rate:' in line][0]
                success_rate = float(success_line.split(':')[1].strip().replace('%', ''))
                total_tests = int([line for line in output.split('\n') if 'Total Tests:' in line][0].split(':')[1].strip())
                passed = int(total_tests * success_rate / 100)
                
                return {
                    'tests_run': total_tests,
                    'failures': total_tests - passed,
                    'errors': 0,
                    'skipped': 0,
                    'success_rate': success_rate
                }
            else:
                return {'tests_run': 0, 'failures': 1, 'errors': 0, 'skipped': 0, 'success_rate': 0}
                
        except Exception as e:
            print(f"Error running scenario tests: {e}")
            return {'tests_run': 0, 'failures': 0, 'errors': 1, 'skipped': 0, 'success_rate': 0}
    
    def run_regression_tests(self) -> Dict[str, Any]:
        """Run regression tests"""
        print(f"\n{'='*60}")
        print("Running Regression Tests")
        print(f"{'='*60}")
        
        try:
            import subprocess
            result = subprocess.run([
                sys.executable, 'tests/regression_tests.py'
            ], capture_output=True, text=True, cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            
            output = result.stdout
            if 'Replay Success Rate:' in output:
                success_line = [line for line in output.split('\n') if 'Replay Success Rate:' in line][0]
                success_rate = float(success_line.split(':')[1].strip().replace('%', ''))
                total_line = [line for line in output.split('\n') if 'Total Scenarios Tested:' in line][0]
                total_tests = int(total_line.split(':')[1].strip())
                passed = int(total_tests * success_rate / 100)
                
                return {
                    'tests_run': total_tests,
                    'failures': total_tests - passed,
                    'errors': 0,
                    'skipped': 0,
                    'success_rate': success_rate
                }
            else:
                return {'tests_run': 0, 'failures': 1, 'errors': 0, 'skipped': 0, 'success_rate': 0}
                
        except Exception as e:
            print(f"Error running regression tests: {e}")
            return {'tests_run': 0, 'failures': 0, 'errors': 1, 'skipped': 0, 'success_rate': 0}
    
    def run_all_tests(self) -> Dict[str, Any]:
        """Run all test suites"""
        self.results['start_time'] = datetime.now()
        print(f"\n🤖 HUMANOID SKILL SIMULATOR - COMPREHENSIVE TEST SUITE")
        print(f"Started: {self.results['start_time'].strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"{'='*80}")
        
        # Define test suites
        test_suites = {
            'Skill Tests': [TestWalkToTargetSkill, TestPickObjectSkill, TestPlaceObjectSkill, TestSkillMetrics],
            'Behavior Tree Tests': [TestBehaviorTreeNodes, TestBehaviorTreeExecution, TestActionNodes],
            'Simulation Tests': [TestSimulationWorld, TestHumanoidRobot, TestTeleopOverride]
        }
        
        # Run unit test suites
        for suite_name, test_classes in test_suites.items():
            try:
                suite_result = self.run_test_suite(suite_name, test_classes)
                self.results['suite_results'][suite_name] = suite_result
                
                self.results['total_tests'] += suite_result['tests_run']
                self.results['failed_tests'] += suite_result['failures']
                self.results['error_tests'] += suite_result['errors']
                self.results['skipped_tests'] += suite_result['skipped']
                
            except Exception as e:
                print(f"Error running {suite_name}: {e}")
                self.results['suite_results'][suite_name] = {
                    'tests_run': 0, 'failures': 0, 'errors': 1, 'skipped': 0, 'success_rate': 0
                }
                self.results['error_tests'] += 1
        
        # Run scenario tests
        scenario_result = self.run_scenario_tests()
        self.results['suite_results']['Scenario Tests'] = scenario_result
        self.results['total_tests'] += scenario_result['tests_run']
        self.results['failed_tests'] += scenario_result['failures']
        self.results['error_tests'] += scenario_result['errors']
        
        # Run regression tests
        regression_result = self.run_regression_tests()
        self.results['suite_results']['Regression Tests'] = regression_result
        self.results['total_tests'] += regression_result['tests_run']
        self.results['failed_tests'] += regression_result['failures']
        self.results['error_tests'] += regression_result['errors']
        
        # Calculate passed tests
        self.results['passed_tests'] = (
            self.results['total_tests'] - 
            self.results['failed_tests'] - 
            self.results['error_tests'] - 
            self.results['skipped_tests']
        )
        
        self.results['end_time'] = datetime.now()
        self.print_summary()
        
        return self.results
    
    def print_summary(self):
        """Print comprehensive test summary"""
        duration = (self.results['end_time'] - self.results['start_time']).total_seconds()
        
        print(f"\n{'='*80}")
        print(f"🎯 COMPREHENSIVE TEST RESULTS SUMMARY")
        print(f"{'='*80}")
        print(f"Total Duration: {duration:.2f} seconds")
        print(f"Total Tests: {self.results['total_tests']}")
        print(f"✅ Passed: {self.results['passed_tests']}")
        print(f"❌ Failed: {self.results['failed_tests']}")
        print(f"🚨 Errors: {self.results['error_tests']}")
        print(f"⏭️  Skipped: {self.results['skipped_tests']}")
        
        if self.results['total_tests'] > 0:
            success_rate = (self.results['passed_tests'] / self.results['total_tests']) * 100
            print(f"📊 Overall Success Rate: {success_rate:.1f}%")
        else:
            success_rate = 0
            print(f"📊 Overall Success Rate: 0.0%")
        
        print(f"\n{'='*60}")
        print("📋 BREAKDOWN BY TEST SUITE")
        print(f"{'='*60}")
        
        for suite_name, suite_result in self.results['suite_results'].items():
            print(f"{suite_name}:")
            print(f"  Tests: {suite_result['tests_run']}")
            print(f"  Success Rate: {suite_result['success_rate']:.1f}%")
            if suite_result['failures'] > 0:
                print(f"  ❌ Failures: {suite_result['failures']}")
            if suite_result['errors'] > 0:
                print(f"  🚨 Errors: {suite_result['errors']}")
            print()
        
        # Recommendations
        print(f"{'='*60}")
        print("💡 RECOMMENDATIONS")
        print(f"{'='*60}")
        
        if success_rate >= 90:
            print("🎉 Excellent! Test suite is in great shape.")
        elif success_rate >= 75:
            print("✅ Good test coverage, but some issues need attention.")
        elif success_rate >= 50:
            print("⚠️  Moderate test success. Several issues need fixing.")
        else:
            print("🚨 Critical issues detected. Immediate attention required.")
        
        if self.results['failed_tests'] > 0:
            print(f"- Address {self.results['failed_tests']} test failures")
        if self.results['error_tests'] > 0:
            print(f"- Fix {self.results['error_tests']} test errors")
        
        print(f"\n{'='*80}")


def main():
    """Main test runner"""
    suite = ComprehensiveTestSuite()
    results = suite.run_all_tests()
    
    # Exit with appropriate code
    if results['failed_tests'] == 0 and results['error_tests'] == 0:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
