#!/usr/bin/env python3
"""
Installation verification and quick test script for humanoid skill simulator.
Run this to verify that everything is working correctly.
"""

import os
import sys
from pathlib import Path

def test_imports():
    """Test all critical imports"""
    print("🔍 Testing imports...")
    
    try:
        import pybullet as p
        print("✅ PyBullet imported successfully")
    except ImportError as e:
        print(f"❌ PyBullet import failed: {e}")
        return False
    
    try:
        import numpy as np
        print("✅ NumPy imported successfully")
    except ImportError as e:
        print(f"❌ NumPy import failed: {e}")
        return False
    
    try:
        import yaml
        print("✅ PyYAML imported successfully")
    except ImportError as e:
        print(f"❌ PyYAML import failed: {e}")
        return False
    
    try:
        import matplotlib
        print("✅ Matplotlib imported successfully")
    except ImportError as e:
        print(f"❌ Matplotlib import failed: {e}")
        return False
    
    return True

def test_local_modules():
    """Test local module imports"""
    print("\n🔍 Testing local modules...")
    
    try:
        from sim.world import SimulationWorld
        from sim.robot import HumanoidRobot
        from sim.teleop_override import TeleopOverride
        print("✅ Simulation modules imported successfully")
    except ImportError as e:
        print(f"❌ Simulation module import failed: {e}")
        return False
    
    try:
        from behavior_tree.node import Node, NodeStatus
        from behavior_tree.sequence import Sequence
        from behavior_tree.fallback import Fallback
        print("✅ Behavior tree modules imported successfully")
    except ImportError as e:
        print(f"❌ Behavior tree module import failed: {e}")
        return False
    
    try:
        from skills.walk import WalkToTargetSkill
        from skills.pick import PickObjectSkill
        from skills.place import PlaceObjectSkill
        print("✅ Skills modules imported successfully")
    except ImportError as e:
        print(f"❌ Skills module import failed: {e}")
        return False
    
    return True

def test_config_files():
    """Test configuration files"""
    print("\n🔍 Testing configuration files...")
    
    config_path = Path("configs/workflows.yaml")
    if config_path.exists():
        try:
            import yaml
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            print(f"✅ Found {len(config)} workflows in configuration")
            
            # List available workflows
            print("   Available workflows:")
            for workflow_name in config.keys():
                if isinstance(config[workflow_name], dict):
                    desc = config[workflow_name].get('description', 'No description')
                    print(f"     - {workflow_name}: {desc}")
        except Exception as e:
            print(f"❌ Failed to load workflow configuration: {e}")
            return False
    else:
        print("❌ Workflow configuration file not found")
        return False
    
    return True

def run_quick_test():
    """Run a quick simulation test"""
    print("\n🔍 Running quick simulation test...")
    
    try:
        # Import run_sim module
        import run_sim
        
        # Create a minimal simulation test
        print("✅ Main simulation module imported successfully")
        
        # Test workflow loading
        sim_runner = run_sim.SimulationRunner(use_gui=False)
        workflows = sim_runner.load_workflow_config("configs/workflows.yaml")
        
        if 'simple_walk' in workflows:
            print("✅ simple_walk workflow found and can be loaded")
        else:
            print("⚠️  simple_walk workflow not found")
        
        return True
        
    except Exception as e:
        print(f"❌ Quick simulation test failed: {e}")
        return False

def main():
    """Main test function"""
    print("🤖 Humanoid Skill Simulator - Installation Verification")
    print("=" * 60)
    
    tests_passed = 0
    total_tests = 4
    
    # Test 1: External imports
    if test_imports():
        tests_passed += 1
    
    # Test 2: Local modules
    if test_local_modules():
        tests_passed += 1
    
    # Test 3: Configuration files
    if test_config_files():
        tests_passed += 1
    
    # Test 4: Quick simulation test
    if run_quick_test():
        tests_passed += 1
    
    print("\n" + "=" * 60)
    print(f"RESULTS: {tests_passed}/{total_tests} tests passed")
    
    if tests_passed == total_tests:
        print("🎉 All tests passed! Installation is working correctly.")
        print("\nYou can now run simulations with:")
        print("  python run_sim.py --workflow simple_walk --no-gui")
        print("  python run_sim.py --workflow fetch_and_place")
        return True
    else:
        print("❌ Some tests failed. Please check the error messages above.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
