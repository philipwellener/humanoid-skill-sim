#!/usr/bin/env python3
"""
Test script to verify the humanoid skill simulator installation
"""

import sys
import os

def test_imports():
    """Test all critical imports"""
    print("Testing imports...")
    
    try:
        import numpy as np
        print("✓ numpy imported successfully")
    except ImportError as e:
        print(f"✗ numpy import failed: {e}")
        return False
    
    try:
        import yaml
        print("✓ yaml imported successfully")
    except ImportError as e:
        print(f"✗ yaml import failed: {e}")
        return False
    
    try:
        import pybullet as p
        print("✓ pybullet imported successfully")
    except ImportError as e:
        print(f"✗ pybullet import failed: {e}")
        return False
    
    try:
        import matplotlib.pyplot as plt
        print("✓ matplotlib imported successfully")
    except ImportError as e:
        print(f"✗ matplotlib import failed: {e}")
        return False
    
    return True

def test_local_modules():
    """Test local module imports"""
    print("\nTesting local modules...")
    
    # Add current directory to path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, current_dir)
    
    try:
        from behavior_tree.node import Node, NodeStatus
        print("✓ behavior_tree.node imported successfully")
    except ImportError as e:
        print(f"✗ behavior_tree.node import failed: {e}")
        return False
    
    try:
        from behavior_tree.sequence import Sequence
        print("✓ behavior_tree.sequence imported successfully")
    except ImportError as e:
        print(f"✗ behavior_tree.sequence import failed: {e}")
        return False
    
    try:
        from sim.world import SimulationWorld
        print("✓ sim.world imported successfully")
    except ImportError as e:
        print(f"✗ sim.world import failed: {e}")
        return False
    
    try:
        from sim.robot import HumanoidRobot
        print("✓ sim.robot imported successfully")
    except ImportError as e:
        print(f"✗ sim.robot import failed: {e}")
        return False
    
    return True

def test_basic_functionality():
    """Test basic functionality"""
    print("\nTesting basic functionality...")
    
    try:
        from behavior_tree.node import Node, NodeStatus
        from behavior_tree.sequence import Sequence
        from behavior_tree.action_nodes import WalkToTarget
        
        # Create a simple behavior tree
        walk_node = WalkToTarget(target=[1.0, 0.0, 0.0])
        seq = Sequence([walk_node])
        
        print("✓ Basic behavior tree creation successful")
        return True
        
    except Exception as e:
        print(f"✗ Basic functionality test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("Humanoid Skill Simulator Installation Test")
    print("=" * 50)
    
    all_passed = True
    
    # Test external dependencies
    if not test_imports():
        all_passed = False
        print("\n❌ External dependency imports failed!")
        print("Try running: pip install pybullet numpy pyyaml matplotlib")
    
    # Test local modules
    if not test_local_modules():
        all_passed = False
        print("\n❌ Local module imports failed!")
    
    # Test basic functionality
    if not test_basic_functionality():
        all_passed = False
        print("\n❌ Basic functionality test failed!")
    
    print("\n" + "=" * 50)
    if all_passed:
        print("🎉 All tests passed! Installation is working correctly.")
        print("\nYou can now run:")
        print("  python run_sim.py --workflow simple_walk --no-gui")
    else:
        print("❌ Some tests failed. Please check the errors above.")
        
    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
