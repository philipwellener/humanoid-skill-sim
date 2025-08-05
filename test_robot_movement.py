#!/usr/bin/env python3
"""
Simple test to debug robot movement issue
"""

import sys
import os
import time

# Add the project root to Python path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from sim.robot import HumanoidRobot
from sim.world import SimulationWorld
import pybullet as p

def test_robot_movement():
    print("🔧 Testing robot movement...")
    
    # Initialize physics
    p.connect(p.DIRECT)  # No GUI
    p.setGravity(0, 0, -9.81)
    
    # Set additional search path for URDF files
    import pybullet_data
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Load robot
    robot = HumanoidRobot()
    success = robot.load_robot()
    
    if not success:
        print("❌ Failed to load robot")
        return
    
    print(f"✅ Robot loaded")
    print(f"📍 Initial position: {robot.get_position()}")
    
    # Test walking
    target = [1.0, 0.5, 0.5]
    print(f"🎯 Starting walk to: {target}")
    
    robot.start_walking_to(target)
    print(f"🚶 Walking started, walking status: {robot.walking}")
    
    # Monitor movement for several steps
    for step in range(50):
        # Step simulation
        p.stepSimulation()
        
        # Check robot status
        is_walking = robot.is_walking()
        current_pos = robot.get_position()
        
        if step % 10 == 0 or step < 5:
            print(f"Step {step:2d}: pos={current_pos} walking={is_walking}")
        
        if not is_walking:
            print(f"🛑 Walking stopped at step {step}")
            break
        
        time.sleep(0.01)
    
    final_pos = robot.get_position()
    print(f"🏁 Final position: {final_pos}")
    
    distance_moved = ((final_pos[0] - 0)**2 + (final_pos[1] - 0)**2)**0.5
    distance_to_target = ((final_pos[0] - target[0])**2 + (final_pos[1] - target[1])**2)**0.5
    
    print(f"📏 Distance moved: {distance_moved:.3f}m")
    print(f"📏 Distance to target: {distance_to_target:.3f}m")
    
    if distance_moved > 0.1:
        print("✅ Robot movement is working!")
    else:
        print("❌ Robot is not moving!")
    
    p.disconnect()

if __name__ == "__main__":
    test_robot_movement()
