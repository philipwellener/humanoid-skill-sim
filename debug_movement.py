#!/usr/bin/env python3
"""
Debug script to test robot movement
"""

import pybullet as p
import time
from sim.robot import HumanoidRobot
from sim.world import SimulationWorld

def test_movement():
    # Initialize simulation
    p.connect(p.DIRECT)  # No GUI for faster testing
    p.setGravity(0, 0, -9.81)
    
    # Create world and robot
    world = SimulationWorld()
    world.setup_environment()
    
    robot = HumanoidRobot()
    success = robot.load_robot()
    if not success:
        print("❌ Failed to load robot")
        return
    
    print(f"🤖 Robot loaded at: {robot.get_position()}")
    
    # Test movement
    target = [1.0, 0.5, 0.5]
    print(f"🎯 Starting walk to: {target}")
    
    robot.start_walking_to(target)
    
    # Monitor movement
    for i in range(50):
        p.stepSimulation()
        
        is_walking = robot.is_walking()
        current_pos = robot.get_position()
        
        if i % 10 == 0:
            print(f"Step {i}: Position = {current_pos}, Walking = {is_walking}")
        
        if not is_walking:
            print(f"✅ Walking completed at step {i}")
            break
        
        time.sleep(0.01)
    
    final_pos = robot.get_position()
    print(f"🏁 Final position: {final_pos}")
    print(f"📏 Distance to target: {((final_pos[0] - target[0])**2 + (final_pos[1] - target[1])**2)**0.5:.3f}")
    
    p.disconnect()

if __name__ == "__main__":
    test_movement()
