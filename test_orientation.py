#!/usr/bin/env python3
"""
Test to understand robot orientation and positioning
"""

import pybullet as p
import pybullet_data
import numpy as np
import time

def test_robot_orientation():
    print("🔧 Testing robot orientation understanding...")
    
    # Initialize physics
    p.connect(p.GUI)  # Use GUI to see the robot
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Load ground
    plane = p.loadURDF("plane.urdf")
    
    # Load robot
    robot_id = p.loadURDF("r2d2.urdf", [0, 0, 0.5], [0, 0, 0, 1])
    
    # Load a test object 
    test_object = p.loadURDF("cube_small.urdf", [1.0, 0.5, 0.1], globalScaling=0.5)
    p.changeVisualShape(test_object, -1, rgbaColor=[1, 0, 0, 1])  # Red cube
    
    print(f"📍 Robot loaded at [0, 0, 0.5]")
    print(f"📍 Test object at [1.0, 0.5, 0.1]")
    
    # Test different orientations
    orientations = [
        (0, "Facing +X (right)"),
        (np.pi/2, "Facing +Y (up)"), 
        (np.pi, "Facing -X (left)"),
        (-np.pi/2, "Facing -Y (down)")
    ]
    
    for angle, description in orientations:
        print(f"\n🔄 Setting robot orientation: {description} (angle: {np.degrees(angle):.1f}°)")
        
        # Set robot orientation
        quaternion = p.getQuaternionFromEuler([0, 0, angle])
        p.resetBasePositionAndOrientation(robot_id, [0, 0, 0.5], quaternion)
        
        # Step simulation to update
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.1)
        
        input("Press Enter to continue to next orientation...")
    
    # Now test approach positioning
    print("\n🎯 Testing approach positioning...")
    
    # Object position
    obj_pos = [1.0, 0.5, 0.1]
    
    # Test approach positions
    approach_distance = 0.45
    
    # Approach from different directions
    approaches = [
        (-approach_distance, 0, "From LEFT (robot faces right toward object)"),
        (approach_distance, 0, "From RIGHT (robot faces left toward object)"),
        (0, -approach_distance, "From BELOW (robot faces up toward object)"),
        (0, approach_distance, "From ABOVE (robot faces down toward object)")
    ]
    
    for dx, dy, description in approaches:
        robot_pos = [obj_pos[0] + dx, obj_pos[1] + dy, 0.5]
        print(f"\n🤖 {description}")
        print(f"   Robot at: {robot_pos[:2]}")
        print(f"   Object at: {obj_pos[:2]}")
        
        # Position robot
        p.resetBasePositionAndOrientation(robot_id, robot_pos, [0, 0, 0, 1])
        
        # Calculate angle to face object
        target_dx = obj_pos[0] - robot_pos[0]
        target_dy = obj_pos[1] - robot_pos[1]
        target_angle = np.arctan2(target_dy, target_dx)
        
        print(f"   Required angle to face object: {np.degrees(target_angle):.1f}°")
        
        # Turn robot to face object
        quaternion = p.getQuaternionFromEuler([0, 0, target_angle])
        p.resetBasePositionAndOrientation(robot_id, robot_pos, quaternion)
        
        # Step simulation
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.05)
            
        input("Press Enter for next approach...")
    
    p.disconnect()
    print("✅ Test complete!")

if __name__ == "__main__":
    test_robot_orientation()
