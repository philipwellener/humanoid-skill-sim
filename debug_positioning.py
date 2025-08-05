#!/usr/bin/env python3

import numpy as np

def analyze_positioning():
    """Analyze the robot positioning logic"""
    
    # Simulate typical scenario
    robot_pos = [0.0, 0.0, 0.5]  # Robot starts at origin
    obj_pos = [1.0, 0.5, 0.05]   # Object position
    
    print("=== POSITIONING ANALYSIS ===")
    print(f"Robot starts at: {robot_pos}")
    print(f"Object is at: {obj_pos}")
    print()
    
    # Current logic from action_nodes.py:
    print("1. Calculate obj_to_robot vector:")
    obj_to_robot = np.array([robot_pos[0], robot_pos[1]]) - np.array([obj_pos[0], obj_pos[1]])
    obj_to_robot_distance = np.linalg.norm(obj_to_robot)
    
    print(f"   obj_to_robot vector: {obj_to_robot}")
    print(f"   obj_to_robot distance: {obj_to_robot_distance:.3f}")
    print()
    
    print("2. Normalize approach direction:")
    if obj_to_robot_distance > 0.01:
        approach_direction = obj_to_robot / obj_to_robot_distance
    else:
        approach_direction = np.array([-1.0, 0.0])
    
    print(f"   Normalized approach direction: {approach_direction}")
    print()
    
    print("3. Calculate target position:")
    target_distance = 0.45
    target_pos = [
        obj_pos[0] + approach_direction[0] * target_distance,
        obj_pos[1] + approach_direction[1] * target_distance,
        robot_pos[2]
    ]
    
    print(f"   Target distance: {target_distance}")
    print(f"   Calculated target position: {target_pos}")
    
    # Calculate what the distance would be
    final_distance = np.linalg.norm(np.array(target_pos[:2]) - np.array(obj_pos[:2]))
    print(f"   Distance from target position to object: {final_distance:.3f}")
    print()
    
    print("4. ROBOT MOVEMENT ISSUE:")
    print("   The robot has a 0.1 multiplier in movement, so it only moves 10% of intended distance")
    
    # Simulate the 0.1 movement multiplier
    movement_progress = 1.0  # Full intended movement
    actual_movement_multiplier = 0.1  # The problematic 0.1 factor
    
    intermediate_pos = [
        robot_pos[0] + (target_pos[0] - robot_pos[0]) * movement_progress * actual_movement_multiplier,
        robot_pos[1] + (target_pos[1] - robot_pos[1]) * movement_progress * actual_movement_multiplier,
        robot_pos[2]
    ]
    
    print(f"   Robot actually moves to: {intermediate_pos}")
    
    actual_distance = np.linalg.norm(np.array(intermediate_pos[:2]) - np.array(obj_pos[:2]))
    print(f"   Actual distance to object: {actual_distance:.3f}")
    print()
    
    print("5. ANALYSIS CONCLUSION:")
    print(f"   - Intended final distance: {final_distance:.3f}m (GOOD)")
    print(f"   - Actual distance due to 0.1 factor: {actual_distance:.3f}m (TOO CLOSE!)")
    print(f"   - Robot should be {final_distance:.3f}m away but ends up {actual_distance:.3f}m away")
    print()
    
    print("6. SOLUTION:")
    print("   Remove the 0.1 multiplier from robot walking logic!")

if __name__ == "__main__":
    analyze_positioning()
