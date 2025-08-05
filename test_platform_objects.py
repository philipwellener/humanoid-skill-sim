#!/usr/bin/env python3
"""
Test script to demonstrate elevated objects on platforms for better gripper interaction
"""

import time
import pybullet as p
import pybullet_data
from sim.world import SimulationWorld
from sim.robot import HumanoidRobot

def test_platform_objects():
    """Test picking objects from platforms at gripper height"""
    
    print("🏛️ Testing Platform-Elevated Objects")
    print("=" * 50)
    
    # Create simulation world and initialize with ground plane
    world = SimulationWorld(use_gui=True)
    world.initialize()  # This sets up the ground plane and basic environment
    
    # Load robot
    robot = HumanoidRobot()
    success = robot.load_robot()
    
    if not success:
        print("❌ Failed to load robot")
        return
    
    # Give robot reference to world for object manipulation
    robot._world_ref = world
    
    # Connect robot to world for picking operations
    robot.world = world
    
    # Create platforms at different locations
    print("\n📦 Creating Platforms...")
    
    # Platform 1: Near robot starting position
    world.add_platform("platform1", [2.0, 1.0, 0.0], color=[0.6, 0.4, 0.2, 1.0])
    
    # Platform 2: Further away  
    world.add_platform("platform2", [3.0, 2.0, 0.0], color=[0.4, 0.6, 0.2, 1.0])
    
    # Add objects on platforms at gripper height
    print("\n📦 Placing Objects on Platforms...")
    
    world.add_object_on_platform("box1", "platform1", "cube_small.urdf")
    world.add_object_on_platform("box2", "platform2", "cube_small.urdf")
    
    # Also create a ground-level object for comparison
    print("\n📦 Adding Ground-Level Object...")
    world.add_object("ground_box", "cube_small.urdf", [1.5, 1.5, 0.1])
    
    # Add target bin
    world.add_simple_box("bin", [4.0, 1.5, 0.1], size=[0.15, 0.15, 0.1], color=[0.2, 0.2, 0.8, 1.0])
    
    # Let simulation settle
    print("\n⏱️ Letting simulation settle...")
    for _ in range(30):
        world.step_simulation()
    time.sleep(1.0)
    
    # Test sequence: pick elevated objects
    print("\n🤖 Testing Robot with Elevated Objects")
    print("-" * 40)
    
    # Test 1: Pick from platform 1 (elevated)
    print("\n1️⃣ Testing Platform Object (Elevated)")
    success1 = robot.attempt_pick("box1")
    if success1:
        print("  ✅ Successfully picked elevated object!")
        time.sleep(2.0)
        
        # Place in bin
        success_place1 = robot.attempt_place("bin")
        if success_place1:
            print("  ✅ Successfully placed elevated object in bin!")
        else:
            print("  ❌ Failed to place elevated object")
    else:
        print("  ❌ Failed to pick elevated object")
    
    time.sleep(1.0)
    
    # Test 2: Pick ground object for comparison
    print("\n2️⃣ Testing Ground Object (Standard)")
    success2 = robot.attempt_pick("ground_box")
    if success2:
        print("  ✅ Successfully picked ground object!")
        time.sleep(2.0)
        
        # Place in bin
        success_place2 = robot.attempt_place("bin")
        if success_place2:
            print("  ✅ Successfully placed ground object in bin!")
        else:
            print("  ❌ Failed to place ground object")
    else:
        print("  ❌ Failed to pick ground object")
    
    time.sleep(1.0)
    
    # Test 3: Pick from platform 2 (elevated)
    print("\n3️⃣ Testing Second Platform Object (Elevated)")
    success3 = robot.attempt_pick("box2")
    if success3:
        print("  ✅ Successfully picked second elevated object!")
        time.sleep(2.0)
        
        # Place in bin
        success_place3 = robot.attempt_place("bin")
        if success_place3:
            print("  ✅ Successfully placed second elevated object in bin!")
        else:
            print("  ❌ Failed to place second elevated object")
    else:
        print("  ❌ Failed to pick second elevated object")
    
    # Summary
    print("\n📊 Test Results Summary")
    print("=" * 50)
    print(f"Platform Object 1 (Elevated): {'✅ PASS' if success1 else '❌ FAIL'}")
    print(f"Ground Object (Standard):     {'✅ PASS' if success2 else '❌ FAIL'}")
    print(f"Platform Object 2 (Elevated): {'✅ PASS' if success3 else '❌ FAIL'}")
    
    elevated_success_rate = sum([success1, success3]) / 2 * 100
    print(f"\n🏛️ Elevated Objects Success Rate: {elevated_success_rate:.0f}%")
    print(f"📦 Ground Objects Success Rate: {100 if success2 else 0:.0f}%")
    
    if elevated_success_rate > 50:
        print("\n🎉 Platform solution is working! Objects at gripper height are easier to pick.")
    else:
        print("\n🔧 Platform solution needs refinement.")
    
    # Keep simulation running for observation
    print("\n⏳ Keeping simulation open for observation...")
    print("   Press Ctrl+C to exit")
    
    try:
        while True:
            world.step_simulation()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n👋 Simulation ended.")
    
    # Cleanup
    p.disconnect()

if __name__ == "__main__":
    test_platform_objects()
