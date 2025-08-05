#!/usr/bin/env python3
"""
Test script to verify the fix for fetch and place workflow with alternative routes
"""

import sys
import time
from sim.world import SimulationWorld
from sim.robot import HumanoidRobot
import pybullet as p

def test_fetch_place_alternative_route():
    """Test fetch and place with alternative route navigation"""
    print("🧪 Testing Fetch & Place Fix with Alternative Routes")
    print("=" * 60)
    
    # Initialize simulation
    p.connect(p.DIRECT)  # Use DIRECT mode to avoid GUI conflicts
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    
    # Create world and robot
    world = SimulationWorld()
    world.initialize()
    robot = HumanoidRobot()
    robot._world_ref = world
    
    # Create fetch and place scenario with platforms blocking direct path
    world.create_fetch_and_place_scenario()
    
    # Load robot
    if not robot.load_robot():
        print("❌ Failed to load robot")
        return False
    
    print("\n📋 Test Scenario:")
    print("1. Pick up box_01 from platform_1")
    print("2. Navigate to bin_1 using alternative route (platforms blocking direct path)")
    print("3. Verify object is still held during multi-waypoint navigation")
    print("4. Place object at bin_1")
    
    # Step 1: Position robot near first platform
    robot.set_position([1.0, 1.0, 0.5])
    print(f"\n🤖 Robot positioned at: {robot.get_position()}")
    
    # Step 2: Pick up box from platform
    print(f"\n🎯 Phase 1: Picking up box_01 from platform...")
    success_pick = robot.attempt_pick("box_01")
    
    if success_pick:
        print("✅ Successfully picked up box_01")
        print(f"   Robot is holding: {robot.get_held_objects()}")
    else:
        print("❌ Failed to pick up box_01")
        return False
    
    # Step 3: Navigate to bin using alternative route (this will trigger multi-waypoint navigation)
    print(f"\n🎯 Phase 2: Navigating to bin_1 (may require alternative route)...")
    bin_position = [3.0, 1.0, 0.5]  # Near bin_1
    
    # Start navigation - this should trigger safe path planning and multi-waypoint navigation
    robot.start_walking_to(bin_position)
    
    # Monitor navigation with frequent object checks
    navigation_steps = 0
    max_navigation_steps = 300  # 5 seconds at 60 FPS
    
    print(f"   🚶 Starting multi-waypoint navigation...")
    print(f"   🔍 Monitoring object attachment during navigation...")
    
    while robot.is_walking() and navigation_steps < max_navigation_steps:
        p.stepSimulation()
        time.sleep(1./60.)
        navigation_steps += 1
        
        # Check object status every 30 steps (every 0.5 seconds)
        if navigation_steps % 30 == 0:
            if robot.is_holding_any():
                print(f"   ✅ Step {navigation_steps}: Object still held: {robot.get_held_objects()}")
            else:
                print(f"   ❌ Step {navigation_steps}: OBJECT DROPPED! Navigation failed.")
                return False
    
    # Check final status after navigation
    if robot.is_holding_any():
        print(f"✅ Navigation complete! Object still held: {robot.get_held_objects()}")
    else:
        print(f"❌ CRITICAL: Object was dropped during navigation!")
        return False
    
    # Step 4: Place object at bin
    print(f"\n🎯 Phase 3: Placing object at bin_1...")
    bin_location = [3.0, 1.0, 0.1]
    
    success_place = robot.attempt_place(bin_location, "box_01")
    
    if success_place:
        print("✅ Successfully placed box_01 at bin_1")
        print(f"   Robot is holding: {robot.get_held_objects()}")
        return True
    else:
        print("❌ Failed to place box_01 at bin_1")
        return False

def main():
    """Main test function"""
    try:
        success = test_fetch_place_alternative_route()
        
        print(f"\n🏁 Test Result:")
        if success:
            print("✅ PASS - Fetch & Place with alternative route works correctly!")
            print("   Objects are properly held during multi-waypoint navigation.")
        else:
            print("❌ FAIL - Objects are still being dropped during navigation.")
            print("   Additional fixes may be needed.")
        
        # Keep simulation running briefly for final check
        print(f"\n⏳ Running final simulation steps...")
        for _ in range(60):
            p.stepSimulation()
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print(f"\n👋 Test completed.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()
