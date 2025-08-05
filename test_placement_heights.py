#!/usr/bin/env python3
"""
Test placement heights for both platforms and bins to verify proper positioning
"""

import sys
import time
from sim.world import SimulationWorld
from sim.robot import HumanoidRobot
import pybullet as p

def test_placement_heights():
    """Test object placement at different heights"""
    print("🧪 Testing placement heights on platforms vs bins...")
    
    # Initialize simulation
    p.connect(p.GUI)  # Use GUI to visually inspect
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    
    # Create world and robot
    world = SimulationWorld()
    robot = HumanoidRobot()
    robot._world_ref = world
    
    # Create elevated scenario with platforms and bins
    world.create_elevated_scenario()
    
    # Add robot to simulation
    robot.load_robot()
    
    print("\n📋 Placement Height Test Plan:")
    print("1. Pick box_01 from platform (elevated)")
    print("2. Place in bin_1 (ground level)")
    print("3. Analyze placement accuracy")
    
    # Step 1: Position robot near first platform
    robot.set_position([1.0, 1.0, 0.5])
    
    # Step 2: Pick up box from platform
    print("\n🎯 Phase 1: Picking from platform...")
    success = robot.attempt_pick("box_01")
    if success:
        print("✅ Successfully picked up box_01 from platform")
    else:
        print("❌ Failed to pick up box_01")
        return False
    
    # Step 3: Move to bin and place
    print("\n🎯 Phase 2: Placing in bin...")
    robot.set_position([2.5, 1.0, 0.5])  # Near bin
    robot.turn_to_face([3.0, 1.0, 0.1])  # Face bin
    
    # Attempt placement at bin location
    bin_location = [3.0, 1.0, 0.1]  # Bin center position
    success = robot.attempt_place(bin_location, "box_01")
    
    if success:
        print("✅ Successfully placed box_01 in bin")
        
        # Get final object position
        time.sleep(1.0)  # Let physics settle
        for _ in range(60):
            p.stepSimulation()
        
        if "box_01" in world.objects:
            obj_id = world.objects["box_01"] 
            final_pos, _ = p.getBasePositionAndOrientation(obj_id)
            print(f"\n📊 PLACEMENT ANALYSIS:")
            print(f"   Target bin location: {bin_location}")
            print(f"   Actual final position: {list(final_pos)}")
            print(f"   Height difference: {final_pos[2] - bin_location[2]:.3f}m")
            print(f"   Distance from bin center: {((final_pos[0] - bin_location[0])**2 + (final_pos[1] - bin_location[1])**2)**0.5:.3f}m")
            
            # Check if placement is successful
            height_ok = abs(final_pos[2] - bin_location[2]) < 0.05  # Within 5cm
            distance_ok = ((final_pos[0] - bin_location[0])**2 + (final_pos[1] - bin_location[1])**2)**0.5 < 0.3  # Within 30cm
            
            if height_ok and distance_ok:
                print("   ✅ PLACEMENT SUCCESSFUL: Object properly placed in bin")
            else:
                if not height_ok:
                    print(f"   ❌ HEIGHT ISSUE: Object at {final_pos[2]:.3f}m, expected ~{bin_location[2]:.3f}m")
                if not distance_ok:
                    print(f"   ❌ DISTANCE ISSUE: Object too far from bin center")
        
    else:
        print("❌ Failed to place box_01 in bin")
        return False
    
    print("\n⏸️  Simulation running - close window to exit")
    print("   Inspect the placement visually in the GUI")
    
    # Keep simulation running for visual inspection
    try:
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
    except KeyboardInterrupt:
        pass
    
    p.disconnect()
    return True

if __name__ == "__main__":
    test_placement_heights()
