#!/usr/bin/env python3
"""
Placement verification tool - Check if objects are properly placed in bins
"""

import math
import sys
import argparse

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    print("PyBullet not available")
    sys.exit(1)

from sim.world import SimulationWorld


def check_placement_status(use_gui=False):
    """Check which objects are successfully placed in bins"""
    
    # Initialize simulation
    world = SimulationWorld(use_gui=use_gui)
    world.initialize()
    world.create_fetch_and_place_scenario()
    
    print("📊 PLACEMENT STATUS CHECKER:")
    print("=" * 50)
    
    # Define bin zones (with tolerance)
    bin_zones = {
        "bin_1": {"center": [3.0, 0.5, 0.1], "tolerance": 0.3},
        "bin_2": {"center": [3.0, 1.0, 0.1], "tolerance": 0.3},
        "bin_3": {"center": [3.0, 1.5, 0.1], "tolerance": 0.3}
    }
    
    pickup_objects = ["box_01", "box_02", "box_03"]
    
    # Check current positions
    positions = world.object_positions
    
    print("📦 OBJECT PLACEMENT STATUS:")
    print()
    
    for obj_id in pickup_objects:
        if obj_id in positions:
            obj_pos = positions[obj_id]
            color = "RED" if "01" in obj_id else "GREEN" if "02" in obj_id else "BLUE"
            
            # Check if object is in any bin
            placed_in_bin = None
            for bin_id, bin_info in bin_zones.items():
                bin_center = bin_info["center"]
                tolerance = bin_info["tolerance"]
                
                # Calculate distance (only X and Y, ignore Z)
                distance = math.sqrt(
                    (obj_pos[0] - bin_center[0])**2 + 
                    (obj_pos[1] - bin_center[1])**2
                )
                
                if distance <= tolerance:
                    placed_in_bin = bin_id
                    break
            
            # Display status
            if placed_in_bin:
                print(f"  ✅ {obj_id:8} ({color:5}): PLACED in {placed_in_bin}")
                print(f"     Position: [{obj_pos[0]:4.1f}, {obj_pos[1]:4.1f}, {obj_pos[2]:4.2f}]")
            else:
                print(f"  ❌ {obj_id:8} ({color:5}): NOT PLACED")
                print(f"     Position: [{obj_pos[0]:4.1f}, {obj_pos[1]:4.1f}, {obj_pos[2]:4.2f}]")
            print()
    
    print("🗃️  BIN LOCATIONS (for reference):")
    for bin_id, bin_info in bin_zones.items():
        center = bin_info["center"]
        tolerance = bin_info["tolerance"]
        print(f"  {bin_id:8}: Center [{center[0]:4.1f}, {center[1]:4.1f}, {center[2]:4.2f}] ± {tolerance}")
    
    print("\n💡 PLACEMENT CRITERIA:")
    print("  - Object must be within 0.3 units of bin center (X,Y)")
    print("  - Z-height doesn't matter (objects can be dropped anywhere)")
    print("  - Bins are large enough to catch objects dropped nearby")
    
    # Clean shutdown
    world.shutdown()


def distance_to_bin(obj_pos, bin_center):
    """Calculate 2D distance from object to bin center"""
    return math.sqrt(
        (obj_pos[0] - bin_center[0])**2 + 
        (obj_pos[1] - bin_center[1])**2
    )


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Check object placement status")
    parser.add_argument("--gui", action="store_true", help="Show GUI visualization")
    args = parser.parse_args()
    
    check_placement_status(use_gui=args.gui)


if __name__ == "__main__":
    main()
