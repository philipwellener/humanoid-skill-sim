#!/usr/bin/env python3
"""
Object finder tool - Shows locations of all objects in the simulation
"""

import sys
import argparse

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    print("PyBullet not available")
    sys.exit(1)

from sim.world import SimulationWorld


def find_objects(use_gui=False):
    """Find and display all object locations"""
    
    # Initialize simulation
    world = SimulationWorld(use_gui=use_gui)
    world.initialize()
    
    # Create the standard scenario
    world.create_fetch_and_place_scenario()
    
    print("🔍 OBJECT LOCATIONS:")
    print("=" * 50)
    
    # Display all objects
    objects = world.objects
    positions = world.object_positions
    
    print("📦 PICKUP OBJECTS:")
    for obj_id in ["box_01", "box_02", "box_03"]:
        if obj_id in positions:
            pos = positions[obj_id]
            color = "RED" if "01" in obj_id else "GREEN" if "02" in obj_id else "BLUE"
            print(f"  {obj_id:8} ({color:5}): [{pos[0]:4.1f}, {pos[1]:4.1f}, {pos[2]:4.2f}]")
    
    print("\n🗃️  TARGET BINS:")
    for obj_id in ["bin_1", "bin_2", "bin_3"]:
        if obj_id in positions:
            pos = positions[obj_id]
            print(f"  {obj_id:8} (GRAY ): [{pos[0]:4.1f}, {pos[1]:4.1f}, {pos[2]:4.2f}]")
    
    print("\n🚧 OBSTACLES:")
    for obj_id in positions:
        if "obstacle" in obj_id:
            pos = positions[obj_id]
            print(f"  {obj_id:8} (BROWN): [{pos[0]:4.1f}, {pos[1]:4.1f}, {pos[2]:4.2f}]")
    
    print("\n🤖 ROBOT STARTING POSITION:")
    print("  robot    (B&W  ): [ 0.0,  0.0,  0.50]")
    
    print("\n💡 USAGE EXAMPLES:")
    print("  # Walk to box_01:")
    print("  w 1.5 0.8")
    print("  # Pick up box_01:")
    print("  p box_01")
    print("  # Walk to bin_1:")
    print("  w 3.0 0.5")
    print("  # Place object:")
    print("  d 3.0 0.5 0.1")
    
    # Clean shutdown
    world.shutdown()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Find object locations in simulation")
    parser.add_argument("--gui", action="store_true", help="Show GUI visualization")
    args = parser.parse_args()
    
    find_objects(use_gui=args.gui)


if __name__ == "__main__":
    main()
