#!/usr/bin/env python3
"""Interactive mode for manual robot control with consolidated commands and placement verification"""

import signal
import sys
import time
from typing import List, Dict, Any, Optional

try:
    import pybullet as p
except ImportError:
    print("PyBullet not available")
    sys.exit(1)

from sim.world import SimulationWorld
from sim.robot import HumanoidRobot


class InteractiveController:
    def __init__(self, headless: bool = False):
        """Initialize interactive controller with optional headless mode"""
        self.headless = headless
        self.world = None
        self.robot = None
        self.running = True
        
        # Set up signal handler for graceful exit
        signal.signal(signal.SIGINT, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        print("\n🛑 Interrupted by user")
        self.running = False
        try:
            p.disconnect()
        except:
            pass
        sys.exit(0)
        
    def start_simulation(self):
        """Start simulation in lightweight mode"""
        try:
            print("🔌 Connecting to PyBullet physics...")
            print("🌍 Creating simulation world...")
            self.world = SimulationWorld(use_gui=not self.headless)
            self.world.initialize()
            
            # Configure GUI if enabled
            if not self.headless:
                # Enable minimal GUI
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
                p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            
            print("🤖 Creating robot...")
            self.robot = HumanoidRobot()
            self.robot.load_robot()
            
            # Give robot reference to world for object manipulation
            self.robot._world_ref = self.world
            
            print("📦 Setting up scenario...")
            self.world.create_fetch_and_place_scenario()
            
            print("✅ Simulation ready!")
            print("\nType 'help' for commands, 'l' for quick reference")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to start simulation: {e}")
            return False
    
    def show_locations(self) -> None:
        """Show all object locations in consolidated format"""
        robot_pos = self.robot.get_position()
        
        # Get current object positions
        current_positions = {}
        if self._update_object_positions():
            current_positions = getattr(self, '_position_update_results', {})
        
        print("\n📋 QUICK REFERENCE:")
        print("=" * 35)
        print("📦 BOXES          🗃️  BINS          🤖 ROBOT")
        
        # Show current box positions
        box_01_pos = current_positions.get('box_01', [1.5, 0.8])[:2]
        box_02_pos = current_positions.get('box_02', [2.0, 1.2])[:2]
        box_03_pos = current_positions.get('box_03', [1.2, 1.5])[:2]
        
        print("box_01: [{:.1f},{:.1f}]  bin_1: [3.0,0.5]  pos: [{:.1f},{:.1f}]".format(
            box_01_pos[0], box_01_pos[1], robot_pos[0], robot_pos[1]))
        print("box_02: [{:.1f},{:.1f}]  bin_2: [3.0,1.0]  held: {}".format(
            box_02_pos[0], box_02_pos[1], self.robot.get_held_objects() or "None"))
        print("box_03: [{:.1f},{:.1f}]  bin_3: [3.0,1.5]".format(
            box_03_pos[0], box_03_pos[1]))
        
        print("\n💡 QUICK COMMANDS:")
        print("Pick box_01: w {:.1f} {:.1f} → p box_01 → w 3.0 0.5 → d 3.0 0.5 0.5".format(
            box_01_pos[0], box_01_pos[1]))
        print("Pick box_02: w {:.1f} {:.1f} → p box_02 → w 3.0 1.0 → d 3.0 1.0 0.5".format(
            box_02_pos[0], box_02_pos[1]))
        print("Pick box_03: w {:.1f} {:.1f} → p box_03 → w 3.0 1.5 → d 3.0 1.5 0.5".format(
            box_03_pos[0], box_03_pos[1]))

    def _update_object_positions(self) -> bool:
        """Update object positions from physics simulation"""
        try:
            # Step the physics simulation to ensure updated positions
            for _ in range(5):
                p.stepSimulation()
                time.sleep(0.01)
            
            updated_positions = {}
            
            # Get all objects in the world
            for obj_name, obj_id in self.world.objects.items():
                try:
                    pos, _ = p.getBasePositionAndOrientation(obj_id)
                    updated_positions[obj_name] = pos
                except Exception as e:
                    print(f"⚠️  Failed to get position for {obj_name}: {e}")
                    continue
            
            # Store results
            self._position_update_results = updated_positions
            return True
            
        except Exception as e:
            print(f"❌ Error updating positions: {e}")
            return False

    def check_placement(self, box_name: str, bin_name: str) -> bool:
        """Check if a box has been successfully placed in a bin"""
        print(f"🔄 Checking placement of {box_name} in {bin_name}...")
        
        # Update positions from physics simulation
        if not self._update_object_positions():
            print("⚠️  Could not update object positions")
            return False
        
        # Get updated positions
        updated_positions = getattr(self, '_position_update_results', {})
        
        if box_name not in updated_positions:
            print(f"❌ Could not find {box_name}")
            return False
            
        box_pos = updated_positions[box_name]
        
        # Define bin regions (approximate)
        bin_regions = {
            'bin_1': {'center': [3.0, 0.5], 'tolerance': 0.3},
            'bin_2': {'center': [3.0, 1.0], 'tolerance': 0.3}, 
            'bin_3': {'center': [3.0, 1.5], 'tolerance': 0.3}
        }
        
        if bin_name not in bin_regions:
            print(f"❌ Unknown bin: {bin_name}")
            return False
            
        bin_region = bin_regions[bin_name]
        bin_center = bin_region['center']
        tolerance = bin_region['tolerance']
        
        # Check if box is within bin region
        dx = abs(box_pos[0] - bin_center[0])
        dy = abs(box_pos[1] - bin_center[1])
        
        is_placed = dx <= tolerance and dy <= tolerance
        
        print(f"📍 {box_name} position: [{box_pos[0]:.2f}, {box_pos[1]:.2f}]")
        print(f"🎯 {bin_name} center: [{bin_center[0]:.2f}, {bin_center[1]:.2f}] (±{tolerance:.1f})")
        print(f"📏 Distance: dx={dx:.2f}, dy={dy:.2f}")
        
        if is_placed:
            print(f"✅ SUCCESS! {box_name} is placed in {bin_name}")
        else:
            print(f"❌ {box_name} is NOT in {bin_name}")
            
        return is_placed

    def show_all_status(self) -> None:
        """Show consolidated status of all boxes and bins"""
        print("\n📊 STATUS REPORT")
        print("=" * 25)
        
        # Update positions from physics simulation
        if not self._update_object_positions():
            print("⚠️  Could not update object positions")
            return
        
        # Get updated positions
        updated_positions = getattr(self, '_position_update_results', {})
        
        # Define bin regions
        bin_regions = {
            'bin_1': {'center': [3.0, 0.5], 'tolerance': 0.3},
            'bin_2': {'center': [3.0, 1.0], 'tolerance': 0.3}, 
            'bin_3': {'center': [3.0, 1.5], 'tolerance': 0.3}
        }
        
        boxes = ['box_01', 'box_02', 'box_03']
        
        # Show consolidated status
        print("📦 BOX STATUS:")
        for box in boxes:
            if box not in updated_positions:
                print(f"   {box}: ❌ Not found")
                continue
                
            box_pos = updated_positions[box]
            
            # Check which bin (if any)
            placed_in_bin = None
            for bin_name, bin_info in bin_regions.items():
                bin_center = bin_info['center']
                tolerance = bin_info['tolerance']
                
                dx = abs(box_pos[0] - bin_center[0])
                dy = abs(box_pos[1] - bin_center[1])
                
                if dx <= tolerance and dy <= tolerance:
                    placed_in_bin = bin_name
                    break
            
            if placed_in_bin:
                print(f"   {box}: ✅ In {placed_in_bin}")
            else:
                print(f"   {box}: 📍 Free [{box_pos[0]:.1f},{box_pos[1]:.1f}]")
        
        # Show robot status
        robot_pos = self.robot.get_position()
        held = self.robot.get_held_objects()
        print(f"🤖 Robot: [{robot_pos[0]:.1f},{robot_pos[1]:.1f}] | Holding: {held or 'Nothing'}")

    def show_debug_status(self) -> None:
        """Show detailed debug status of all boxes and bins"""
        print("\n🔧 DETAILED DEBUG STATUS")
        print("=" * 35)
        
        # Update positions from physics simulation
        if not self._update_object_positions():
            print("⚠️  Could not update object positions")
            return
        
        # Get updated positions
        updated_positions = getattr(self, '_position_update_results', {})
        
        # Show all object positions
        print("📍 All object positions:")
        for obj_name, pos in updated_positions.items():
            print(f"   {obj_name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        
        # Define bin regions
        bin_regions = {
            'bin_1': {'center': [3.0, 0.5], 'tolerance': 0.3},
            'bin_2': {'center': [3.0, 1.0], 'tolerance': 0.3}, 
            'bin_3': {'center': [3.0, 1.5], 'tolerance': 0.3}
        }
        
        boxes = ['box_01', 'box_02', 'box_03']
        
        # Detailed analysis
        print("\n🔍 Distance analysis:")
        for box in boxes:
            if box not in updated_positions:
                print(f"   {box}: ❌ Not found")
                continue
                
            box_pos = updated_positions[box]
            print(f"\n   📦 {box}: [{box_pos[0]:.3f}, {box_pos[1]:.3f}]")
            
            for bin_name, bin_info in bin_regions.items():
                bin_center = bin_info['center']
                tolerance = bin_info['tolerance']
                
                dx = abs(box_pos[0] - bin_center[0])
                dy = abs(box_pos[1] - bin_center[1])
                
                is_in = dx <= tolerance and dy <= tolerance
                status = "✅ IN" if is_in else "❌ OUT"
                print(f"      vs {bin_name}: dx={dx:.3f}, dy={dy:.3f} (tol={tolerance}) → {status}")

    def run_interactive(self):
        """Run the interactive command loop"""
        
        print("\n🎮 INTERACTIVE MODE")
        print("=" * 40)
        
        while self.running:
            try:
                cmd = input("\n> ").strip().lower()
                
                if not cmd:
                    continue
                    
                if cmd in ['quit', 'exit', 'q']:
                    print("👋 Goodbye!")
                    break
                    
                elif cmd in ['help', 'h']:
                    self.show_help()
                    
                elif cmd in ['location', 'locations', 'l']:
                    self.show_locations()
                    
                elif cmd.startswith('w '):
                    # Walk to position: w x y
                    parts = cmd.split()
                    if len(parts) >= 3:
                        try:
                            x, y = float(parts[1]), float(parts[2])
                            print(f"🚶 Walking to [{x:.1f}, {y:.1f}]...")
                            self.robot.start_walking_to([x, y, 0])
                            
                            # Wait for walking to complete
                            while self.robot.is_walking():
                                time.sleep(0.1)
                            
                            print("✅ Arrived!")
                        except ValueError:
                            print("❌ Invalid coordinates. Use: w x y")
                    else:
                        print("❌ Usage: w x y")
                        
                elif cmd.startswith('p '):
                    # Pick up object: p object_name
                    obj_name = cmd[2:].strip()
                    print(f"🖐️  Attempting to pick up {obj_name}...")
                    
                    # Get object position from world
                    if obj_name in self.world.objects:
                        obj_id = self.world.objects[obj_name]
                        try:
                            pos, _ = p.getBasePositionAndOrientation(obj_id)
                            if self.robot.attempt_pick(obj_name, list(pos)):
                                print(f"✅ Successfully picked up {obj_name}!")
                            else:
                                print(f"❌ Failed to pick up {obj_name}")
                        except Exception as e:
                            print(f"❌ Error getting object position: {e}")
                    else:
                        print(f"❌ Object {obj_name} not found in world")
                        
                elif cmd.startswith('d '):
                    # Drop at position: d x y z
                    parts = cmd.split()
                    if len(parts) >= 4:
                        try:
                            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                            print(f"📦 Dropping at [{x:.1f}, {y:.1f}, {z:.1f}]...")
                            if self.robot.attempt_place([x, y, z]):
                                print("✅ Successfully dropped!")
                                # Let physics settle
                                print("⏳ Letting physics settle...")
                                for _ in range(10):
                                    p.stepSimulation()
                                    time.sleep(0.05)
                                print("📍 Object settled")
                            else:
                                print("❌ Failed to drop")
                        except ValueError:
                            print("❌ Invalid coordinates. Use: d x y z")
                    else:
                        print("❌ Usage: d x y z")
                        
                elif cmd.startswith('s'):
                    # Robot status
                    pos = self.robot.get_position()
                    held = self.robot.get_held_objects()
                    print(f"🤖 Robot position: [{pos[0]:.1f}, {pos[1]:.1f}]")
                    print(f"🖐️  Held objects: {held if held else 'None'}")
                    
                elif cmd.startswith('c'):
                    # Show all box/bin status
                    self.show_all_status()
                        
                elif cmd == 'debug':
                    # Detailed debug information
                    self.show_debug_status()
                    
                elif cmd.startswith('cam'):
                    # Camera controls
                    if cmd == 'cam front':
                        self.world.set_camera_view(distance=3.5, yaw=-30, pitch=-15)
                    elif cmd == 'cam side':
                        self.world.set_camera_view(distance=4.0, yaw=-90, pitch=-20)
                    elif cmd == 'cam top':
                        self.world.set_camera_view(distance=5.0, yaw=0, pitch=-70)
                    elif cmd == 'cam follow':
                        robot_pos = self.robot.get_position()
                        self.world.follow_robot_camera(robot_pos)
                    else:
                        print("📷 Camera commands:")
                        print("  cam front  - Front view of robot")
                        print("  cam side   - Side view of robot")  
                        print("  cam top    - Top-down view")
                        print("  cam follow - Follow robot camera")
                    
                else:
                    print("❌ Unknown command. Type 'help' for available commands.")
                    
            except KeyboardInterrupt:
                print("\n🛑 Interrupted by user")
                break
            except Exception as e:
                print(f"❌ Error: {e}")
                
        # Cleanup
        try:
            p.disconnect()
        except:
            pass

    def show_help(self):
        """Show available commands"""
        print("\n🎮 AVAILABLE COMMANDS:")
        print("=" * 30)
        print("w x y        - Walk to position [x, y]")
        print("p object     - Pick up object (e.g., p box_01)")
        print("d x y z      - Drop at position [x, y, z]")
        print("s            - Show robot status")
        print("c            - Show box/bin status for all objects")
        print("l            - Show locations (quick reference)")
        print("debug        - Show debug information")
        print("cam front    - Front view of robot")
        print("cam side     - Side view of robot")
        print("cam top      - Top-down view")
        print("cam follow   - Follow robot camera")
        print("help/h       - Show this help")
        print("quit/q       - Exit interactive mode")


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Interactive robot control")
    parser.add_argument('--headless', action='store_true', 
                       help='Run without GUI')
    
    args = parser.parse_args()
    
    controller = InteractiveController(headless=args.headless)
    
    if controller.start_simulation():
        controller.run_interactive()
    else:
        print("❌ Failed to start simulation")
        sys.exit(1)


if __name__ == "__main__":
    main()
