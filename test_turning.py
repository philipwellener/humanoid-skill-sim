#!/usr/bin/env python3
"""
Test script to demonstrate robot turning behavior
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from behavior_tree.action_nodes import TurnToFace, WalkToTarget, PickObject
from behavior_tree.node import NodeStatus
import time

class MockRobot:
    """Mock robot for testing turning behavior"""
    def __init__(self):
        self.position = [0.0, 0.0, 0.5]
        self.orientation = 0.0
        self.target_orientation = None
        self.turning = False
        self.turn_start_time = None
        self.turn_speed = 2.0
        
    def get_position(self):
        return self.position.copy()
        
    def get_orientation(self):
        return self.orientation
        
    def turn_to_face(self, target_position):
        """Turn to face a target position"""
        import numpy as np
        dx = target_position[0] - self.position[0]
        dy = target_position[1] - self.position[1]
        target_angle = np.arctan2(dy, dx)
        self.target_orientation = target_angle
        self.turning = True
        self.turn_start_time = time.time()
        print(f"🔄 Starting turn from {self.orientation:.2f} to {target_angle:.2f} radians")
        
    def turn_to_angle(self, target_angle):
        """Turn to a specific angle"""
        self.target_orientation = target_angle
        self.turning = True
        self.turn_start_time = time.time()
        print(f"🔄 Starting turn from {self.orientation:.2f} to {target_angle:.2f} radians")
        
    def is_turning(self):
        """Check if robot is currently turning"""
        if not self.turning:
            return False
            
        import numpy as np
        
        # Calculate angle difference
        angle_diff = self._normalize_angle(self.target_orientation - self.orientation)
        
        if abs(angle_diff) < 0.05:  # Close enough
            self.orientation = self.target_orientation
            self.turning = False
            print(f"✅ Turn complete, now facing {self.orientation:.2f} radians")
            return False
            
        # Update orientation during turn
        elapsed_time = time.time() - self.turn_start_time
        turn_direction = 1 if angle_diff > 0 else -1
        turn_amount = self.turn_speed * elapsed_time * turn_direction
        
        # Don't overshoot
        if abs(turn_amount) > abs(angle_diff):
            turn_amount = angle_diff
            
        self.orientation = self._normalize_angle(self.orientation + turn_amount)
        return True
        
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        import numpy as np
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

def test_turning():
    """Test the turning behavior"""
    print("🤖 Testing Robot Turning Behavior\n")
    
    # Create mock robot
    robot = MockRobot()
    context = {'robot': robot}
    
    # Test 1: Turn to face a specific position
    print("Test 1: Turn to face position [1.0, 1.0]")
    turn_action = TurnToFace(target=[1.0, 1.0])
    
    # Execute the action
    max_ticks = 50
    tick_count = 0
    
    while tick_count < max_ticks:
        status = turn_action.tick(context)
        tick_count += 1
        
        if status == NodeStatus.SUCCESS:
            print(f"✅ Turn completed successfully after {tick_count} ticks\n")
            break
        elif status == NodeStatus.FAILURE:
            print(f"❌ Turn failed after {tick_count} ticks\n")
            break
        elif status == NodeStatus.RUNNING:
            print(f"  Tick {tick_count}: Still turning... (orientation: {robot.orientation:.2f})")
            time.sleep(0.1)  # Small delay to see the progression
        
        if tick_count >= max_ticks:
            print(f"⏰ Turn timed out after {max_ticks} ticks\n")
    
    # Test 2: Turn to a specific angle
    print("Test 2: Turn to angle π/2 (90 degrees)")
    import numpy as np
    turn_action2 = TurnToFace(angle=np.pi/2)
    turn_action2.reset()
    
    tick_count = 0
    while tick_count < max_ticks:
        status = turn_action2.tick(context)
        tick_count += 1
        
        if status == NodeStatus.SUCCESS:
            print(f"✅ Turn completed successfully after {tick_count} ticks")
            print(f"Final orientation: {robot.orientation:.2f} radians ({np.degrees(robot.orientation):.1f} degrees)\n")
            break
        elif status == NodeStatus.FAILURE:
            print(f"❌ Turn failed after {tick_count} ticks\n")
            break
        elif status == NodeStatus.RUNNING:
            print(f"  Tick {tick_count}: Still turning... (orientation: {robot.orientation:.2f})")
            time.sleep(0.1)
        
        if tick_count >= max_ticks:
            print(f"⏰ Turn timed out after {max_ticks} ticks\n")

if __name__ == "__main__":
    test_turning()
