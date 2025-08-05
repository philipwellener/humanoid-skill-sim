"""
Unit tests for individual skills
"""

import unittest
import sys
import os
import time

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from skills.walk import WalkToTargetSkill
from skills.pick import PickObjectSkill
from skills.place import PlaceObjectSkill
from sim.robot import HumanoidRobot
from sim.world import SimulationWorld


class MockRobot:
    """Mock robot for testing"""
    
    def __init__(self):
        self.position = [0, 0, 0.5]
        self.reach_distance = 1.0
        self.held_objects = {}
        self.walking = False
        self.target = None
        self.walk_will_fail = False
        self.robot_id = 1  # Mock robot ID for IK calculations
        self.joints = [0] * 7  # Mock joint states (7 joints for arm)
        self.end_effector_link = 6  # End effector link index
        
    def get_position(self):
        return self.position.copy()
        
    def start_walking_to(self, target):
        self.walking = True
        self.target = target
        
        # Check if target is unreasonably far for testing retry logic
        current_pos = self.get_position()
        distance = ((target[0] - current_pos[0])**2 + (target[1] - current_pos[1])**2)**0.5
        self.walk_will_fail = distance > 5.0  # Mark long distance walks as destined to fail
        
    def is_walking(self):
        if self.walking:
            # Simulate walking completion after short delay
            time.sleep(0.1)
            
            if not self.walk_will_fail:
                # Normal successful walk
                self.position = self.target.copy()
            # If walk will fail, don't update position
            
            self.walking = False
        return False
        
    def attempt_pick(self, object_id, position):
        # Simulate successful pick
        self.held_objects[object_id] = True
        return True
        
    def attempt_place(self, location, object_id=None):
        # Simulate successful place
        if object_id:
            if object_id in self.held_objects:
                del self.held_objects[object_id]
                return True
        else:
            # Place any object
            if self.held_objects:
                key = next(iter(self.held_objects))
                del self.held_objects[key]
                return True
        return False
        
    def is_holding(self, object_id):
        return object_id in self.held_objects
        
    def is_holding_any(self):
        return len(self.held_objects) > 0
        
    def get_held_objects(self):
        return list(self.held_objects.keys())


class MockWorld:
    """Mock world for testing"""
    
    def __init__(self):
        self.objects = {
            "box_01": [1.5, 0.8, 0.05],
            "box_02": [2.0, 1.2, 0.05]
        }
        
    def get_object_position(self, object_id):
        """Get position of object"""
        return self.objects.get(object_id)
        
    def has_object(self, object_id):
        """Check if object exists"""
        return object_id in self.objects
        
    def remove_object(self, object_id):
        """Remove object from world"""
        if object_id in self.objects:
            del self.objects[object_id]
            
    def add_object(self, object_id, position):
        """Add object to world"""
        self.objects[object_id] = position
        
    def get_object_position(self, object_id):
        return self.objects.get(object_id)
        
    def set_object_position(self, object_id, position):
        if object_id in self.objects:
            self.objects[object_id] = position
            return True
        return False
        
    def get_all_objects(self):
        return list(self.objects.keys())


class TestWalkToTargetSkill(unittest.TestCase):
    """Test cases for WalkToTarget skill"""
    
    def setUp(self):
        self.robot = MockRobot()
        self.world = MockWorld()
        
    def test_successful_walk(self):
        """Test successful walking to target"""
        skill = WalkToTargetSkill(target=[1.0, 1.0, 0.0], tolerance=0.1)
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "SUCCESS")
        self.assertIsNone(result["error"])
        self.assertGreater(result["duration"], 0)
        
    def test_walk_already_at_target(self):
        """Test when robot is already at target"""
        skill = WalkToTargetSkill(target=[0.0, 0.0, 0.0], tolerance=0.6)
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "SUCCESS")
        
    def test_walk_retry_logic(self):
        """Test retry logic when walk fails"""
        # Set robot far from target with tight tolerance
        skill = WalkToTargetSkill(target=[10.0, 10.0, 0.0], tolerance=0.01)
        
        # Execute multiple times to test retry
        result1 = skill.execute(self.robot, self.world)
        self.assertIn(result1["status"], ["FAILURE", "RETRY"])
        
        if skill.can_retry():
            result2 = skill.execute(self.robot, self.world)
            self.assertIsNotNone(result2)


class TestPickObjectSkill(unittest.TestCase):
    """Test cases for PickObject skill"""
    
    def setUp(self):
        self.robot = MockRobot()
        self.world = MockWorld()
        
    def test_successful_pick(self):
        """Test successful object pickup"""
        # Move robot close to object
        self.robot.position = [1.4, 0.7, 0.5]
        
        skill = PickObjectSkill(object_id="box_01", test_mode=True)
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "SUCCESS")
        self.assertTrue(result["grasp_success"])
        self.assertTrue(self.robot.is_holding("box_01"))
        
    def test_pick_object_not_found(self):
        """Test picking non-existent object"""
        skill = PickObjectSkill(object_id="nonexistent")
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "FAILURE")
        self.assertIn("not found", result["error"])
        
    def test_pick_out_of_reach(self):
        """Test picking object that's out of reach"""
        # Keep robot at origin, object is at [1.5, 0.8]
        skill = PickObjectSkill(object_id="box_01")
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "FAILURE")
        self.assertIn("out of reach", result["error"])
        
    def test_pick_already_held(self):
        """Test picking object that's already held"""
        self.robot.held_objects["box_01"] = True
        self.robot.position = [1.4, 0.7, 0.5]
        
        skill = PickObjectSkill(object_id="box_01")
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "SUCCESS")
        self.assertIn("already held", result["error"])


class TestPlaceObjectSkill(unittest.TestCase):
    """Test cases for PlaceObject skill"""
    
    def setUp(self):
        self.robot = MockRobot()
        self.world = MockWorld()
        # Give robot an object to place
        self.robot.held_objects["box_01"] = True
        
    def test_successful_place(self):
        """Test successful object placement"""
        skill = PlaceObjectSkill(location=[0.5, 0.5, 0.1], test_mode=True)
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "SUCCESS")
        self.assertTrue(result["placement_success"])
        self.assertFalse(self.robot.is_holding("box_01"))
        
    def test_place_specific_object(self):
        """Test placing specific object"""
        self.robot.held_objects["box_02"] = True
        
        skill = PlaceObjectSkill(location=[0.5, 0.5, 0.1], object_id="box_01")
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "SUCCESS")
        
    def test_place_no_objects_held(self):
        """Test placing when no objects are held"""
        self.robot.held_objects.clear()
        
        skill = PlaceObjectSkill(location=[0.5, 0.5, 0.1])
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "FAILURE")
        self.assertIn("not holding any objects", result["error"])
        
    def test_place_out_of_reach(self):
        """Test placing at location out of reach"""
        skill = PlaceObjectSkill(location=[10.0, 10.0, 0.1])
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "FAILURE")
        self.assertIn("out of reach", result["error"])
        
    def test_place_object_not_held(self):
        """Test placing specific object not held"""
        skill = PlaceObjectSkill(location=[0.5, 0.5, 0.1], object_id="box_02")
        result = skill.execute(self.robot, self.world)
        
        self.assertEqual(result["status"], "FAILURE")
        self.assertIn("not holding", result["error"])


class TestSkillMetrics(unittest.TestCase):
    """Test skill metrics and performance monitoring"""
    
    def setUp(self):
        self.robot = MockRobot()
        self.world = MockWorld()
        
    def test_skill_duration_tracking(self):
        """Test that skills track execution duration"""
        skill = WalkToTargetSkill(target=[1.0, 1.0, 0.0])
        result = skill.execute(self.robot, self.world)
        
        self.assertGreater(result["duration"], 0)
        self.assertLess(result["duration"], 2.0)  # Should complete quickly in mock
        
    def test_skill_attempt_counting(self):
        """Test that skills count execution attempts"""
        skill = PickObjectSkill(object_id="box_01")
        
        result1 = skill.execute(self.robot, self.world)
        self.assertEqual(result1["attempt"], 1)
        
        if skill.can_retry():
            result2 = skill.execute(self.robot, self.world)
            self.assertEqual(result2["attempt"], 2)
            
    def test_skill_reset_functionality(self):
        """Test skill reset functionality"""
        skill = WalkToTargetSkill(target=[1.0, 1.0, 0.0])
        
        # Execute once
        skill.execute(self.robot, self.world)
        self.assertGreater(skill.attempts, 0)
        
        # Reset and check
        skill.reset()
        self.assertEqual(skill.attempts, 0)
        self.assertEqual(skill.status, "IDLE")


if __name__ == '__main__':
    # Run tests
    unittest.main(verbosity=2)
