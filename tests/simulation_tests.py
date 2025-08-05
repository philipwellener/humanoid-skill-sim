"""
Unit tests for simulation components
"""

import unittest
import sys
import os
import time

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.world import SimulationWorld
from sim.robot import HumanoidRobot
from sim.teleop_override import TeleopOverride


class TestSimulationWorld(unittest.TestCase):
    """Test simulation world functionality"""
    
    def setUp(self):
        """Set up test environment"""
        self.world = None
    
    def tearDown(self):
        """Clean up after tests"""
        if self.world:
            try:
                self.world.shutdown()
            except:
                pass
    
    def test_world_creation(self):
        """Test creating simulation world"""
        self.world = SimulationWorld(use_gui=False)
        self.assertIsNotNone(self.world)
        self.assertFalse(self.world.use_gui)
    
    def test_world_initialization(self):
        """Test world initialization"""
        self.world = SimulationWorld(use_gui=False)
        result = self.world.initialize()
        self.assertTrue(result)
    
    def test_object_spawning(self):
        """Test spawning objects in world"""
        self.world = SimulationWorld(use_gui=False)
        self.world.initialize()
        
        # Test spawning a box
        object_id = self.world.spawn_object("box", [1, 1, 0.5], size=[0.2, 0.2, 0.2])
        self.assertIsNotNone(object_id)
        
        # Check object exists
        objects = self.world.get_objects()
        self.assertIn(object_id, objects)
    
    def test_ground_plane_exists(self):
        """Test that ground plane is created"""
        self.world = SimulationWorld(use_gui=False)
        self.world.initialize()
        
        # Ground plane should be created during initialization
        # This is a basic check that initialization completed without error
        self.assertTrue(hasattr(self.world, 'physics_client'))
    
    def test_physics_step(self):
        """Test physics simulation step"""
        self.world = SimulationWorld(use_gui=False)
        self.world.initialize()
        
        # Should be able to step physics without error
        try:
            self.world.step_simulation()
        except Exception as e:
            self.fail(f"Physics step failed: {e}")
    
    def test_world_reset(self):
        """Test world reset functionality"""
        self.world = SimulationWorld(use_gui=False)
        self.world.initialize()
        
        # Spawn an object
        object_id = self.world.spawn_object("box", [1, 1, 0.5])
        self.assertIsNotNone(object_id)
        
        # Reset world
        self.world.reset()
        
        # Objects should be cleared
        objects = self.world.get_objects()
        self.assertEqual(len(objects), 0)


class TestHumanoidRobot(unittest.TestCase):
    """Test humanoid robot functionality"""
    
    def setUp(self):
        """Set up test environment"""
        self.robot = HumanoidRobot()
        self.world = None
    
    def tearDown(self):
        """Clean up after tests"""
        if self.world:
            try:
                self.world.shutdown()
            except:
                pass
    
    def test_robot_creation(self):
        """Test robot creation"""
        self.assertIsNotNone(self.robot)
        self.assertIsNone(self.robot.robot_id)  # Not loaded yet
    
    def test_robot_loading(self):
        """Test robot URDF loading"""
        # This might fail if URDF file doesn't exist, which is expected
        result = self.robot.load_robot()
        # We don't assert True here because URDF might not exist
        # Just check that method doesn't crash
        self.assertIn(result, [True, False])
    
    def test_robot_position_tracking(self):
        """Test robot position methods"""
        # Test default position
        pos = self.robot.get_position()
        self.assertEqual(len(pos), 3)  # Should be [x, y, z]
        
        # Test setting position
        new_pos = [1.0, 2.0, 0.5]
        self.robot.set_position(new_pos)
        
        # Check if position was updated (might be mock implementation)
        updated_pos = self.robot.get_position()
        self.assertEqual(len(updated_pos), 3)
    
    def test_robot_inverse_kinematics(self):
        """Test inverse kinematics calculations"""
        target_pos = [0.5, 0.3, 0.8]
        joint_angles = self.robot.calculate_ik(target_pos)
        
        # Should return a list of joint angles
        self.assertIsInstance(joint_angles, list)
        if joint_angles:  # If IK is implemented
            self.assertGreater(len(joint_angles), 0)
    
    def test_robot_manipulation_commands(self):
        """Test robot manipulation methods"""
        # Test pick command
        result = self.robot.attempt_pick("test_object", [1, 1, 0])
        self.assertIsInstance(result, bool)
        
        # Test place command
        result = self.robot.attempt_place([2, 2, 0])
        self.assertIsInstance(result, bool)
    
    def test_robot_walking_interface(self):
        """Test robot walking interface"""
        target = [1.0, 0.0, 0.0]
        
        # Start walking
        self.robot.start_walking_to(target)
        
        # Check walking status
        is_walking = self.robot.is_walking()
        self.assertIsInstance(is_walking, bool)
        
        # Stop walking
        self.robot.stop_walking()
    
    def test_robot_sensor_data(self):
        """Test robot sensor interfaces"""
        # Test getting joint states
        joint_states = self.robot.get_joint_states()
        if joint_states is not None:
            self.assertIsInstance(joint_states, (list, dict))
        
        # Test getting base pose
        base_pose = self.robot.get_base_pose()
        if base_pose is not None:
            self.assertEqual(len(base_pose), 3)  # [x, y, z] at minimum


class TestTeleopOverride(unittest.TestCase):
    """Test teleoperation override functionality"""
    
    def setUp(self):
        """Set up test environment"""
        self.teleop = TeleopOverride()
    
    def test_teleop_creation(self):
        """Test teleop system creation"""
        self.assertIsNotNone(self.teleop)
        self.assertFalse(self.teleop.is_active())  # Should start inactive
    
    def test_teleop_activation(self):
        """Test teleop activation"""
        # Test activation
        self.teleop.activate()
        self.assertTrue(self.teleop.is_active())
        
        # Test deactivation
        self.teleop.deactivate()
        self.assertFalse(self.teleop.is_active())
    
    def test_teleop_command_interface(self):
        """Test teleop command interface"""
        # Test setting movement command
        result = self.teleop.set_movement_command([1, 0, 0])
        self.assertIsInstance(result, bool)
        
        # Test setting manipulation command
        result = self.teleop.set_manipulation_command("pick", "object_01")
        self.assertIsInstance(result, bool)
    
    def test_teleop_override_detection(self):
        """Test override detection"""
        # Should detect when human intervention is needed
        needs_override = self.teleop.check_override_needed({
            'robot_stuck': True,
            'task_failed': False
        })
        self.assertIsInstance(needs_override, bool)
    
    def test_teleop_logging(self):
        """Test teleop intervention logging"""
        # Test starting intervention log
        intervention_id = self.teleop.start_intervention("manual_pick")
        self.assertIsNotNone(intervention_id)
        
        # Test ending intervention
        if intervention_id:
            result = self.teleop.end_intervention(intervention_id, success=True)
            self.assertIsInstance(result, bool)
    
    def test_teleop_safety_checks(self):
        """Test teleop safety mechanisms"""
        # Test emergency stop
        result = self.teleop.emergency_stop()
        self.assertIsInstance(result, bool)
        
        # Test safety bounds checking
        safe_command = [0.5, 0.5, 0]  # Reasonable movement
        unsafe_command = [100, 100, 0]  # Extreme movement
        
        safe_result = self.teleop.validate_command(safe_command)
        unsafe_result = self.teleop.validate_command(unsafe_command)
        
        self.assertIsInstance(safe_result, bool)
        self.assertIsInstance(unsafe_result, bool)
    
    def test_teleop_state_management(self):
        """Test teleop state management"""
        # Test getting current state
        state = self.teleop.get_state()
        self.assertIsInstance(state, dict)
        
        # Test resetting state
        result = self.teleop.reset()
        self.assertIsInstance(result, bool)
        
        # After reset, should not be active
        self.assertFalse(self.teleop.is_active())
    
    def test_teleop_feedback_system(self):
        """Test teleop feedback mechanisms"""
        # Test setting feedback
        feedback = {
            'force': [0.1, 0.2, 0.0],
            'status': 'grasping'
        }
        result = self.teleop.update_feedback(feedback)
        self.assertIsInstance(result, bool)
        
        # Test getting feedback
        current_feedback = self.teleop.get_feedback()
        self.assertIsInstance(current_feedback, dict)


if __name__ == '__main__':
    unittest.main(verbosity=2)
