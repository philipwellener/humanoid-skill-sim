"""
Unit tests for behavior tree components
"""

import unittest
import sys
import os
import time

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from behavior_tree.node import Node, NodeStatus
from behavior_tree.sequence import Sequence
from behavior_tree.fallback import Fallback
from behavior_tree.action_nodes import WalkToTarget, PickObject, PlaceObject, WaitForHuman


class MockContext:
    """Mock context for testing"""
    
    def __init__(self):
        self.robot_position = [0, 0, 0.5]
        self.objects = {"box_01": {"position": [1, 1, 0]}}
        self.held_objects = {}
        self.simulation_time = 0
    
    def get_robot_position(self):
        return self.robot_position.copy()
    
    def move_robot_to(self, position):
        self.robot_position = position.copy()
        return True
    
    def pick_object(self, object_id):
        if object_id in self.objects:
            self.held_objects[object_id] = self.objects[object_id]
            return True
        return False
    
    def place_object(self, location, object_id=None):
        if object_id and object_id in self.held_objects:
            del self.held_objects[object_id]
            return True
        return False


class TestBehaviorTreeNodes(unittest.TestCase):
    """Test basic behavior tree node functionality"""
    
    def setUp(self):
        self.context = MockContext()
    
    def test_node_creation(self):
        """Test basic node creation"""
        # Use Sequence as a concrete node instead of abstract Node
        node = Sequence(name="test_node")
        self.assertEqual(node.name, "test_node")
        self.assertIsNone(node.start_time)
        self.assertIsNone(node.end_time)
        self.assertEqual(len(node.children), 0)
    
    def test_node_child_management(self):
        """Test adding and removing children"""
        # Use Sequence nodes as concrete implementations
        parent = Sequence(name="parent")
        child1 = Sequence(name="child1")
        child2 = Sequence(name="child2")
        
        parent.add_child(child1)
        parent.add_child(child2)
        
        self.assertEqual(len(parent.children), 2)
        self.assertEqual(parent.children[0], child1)
        self.assertEqual(parent.children[1], child2)
        
        parent.remove_child(child1)
        self.assertEqual(len(parent.children), 1)
        self.assertEqual(parent.children[0], child2)
    
    def test_node_execution_timing(self):
        """Test node execution timing"""
        # Use a simple sequence node
        node = Sequence(name="test")
        
        # Test start execution
        node._start_execution()
        self.assertIsNotNone(node.start_time)
        
        # Test end execution
        result = node._end_execution(NodeStatus.SUCCESS)
        self.assertEqual(result, NodeStatus.SUCCESS)
        self.assertIsNotNone(node.end_time)
        self.assertGreaterEqual(node.end_time, node.start_time)
    
    def test_node_reset(self):
        """Test node reset functionality"""
        # Use a simple sequence node
        node = Sequence(name="test")
        node._start_execution()
        node._end_execution(NodeStatus.SUCCESS)
        
        node.reset()
        self.assertIsNone(node.start_time)
        self.assertIsNone(node.end_time)


class TestBehaviorTreeExecution(unittest.TestCase):
    """Test behavior tree execution logic"""
    
    def setUp(self):
        self.context = MockContext()
    
    def test_sequence_all_success(self):
        """Test sequence when all children succeed"""
        sequence = Sequence()
        
        # Add mock children that always succeed
        class MockSuccessNode(Node):
            def tick(self, context=None):
                return NodeStatus.SUCCESS
        
        sequence.add_child(MockSuccessNode("child1"))
        sequence.add_child(MockSuccessNode("child2"))
        sequence.add_child(MockSuccessNode("child3"))
        
        result = sequence.tick(self.context)
        self.assertEqual(result, NodeStatus.SUCCESS)
    
    def test_sequence_one_failure(self):
        """Test sequence when one child fails"""
        sequence = Sequence()
        
        class MockSuccessNode(Node):
            def tick(self, context=None):
                return NodeStatus.SUCCESS
        
        class MockFailureNode(Node):
            def tick(self, context=None):
                return NodeStatus.FAILURE
        
        sequence.add_child(MockSuccessNode("child1"))
        sequence.add_child(MockFailureNode("child2"))
        sequence.add_child(MockSuccessNode("child3"))
        
        result = sequence.tick(self.context)
        self.assertEqual(result, NodeStatus.FAILURE)
    
    def test_sequence_running_state(self):
        """Test sequence with running child"""
        sequence = Sequence()
        
        class MockSuccessNode(Node):
            def tick(self, context=None):
                return NodeStatus.SUCCESS
        
        class MockRunningNode(Node):
            def tick(self, context=None):
                return NodeStatus.RUNNING
        
        sequence.add_child(MockSuccessNode("child1"))
        sequence.add_child(MockRunningNode("child2"))
        sequence.add_child(MockSuccessNode("child3"))
        
        result = sequence.tick(self.context)
        self.assertEqual(result, NodeStatus.RUNNING)
    
    def test_fallback_first_success(self):
        """Test fallback when first child succeeds"""
        fallback = Fallback()
        
        class MockSuccessNode(Node):
            def tick(self, context=None):
                return NodeStatus.SUCCESS
        
        class MockFailureNode(Node):
            def tick(self, context=None):
                return NodeStatus.FAILURE
        
        fallback.add_child(MockSuccessNode("child1"))
        fallback.add_child(MockFailureNode("child2"))
        
        result = fallback.tick(self.context)
        self.assertEqual(result, NodeStatus.SUCCESS)
    
    def test_fallback_all_failure(self):
        """Test fallback when all children fail"""
        fallback = Fallback()
        
        class MockFailureNode(Node):
            def tick(self, context=None):
                return NodeStatus.FAILURE
        
        fallback.add_child(MockFailureNode("child1"))
        fallback.add_child(MockFailureNode("child2"))
        fallback.add_child(MockFailureNode("child3"))
        
        result = fallback.tick(self.context)
        self.assertEqual(result, NodeStatus.FAILURE)
    
    def test_empty_sequence(self):
        """Test sequence with no children"""
        sequence = Sequence()
        result = sequence.tick(self.context)
        self.assertEqual(result, NodeStatus.SUCCESS)
    
    def test_empty_fallback(self):
        """Test fallback with no children"""
        fallback = Fallback()
        result = fallback.tick(self.context)
        self.assertEqual(result, NodeStatus.FAILURE)


class TestActionNodes(unittest.TestCase):
    """Test action node implementations"""
    
    def setUp(self):
        self.context = MockContext()
    
    def test_walk_to_target_creation(self):
        """Test WalkToTarget node creation"""
        walk_node = WalkToTarget([1, 0, 0], tolerance=0.1)
        self.assertEqual(walk_node.target, [1, 0, 0])
        self.assertEqual(walk_node.tolerance, 0.1)
    
    def test_walk_to_target_already_at_location(self):
        """Test walking when already at target"""
        # Set robot position close to target
        self.context.robot_position = [0.05, 0.05, 0.5]
        walk_node = WalkToTarget([0, 0, 0.5], tolerance=0.1)
        
        # Create a proper context with robot that has get_position method
        class MockRobotForBT:
            def __init__(self, position):
                self.position = position
            def get_position(self):
                return self.position
        
        context_with_robot = type('Context', (), {
            'robot': MockRobotForBT([0.05, 0.05, 0.5])
        })()
        
        result = walk_node.tick(context_with_robot)
        self.assertEqual(result, NodeStatus.SUCCESS)
    
    def test_pick_object_creation(self):
        """Test PickObject node creation"""
        pick_node = PickObject("box_01")
        self.assertEqual(pick_node.object_id, "box_01")
    
    def test_place_object_creation(self):
        """Test PlaceObject node creation"""
        place_node = PlaceObject([2, 2, 0.1])
        self.assertEqual(place_node.location, [2, 2, 0.1])
    
    def test_wait_for_human_creation(self):
        """Test WaitForHuman node creation"""
        wait_node = WaitForHuman(duration=2.0)
        self.assertEqual(wait_node.duration, 2.0)
    
    def test_wait_for_human_timing(self):
        """Test WaitForHuman timing behavior"""
        wait_node = WaitForHuman(duration=0.1)  # Short duration for testing
        
        # First tick should start waiting
        result1 = wait_node.tick(self.context)
        self.assertEqual(result1, NodeStatus.RUNNING)
        
        # Wait for duration to pass
        time.sleep(0.15)
        
        # Second tick should complete
        result2 = wait_node.tick(self.context)
        self.assertEqual(result2, NodeStatus.SUCCESS)
    
    def test_action_node_parameter_validation(self):
        """Test action node parameter validation"""
        # Test invalid target for WalkToTarget
        with self.assertRaises((ValueError, TypeError)):
            WalkToTarget("invalid_target")
        
        # Test invalid duration for WaitForHuman
        with self.assertRaises((ValueError, TypeError)):
            WaitForHuman(duration=-1)
    
    def test_complex_behavior_tree(self):
        """Test a complex behavior tree structure"""
        # Create a sequence that walks, picks, walks, places
        root_sequence = Sequence()
        
        # Mock action nodes that simulate success
        class MockWalkNode(Node):
            def tick(self, context=None):
                return NodeStatus.SUCCESS
        
        class MockPickNode(Node):
            def tick(self, context=None):
                context.held_objects["box_01"] = {"position": [1, 1, 0]}
                return NodeStatus.SUCCESS
        
        class MockPlaceNode(Node):
            def tick(self, context=None):
                if "box_01" in context.held_objects:
                    del context.held_objects["box_01"]
                    return NodeStatus.SUCCESS
                return NodeStatus.FAILURE
        
        root_sequence.add_child(MockWalkNode("walk1"))
        root_sequence.add_child(MockPickNode("pick"))
        root_sequence.add_child(MockWalkNode("walk2"))
        root_sequence.add_child(MockPlaceNode("place"))
        
        result = root_sequence.tick(self.context)
        self.assertEqual(result, NodeStatus.SUCCESS)
        self.assertEqual(len(self.context.held_objects), 0)  # Object should be placed


if __name__ == '__main__':
    unittest.main(verbosity=2)
