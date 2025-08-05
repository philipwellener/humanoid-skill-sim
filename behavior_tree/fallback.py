"""
Fallback (Selector) node implementation for behavior trees
"""

from typing import Any
try:
    from .node import Node, NodeStatus
except ImportError:
    from node import Node, NodeStatus


class Fallback(Node):
    """
    Fallback (Selector) node executes children in order until one succeeds.
    Succeeds if any child succeeds.
    Fails if all children fail.
    Returns RUNNING while children are executing.
    """
    
    def __init__(self, children=None, name: str = None):
        super().__init__(name)
        self.current_child_index = 0
        
        if children:
            for child in children:
                self.add_child(child)
    
    def tick(self, context: Any = None) -> NodeStatus:
        """Execute fallback logic"""
        if not self.children:
            return self._end_execution(NodeStatus.FAILURE)
            
        if self.start_time is None:
            self._start_execution()
            
        # Try children in order until one succeeds
        while self.current_child_index < len(self.children):
            current_child = self.children[self.current_child_index]
            child_status = current_child.tick(context)
            
            if child_status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif child_status == NodeStatus.SUCCESS:
                self.reset()
                return self._end_execution(NodeStatus.SUCCESS)
            elif child_status == NodeStatus.FAILURE:
                self.current_child_index += 1
                # Continue to next child
            else:
                raise ValueError(f"Unexpected status from child: {child_status}")
                
        # All children failed
        self.reset()
        return self._end_execution(NodeStatus.FAILURE)
    
    def reset(self) -> None:
        """Reset fallback state"""
        super().reset()
        self.current_child_index = 0
