"""
Base Node class for behavior tree implementation
"""

from enum import Enum
from abc import ABC, abstractmethod
from typing import Optional, List, Any
import uuid
import time


class NodeStatus(Enum):
    """Status returned by behavior tree nodes"""
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE" 
    RUNNING = "RUNNING"


class Node(ABC):
    """Base class for all behavior tree nodes"""
    
    def __init__(self, name: Optional[str] = None):
        self.name = name or self.__class__.__name__
        self.id = str(uuid.uuid4())[:8]
        self.children: List['Node'] = []
        self.parent: Optional['Node'] = None
        self.status = NodeStatus.FAILURE
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        
    def add_child(self, child: 'Node') -> None:
        """Add a child node"""
        self.children.append(child)
        child.parent = self
        
    def remove_child(self, child: 'Node') -> None:
        """Remove a child node"""
        if child in self.children:
            self.children.remove(child)
            child.parent = None
            
    @abstractmethod
    def tick(self, context: Any = None) -> NodeStatus:
        """
        Execute the node logic and return status
        
        Args:
            context: Shared context/blackboard for the behavior tree
            
        Returns:
            NodeStatus indicating the result of execution
        """
        pass
    
    def reset(self) -> None:
        """Reset node status and timing"""
        self.status = NodeStatus.FAILURE
        self.start_time = None
        self.end_time = None
        for child in self.children:
            child.reset()
            
    def get_duration(self) -> Optional[float]:
        """Get execution duration if completed"""
        if self.start_time and self.end_time:
            return self.end_time - self.start_time
        return None
        
    def _start_execution(self) -> None:
        """Mark start of execution"""
        self.start_time = time.time()
        
    def _end_execution(self, status: NodeStatus) -> NodeStatus:
        """Mark end of execution and set status"""
        self.end_time = time.time()
        self.status = status
        return status
        
    def __str__(self) -> str:
        return f"{self.name}({self.id})"
        
    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name='{self.name}', id='{self.id}')"
