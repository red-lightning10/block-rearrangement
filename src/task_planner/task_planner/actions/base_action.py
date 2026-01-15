"""Base class for action handlers."""

from abc import ABC, abstractmethod
from typing import List


class BaseAction(ABC):
    """Base class for all action handlers.
    
    Subclasses should implement execute() to handle the specific action logic.
    """
    
    def __init__(self, node, action_name: str, required_args: int):
        """Initialize action handler.
        
        Args:
            node: ROS 2 node instance (for logging and service clients)
            action_name: Name of the action (e.g., "pick-up")
            required_args: Minimum number of arguments required
        """
        self.node = node
        self.action_name = action_name
        self.required_args = required_args
    
    def validate_args(self, args: List[str]) -> bool:
        """Validate that action has required number of arguments.
        
        Args:
            args: Action arguments
            
        Returns:
            bool: True if valid, False otherwise
        """
        if len(args) < self.required_args:
            self.node.get_logger().error(
                f'{self.action_name} requires {self.required_args} argument(s), got {len(args)}')
            return False
        return True
    
    @abstractmethod
    def execute(self, args: List[str]):
        """Execute the action.
        
        Args:
            args: Action arguments
            
        Returns:
            Future (with 'done' method): For simple actions that make a single service call
            bool: For complex actions that handle their own execution
        """
        pass


