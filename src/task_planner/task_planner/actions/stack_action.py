"""Stack action handler."""

import rclpy
from interfaces.srv import Place
from task_planner.actions.base_action import BaseAction


class StackAction(BaseAction):
    """Handler for stack action: place object on another object."""
    
    def __init__(self, node, place_client):
        """Initialize stack action handler.
        
        Args:
            node: ROS 2 node instance
            place_client: Client for /place service
        """
        super().__init__(node, "stack", required_args=2)
        self.place_client = place_client
    
    def execute(self, args: list):
        """Execute stack action.
        
        Args:
            args: [object_id, target_object]
            
        Returns:
            Future: Service call future
        """
        if not self.validate_args(args):
            return False
        
        object_id = args[0]
        target_object = args[1]
        req = Place.Request()
        req.object_id = object_id
        req.support_surface = target_object
        
        future = self.place_client.call_async(req)
        return future


