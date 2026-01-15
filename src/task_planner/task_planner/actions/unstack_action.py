import rclpy
from interfaces.srv import PickUp
from task_planner.actions.base_action import BaseAction


class UnstackAction(BaseAction):
    """Handler for unstack action: pick object from top of another object."""
    
    def __init__(self, node, pick_up_client):
        """Initialize unstack action handler.
        
        Args:
            node: ROS 2 node instance
            pick_up_client: Client for /pick_up service
        """
        super().__init__(node, "unstack", required_args=2)
        self.pick_up_client = pick_up_client
    
    def execute(self, args: list):
        """Execute unstack action.
        
        Args:
            args: [object_id, base_object] - base_object is the object it's on
        
        Returns:
            Future: Service call future (action_executor will handle spinning)
        """
        if not self.validate_args(args):
            return False
        
        object_id = args[0]
        base_object = args[1]
        req = PickUp.Request()
        req.object_id = object_id
        req.support_surface = base_object
        
        future = self.pick_up_client.call_async(req)
        return future


