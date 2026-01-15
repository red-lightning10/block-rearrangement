import rclpy
from interfaces.srv import PickUp
from task_planner.actions.base_action import BaseAction


class PickUpAction(BaseAction):
    """Handler for pick-up action: pick object from table."""
    
    def __init__(self, node, pick_up_client):
        """Initialize pick-up action handler.
        
        Args:
            node: ROS 2 node instance
            pick_up_client: Client for /pick_up service
        """
        super().__init__(node, "pick-up", required_args=1)
        self.pick_up_client = pick_up_client
    
    def execute(self, args: list):
        """Execute pick-up action.
        
        Args:
            args: [object_id]
            
        Returns:
            Future: Service call future (action_executor will handle spinning)
        """
        if not self.validate_args(args):
            return False
        
        object_id = args[0]
        req = PickUp.Request()
        req.object_id = object_id
        req.support_surface = "table_base"
        
        future = self.pick_up_client.call_async(req)
        return future


