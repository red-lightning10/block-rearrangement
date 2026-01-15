from interfaces.srv import Place
from task_planner.actions.base_action import BaseAction


class PutDownAction(BaseAction):
    """Handler for put-down action: place object on table."""
    
    def __init__(self, node, place_client):
        """Initialize put-down action handler.
        
        Args:
            node: ROS 2 node instance
            place_client: Client for /place service
        """
        super().__init__(node, "put-down", required_args=1)
        self.place_client = place_client
    
    def execute(self, args: list):
        """Execute put-down action.
        
        Args:
            args: [object_id]
            
        Returns:
            Future: Service call future (action_executor will handle spinning)
        """
        if not self.validate_args(args):
            self.node.get_logger().error(f'Put-down: Invalid arguments: {args}')
            return False
        
        object_id = args[0]
        self.node.get_logger().info(f'Put-down: Placing {object_id} on table_base')

        if not self.place_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Put-down: /place service not available')
            return False
        
        req = Place.Request()
        req.object_id = object_id
        req.support_surface = "table_base"
        
        self.node.get_logger().info(f'Put-down: Calling /place service with object_id={object_id}, support_surface=table_base')
        future = self.place_client.call_async(req)
        if future is None:
            self.node.get_logger().error('Put-down: Failed to create service call future')
            return False
        self.node.get_logger().info(f'Put-down: Service call initiated for {object_id}')
        return future


