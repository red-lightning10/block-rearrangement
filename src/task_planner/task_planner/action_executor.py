import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interfaces.srv import GetAction, PickUp, Place, GetGrounding, AdjustObjects
import sys
import re
import threading

from task_planner.actions import (
    PickUpAction,
    PutDownAction,
    StackAction,
    UnstackAction,
)


class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')
        
        # MutuallyExclusiveCallbackGroup since actions are sequential and dependent
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        self.get_action_client = self.create_client(
            GetAction, 
            '/get_action',
            callback_group=self.callback_group
        )
        self.pick_up_client = self.create_client(
            PickUp, 
            '/pick_up',
            callback_group=self.callback_group
        )
        self.place_client = self.create_client(
            Place, 
            '/place',
            callback_group=self.callback_group
        )
        self.grounding_client = self.create_client(
            GetGrounding,
            '/grounding/get_state',
            callback_group=self.callback_group
        )
        self.adjust_objects_client = self.create_client(
            AdjustObjects,
            '/grounding/adjust_objects',
            callback_group=self.callback_group
        )
        
        # Initialize action handlers
        self._init_action_handlers()
        
        self.get_logger().info('Action Executor initialized')
        self.get_logger().info('Waiting for services...')
        
        # Wait for all services
        self.get_action_client.wait_for_service()
        self.pick_up_client.wait_for_service()
        self.place_client.wait_for_service()
        self.grounding_client.wait_for_service()
        self.adjust_objects_client.wait_for_service()
        
        self.get_logger().info('All services available!')
    
    def _init_action_handlers(self):
        """Initialize action handlers and register them."""
        
        self.action_handlers = {
            "pick-up": PickUpAction(self, self.pick_up_client),
            "put-down": PutDownAction(self, self.place_client),
            "stack": StackAction(self, self.place_client),
            "unstack": UnstackAction(self, self.pick_up_client),
        }
        
        self.get_logger().info(f'Registered {len(self.action_handlers)} action handlers: {list(self.action_handlers.keys())}')
    
    def execute_goal(self, goal: str, max_steps: int = 10):
        """Execute a TAMP plan to achieve the given goal.
        
        Args:
            goal: PDDL goal expression (e.g., "(on red_cube_0 blue_cube_0)")
            max_steps: Maximum number of action steps to execute
        
        Returns:
            bool: True if goal achieved, False otherwise
        """
        self.get_logger().info(f'Starting execution for goal: {goal}')
        
        for _ in range(max_steps):
            self.get_logger().info('Getting current grounding...')
            ground_req = GetGrounding.Request()
            ground_future = self.grounding_client.call_async(ground_req)
            
            rclpy.spin_until_future_complete(
                self,
                ground_future,
                timeout_sec=5.0
            )
            
            if not ground_future.done():
                self.get_logger().error('Grounding service call timed out')
                return False
            
            grounding_response = ground_future.result()
            if not grounding_response.success:
                self.get_logger().error(f'Grounding failed: {grounding_response.message}')
                return False
            
            # Process grounded predicates (remove parentheses for TAMPInterface)
            grounded_predicates = grounding_response.grounded_predicates
            grounded_states = [p.strip('()').strip() for p in grounded_predicates]
            self.get_logger().info(f'Got {len(grounded_states)} grounded predicates')
            
            # Check if goal is already satisfied before planning
            if self._is_goal_satisfied(goal, grounded_states):
                self.get_logger().info('Goal already satisfied!')
                return True
            
            # Adjust object positions based on grounded predicates
            self.get_logger().info('Adjusting object positions based on grounded predicates...')
            adjust_req = AdjustObjects.Request()
            adjust_future = self.adjust_objects_client.call_async(adjust_req)
            
            rclpy.spin_until_future_complete(
                self,
                adjust_future,
                timeout_sec=5.0
            )
            
            if adjust_future.done():
                adjust_response = adjust_future.result()
                if adjust_response.success:
                    self.get_logger().info(f'Object positions adjusted: {adjust_response.message}')
                else:
                    self.get_logger().warn(f'Object adjustment warning: {adjust_response.message}')
            else:
                self.get_logger().warn('Adjust objects service call timed out (non-fatal)')
            
            # Get next action from planner with grounded predicates
            action_req = GetAction.Request()
            action_req.goal = goal
            action_req.grounded_predicates = grounded_states
            
            action_future = self.get_action_client.call_async(action_req)
            
            # Block until planning completes or times out
            rclpy.spin_until_future_complete(
                self, 
                action_future, 
                timeout_sec=30.0
            )
            
            if not action_future.done():
                self.get_logger().error('Planning service call timed out or did not complete')
                return False
            
            action_response = action_future.result()
            
            if not action_response.success:
                self.get_logger().error(f'Planning failed: {action_response.message}')
                return False
            
            action_name = action_response.action
            action_args = action_response.args
            
            self.get_logger().info(f'Executing: {action_name} {action_args}')
            success = self.execute_action(action_name, action_args)
            
            if not success:
                self.get_logger().error(f'Action execution failed: {action_name} {action_args}')
                self.get_logger().error('Stopping execution due to action failure')
                return False
            
            self.get_logger().info(f'Action {action_name} completed successfully, continuing...')
        
        self.get_logger().warn(f'Max steps ({max_steps}) reached without achieving goal')
        return False
    
    def execute_action(self, action_name: str, args: list) -> bool:
        """Execute a single action by routing to the appropriate handler.
        
        Args:
            action_name: PDDL action name (e.g., "pick-up", "stack")
            args: Action arguments (e.g., ["red_cube_0"] or ["red_cube_0", "blue_cube_0"])
        
        Returns:
            bool: True if action succeeded, False otherwise
        """
        if action_name not in self.action_handlers:
            self.get_logger().error(f'Unknown action: {action_name}')
            self.get_logger().info(f'Available actions: {list(self.action_handlers.keys())}')
            return False
        
        handler = self.action_handlers[action_name]
        result = handler.execute(args)

        if result is False:
            self.get_logger().error(f'{action_name} execution returned False (likely validation error)')
            return False
        
        if hasattr(result, 'done'):
            self.get_logger().info(f'{action_name}: Waiting for service call to complete...')
            rclpy.spin_until_future_complete(
                self, 
                result, 
                timeout_sec=60.0
            )
            
            if not result.done():
                self.get_logger().error(f'{action_name} service call timed out or did not complete')
                return False
            
            response = result.result()
            if response is None:
                self.get_logger().error(f'{action_name} service call returned None response')
                return False
            
            if response.success:
                self.get_logger().info(f'{action_name} completed successfully: {response.message}')
            else:
                self.get_logger().error(f'{action_name} failed: {response.message}')
            
            return response.success
        else:
            return result
    
    def _is_goal_satisfied(self, goal: str, grounded_states: list) -> bool:
        """Check if the goal is satisfied in the given grounded states.
        
        Args:
            goal: PDDL goal expression (e.g., "(on red_cube_0 blue_cube_0)" or "(AND (on red_cube_0 blue_cube_0) (on blue_cube_0 yellow_cube_0))")
            grounded_states: List of grounded predicate strings (e.g., ["ONTABLE red_cube_0", "CLEAR blue_cube_0"])
        
        Returns:
            bool: True if goal is satisfied, False otherwise
        """
        
        # Preprocess the groundings string for comparison with goal
        grounded_lower = [state.lower() for state in grounded_states]
        goal_lower = goal.lower().strip()

        if goal_lower.startswith('(') and goal_lower.endswith(')'):
            goal_lower = goal_lower[1:-1].strip()

        if goal_lower.startswith('and'):
            goal_lower = goal_lower[3:].strip()  # Remove "and"
            predicates = re.findall(r'\([^)]+\)', goal_lower)
            
            for pred in predicates:
                pred_clean = pred.strip('()').strip()
                if pred_clean not in grounded_lower:
                    return False
            return True
        else:
            goal_clean = goal_lower.strip('()').strip() # Single predicate goal: "(pred arg1 arg2)"
            return goal_clean in grounded_lower


def main(args=None):
    rclpy.init(args=args)
    executor_node = ActionExecutor()
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(executor_node)

    # Run executor in a separate thread to handle callbacks while execute_goal runs
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    try:
        if len(sys.argv) > 1:
            goal = sys.argv[1]
            executor_node.execute_goal(goal)
        else:
            executor_node.get_logger().info('Usage: ros2 run task_planner action_executor "<PDDL goal>"')
            executor_node.get_logger().info('Example: ros2 run task_planner action_executor "(on red_cube_0 blue_cube_0)"')
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        executor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


