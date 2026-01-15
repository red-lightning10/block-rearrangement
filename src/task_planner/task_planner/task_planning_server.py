import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
from interfaces.srv import GetAction
from task_planner.utils.tamp_interface import TAMPInterface
import yaml
import os
import tempfile
from rclpy.executors import MultiThreadedExecutor


class TAMPPlanningServer(Node):
    def __init__(self):
        super().__init__('tamp_planning_server')

        self.check_pddl_file()

        self.callback_group = ReentrantCallbackGroup()

        self.service = self.create_service(
            GetAction, 
            '/get_action', 
            self.next_action_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('TAMP Planning Server started')
    
    def check_pddl_file(self):
        """Get PDDL directory and check if domain file is valid."""
        package_dir = get_package_share_directory('task_planner')
        config_path = os.path.join(package_dir, 'config', 'config.yaml')
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                domain_name = config['domain']['file']
        except FileNotFoundError as e:
            raise RuntimeError(f'Config file not found: {config_path}') from e
        except (yaml.YAMLError, KeyError, TypeError) as e:
            raise RuntimeError(f'Invalid config: {config_path}') from e

        self.domain_file = os.path.join(package_dir, 'pddl', domain_name)
        if not os.path.exists(self.domain_file):
            self.get_logger().error(f'Domain file not found at {self.domain_file}')

    def next_action_callback(self, request, response):
        if not request.goal:
            response.success = False
            response.message = "No goal provided"
            return response
        
        goal = request.goal
        grounded_states = request.grounded_predicates
        
        # Create temporary task file for the pddl solvers input
        with tempfile.NamedTemporaryFile(mode='w', suffix='.pddl', delete=False) as f:
            task_filename = f.name
        
        try:
            solution = TAMPInterface.plan_from_grounded(
                self.domain_file,
                grounded_states,
                task_filename,
                goal
            )

            if solution is None:
                self.get_logger().error("Planner returned None - no solution found")
                response.success = False
                response.message = "No plan found for the given goal"
                return response

            parsed = TAMPInterface.parse_plan_ops(solution)

            # TODO: Check if this is really the case
            if not parsed:
                self.get_logger().error("Planner returned empty solution - goal cannot be reached")
                response.success = False
                response.message = "Goal cannot be reached - no solution found"
                return response

            opname, args = parsed.pop(0)
            response.action = opname
            response.args = args
            response.success = True
            response.message = "Action from generated plan received and grounded"
            return response
        
        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")
            response.success = False
            response.message = f"Planning error: {str(e)}"
            return response
        
        finally:
            if os.path.exists(task_filename):
                os.unlink(task_filename)

def main(args=None):
    rclpy.init(args=args)
    node = TAMPPlanningServer()
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()