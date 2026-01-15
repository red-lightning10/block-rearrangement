import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gripper.utils.joint_state_utils import merge_joint_states
from gripper.utils.config_loader import GripperConfig


class JointStateMerger(Node):
    """Merge joint states from UR arm and gripper into single topic"""

    def __init__(self, config: GripperConfig = None):
        super().__init__('joint_state_merger')

        self.config = config if config is not None else GripperConfig()
        self.ur_joint_states = None
        self.gripper_joint_states = None
        self._gripper_joint_set = set(self.config.gripper_joint_names)

        self.ur_sub = self.create_subscription(JointState, self.config.ur_joint_states_topic, 
                                                self.ur_callback, self.config.queue_size)
        self.gripper_sub = self.create_subscription(JointState, self.config.gripper_joint_states_topic, 
                                                self.gripper_callback, self.config.queue_size)

        # Overwrites the UR driver's /joint_states with merged (arm + gripper) states
        # RSP subscribes to /joint_states and needs all joints to publish transforms
        self.merged_pub = self.create_publisher(JointState, self.config.merged_joint_states_topic, self.config.queue_size)
        self.get_logger().info('Joint state merger started')

    def _is_ur_only_message(self, msg: JointState) -> bool:
        """Check if message contains only UR arm joints (no gripper joints)"""
        msg_joint_set = set(msg.name)
        return not msg_joint_set.intersection(self._gripper_joint_set)

    def ur_callback(self, msg):
        """Store UR arm joint states and publish merged state if valid"""
        if self._is_ur_only_message(msg):
            self.ur_joint_states = msg
            self._try_publish_merged()

    def gripper_callback(self, msg):
        """Store gripper joint states and publish merged state if UR states available"""
        self.gripper_joint_states = msg
        self._try_publish_merged()

    def _try_publish_merged(self):
        """Publish merged joint states if UR states are available"""
        if self.ur_joint_states is None:
            return

        merged = merge_joint_states(self.ur_joint_states, self.gripper_joint_states)
        self.merged_pub.publish(merged)

def main(args=None):
    rclpy.init(args=args)
    merger = JointStateMerger()
    try:
        rclpy.spin(merger)
    except KeyboardInterrupt:
        pass
    finally:
        merger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

