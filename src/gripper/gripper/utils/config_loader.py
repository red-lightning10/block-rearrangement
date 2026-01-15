"""Configuration loader for gripper package"""

import yaml
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from typing import Dict, Any, List


class GripperConfig:
    """Load and provide access to gripper configuration"""
    
    def __init__(self, config_file: str = None):
        """
        Initialize configuration loader
        
        Args:
            config_file: Optional path to config file. If None, uses default.
        """
        if config_file is None:
            package_share = get_package_share_directory('gripper')
            config_file = os.path.join(package_share, 'config', 'gripper_config.yaml')
        
        self.config_file = config_file
        self._config: Dict[str, Any] = {}
        self._load_config()
    
    def _load_config(self):
        """Load configuration from YAML file"""
        if not os.path.exists(self.config_file):
            raise FileNotFoundError(f"Config file not found: {self.config_file}")
        
        with open(self.config_file, 'r') as f:
            self._config = yaml.safe_load(f)
    
    def get(self, key_path: str, default=None):
        """
        Get configuration value using dot notation (e.g., 'hardware.max_position')
        
        Args:
            key_path: Dot-separated path to config value
            default: Default value if key not found
        
        Returns:
            Configuration value or default
        """
        keys = key_path.split('.')
        value = self._config
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        
        return value
    
    def get_library_path(self) -> str:
        """Get absolute path to gripper library"""
        package_share = get_package_share_directory('gripper')
        relative_path_parts = self.get('library.relative_path', [])
        lib_path = os.path.join(package_share, *relative_path_parts)
        return os.path.abspath(lib_path)
    
    @property
    def min_position(self) -> int:
        return self.get('hardware.min_position', 0)
    
    @property
    def max_position(self) -> int:
        return self.get('hardware.max_position', 740)
    
    @property
    def max_position_rad(self) -> float:
        return self.get('hardware.max_position_rad', 1.12)
    
    @property
    def current_limit_ma(self) -> int:
        return self.get('hardware.current_limit_ma', 300)
    
    @property
    def current_multiplier(self) -> float:
        return self.get('hardware.current_multiplier', 4.02)
    
    @property
    def position_control_mode(self) -> int:
        return self.get('hardware.position_control_mode', 5)
    
    @property
    def gripper_joint_names(self) -> List[str]:
        return self.get('joints.names', ['rh_p12_rn', 'rh_r2', 'rh_l1', 'rh_l2'])
    
    @property
    def position_tolerance_rad(self) -> float:
        return self.get('action_server.position_tolerance_rad', 0.02)
    
    @property
    def position_tolerance_raw(self) -> int:
        return self.get('action_server.position_tolerance_raw', 5)
    
    @property
    def movement_timeout(self) -> float:
        return self.get('action_server.movement_timeout', 5.0)
    
    @property
    def feedback_rate(self) -> float:
        return self.get('action_server.feedback_rate', 10.0)
    
    @property
    def holding_sample_count(self) -> int:
        return self.get('action_server.holding_sample_count', 10)
    
    @property
    def holding_threshold_ratio(self) -> float:
        return self.get('action_server.holding_threshold_ratio', 0.9)
    
    @property
    def open_guard_threshold(self) -> int:
        return self.get('action_server.open_guard_threshold', 5)
    
    @property
    def stall_detection_threshold(self) -> int:
        return self.get('action_server.stall_detection_threshold', 2)
    
    @property
    def stall_count_limit(self) -> int:
        return self.get('action_server.stall_count_limit', 10)
    
    @property
    def ur_joint_states_topic(self) -> str:
        return self.get('joint_state_merger.ur_joint_states_topic', '/joint_states')
    
    @property
    def gripper_joint_states_topic(self) -> str:
        return self.get('joint_state_merger.gripper_joint_states_topic', '/gripper_joint_states')
    
    @property
    def merged_joint_states_topic(self) -> str:
        return self.get('joint_state_merger.merged_joint_states_topic', '/joint_states')
    
    @property
    def queue_size(self) -> int:
        return self.get('joint_state_merger.queue_size', 10)
    
    @property
    def publish_rate(self) -> float:
        return self.get('joint_state_publisher.publish_rate', 10.0)
    
    @property
    def is_holding_service(self) -> str:
        return self.get('services.is_holding_service', '/gripper/is_holding')
    
    @property
    def gripper_command_action(self) -> str:
        return self.get('actions.gripper_command_action', 'gripper_controller/gripper_cmd')

