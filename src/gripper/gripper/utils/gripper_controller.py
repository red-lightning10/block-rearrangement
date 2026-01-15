from rclpy.node import Node
import ctypes
import os
import threading
from gripper.utils.config_loader import GripperConfig


class GripperController:
    """Gripper control using position control mode"""
    
    def __init__(self, node: Node, config: GripperConfig = None):
        self.node = node
        self.logger = node.get_logger()
        self.config = config if config is not None else GripperConfig()
        self.lib = None
        # Lock to serialize access to C library (prevent segfaults from concurrent calls)
        self._lib_lock = threading.Lock()
        
        self._load_library()
    def _load_library(self):
        """Load and initialize the gripper library"""
        try:
            lib_path = self.config.get_library_path()
            if not os.path.exists(lib_path):
                self.logger.error(f'Gripper library not found at: {lib_path}')
                return
            
            self.logger.info(f'Loading gripper library from: {lib_path}')
            self.lib = ctypes.CDLL(lib_path)
            
            # Configure function signatures
            self.lib.openPort.restype = ctypes.c_bool
            self.lib.closePort.restype = None
            self.lib.enableTorque.argtypes = [ctypes.c_bool]
            self.lib.setOperatingMode.argtypes = [ctypes.c_int]
            self.lib.setGoalPosition.argtypes = [ctypes.c_int]
            self.lib.setGoalCurrent.argtypes = [ctypes.c_int]
            self.lib.readPosition.restype = ctypes.c_int
            self.lib.readCurrent.restype = ctypes.c_int16
            
            # Initialize gripper
            if self.lib.openPort():
                self.lib.setOperatingMode(self.config.position_control_mode)
                self.lib.enableTorque(True)
                self.logger.info('Gripper initialized successfully')
            else:
                self.logger.error('Failed to open gripper port')
                self.lib = None
                
        except Exception as e:
            self.logger.error(f'Failed to load gripper library: {e}')
            import traceback
            self.logger.error(traceback.format_exc())
            self.lib = None
    
    def open(self) -> bool:
        """Open gripper to fully open position"""
        if not self.lib:
            return False
        self.lib.setGoalCurrent(self.config.current_limit_ma)
        return self.set_position(self.config.min_position)
    
    def close(self) -> bool:
        """Close gripper to fully closed position"""
        if not self.lib:
            return False
        self.lib.setGoalCurrent(self.config.current_limit_ma)
        return self.set_position(self.config.max_position)
    
    def set_position(self, position: int) -> bool:
        """
        Set gripper to a specific position
        
        Args:
            position: Target position (min_position to max_position, where min=open, max=closed)
        
        Returns:
            bool: Success status
        """
        if self.lib is None:
            self.logger.error('Gripper library not loaded')
            return False
        
        # Clamp position to valid range
        position = max(self.config.min_position, min(self.config.max_position, position))
        
        # Serialize access to C library to prevent segfaults from concurrent calls
        with self._lib_lock:
            try:
                # Set operating mode and goal position
                self.lib.setOperatingMode(self.config.position_control_mode)
                self.lib.setGoalPosition(position)
                self.logger.info(f'Set gripper position to {position}')
                return True
            except AttributeError as e:
                self.logger.error(f'Gripper library function missing: {e}')
                return False
            except OSError as e:
                self.logger.error(f'OS error calling gripper library: {e}')
                return False
            except Exception as e:
                self.logger.error(f'Error setting gripper position: {e}')
                import traceback
                self.logger.error(traceback.format_exc())
                return False
    
    def get_position(self) -> float:
        """
        Get current gripper encoder position.
        Returns -1.0 on error or if the library is unavailable.
        """
        if self.lib is None:
            return -1.0
        # Serialize access to C library to prevent segfaults from concurrent calls
        with self._lib_lock:
            try:
                raw = self.lib.readPosition()
                return float(raw)
            except Exception as exc:
                self.logger.error(f'Error reading gripper position: {exc}')
                return -1.0

    def get_current_ma(self) -> float:
        """
        Read the motor current in milliamps. Returns -1.0 on error.
        """
        if self.lib is None:
            return -1.0
        # Serialize access to C library to prevent segfaults from concurrent calls
        with self._lib_lock:
            try:
                raw = int(self.lib.readCurrent())
                return float(raw) * self.config.current_multiplier
            except Exception as exc:
                self.logger.error(f'Error reading gripper current: {exc}')
                return -1.0
    
    def is_gripping(self) -> bool:
        """
        Check if gripper is holding an object based on current reading.
        
        Returns:
            bool: True if current indicates contact (above threshold).
        """
        if self.lib is None:
            return False
        
        try:
            position = self.get_position()
            if position is None or position < 0:
                return False

            # Quick open guard to avoid false positives when fully open
            open_threshold = self.config.min_position + self.config.open_guard_threshold
            if position <= open_threshold:
                return False

            current_ma = self.get_current_ma()
            if current_ma < 0:
                return False
            
            threshold = self.config.current_limit_ma * self.config.holding_threshold_ratio
            return current_ma >= threshold
        except Exception as e:
            self.logger.error(f'Error checking grip status: {e}')
            return False
    
    def cleanup(self):
        """Cleanup gripper resources"""
        if self.lib is not None:
            try:
                self.lib.enableTorque(False)
                self.lib.closePort()
                self.logger.info('Gripper cleaned up')
            except Exception as e:
                self.logger.error(f'Error during cleanup: {e}')

