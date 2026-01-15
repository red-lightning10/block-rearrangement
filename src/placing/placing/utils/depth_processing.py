"""
Depth processing utilities for placement pipeline.
Handles depth normalization and preprocessing.
"""
import numpy as np
from scipy.stats import tmean


def normalize_depth(depth_map, k=0.1):
    """
    Normalize a raw depth map into a scaled "height contrast" map relative to the
    deeper parts of the scene.

    Inputs:
        - depth_map (HxW): raw depth values (in mm)
        - k (float): scaling factor to control contrast of the normalized depth
        - debug (bool): if True, display a heatmap of the normalized depth

    Returns:
        - normal_depth (float32 HxW): normalized depth map, where larger positive values
          correspond to points that are closer than the trimmed background depth
          (based on the upper 25% of depth values).
    """
    trim = 0.90
    cutt_off = int(len(depth_map) * trim)
    sorted_depth = np.sort(depth_map.ravel())
    
    trim_mean = tmean(sorted_depth[cutt_off:])

    normal_depth = np.asarray(k * (trim_mean - depth_map))
    return normal_depth

