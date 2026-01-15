"""
Clustering utilities for separating table surfaces from obstacles.
Uses HDBSCAN for 3D point clustering.
"""
import numpy as np
import cv2
from hdbscan import HDBSCAN
from .depth_processing import normalize_depth


def hdbScan(depth_map, k=0.1):
    """
    Cluster the scene in (x, y, depth) space using HDBSCAN to separate the main
    support surface (table) from elevated obstacles.

    Inputs:
        - depth_map (HxW): depth array in mm (numpy array)
        - k (float): scaling factor used in normalize_depth() to control depth contrast

    Returns:
        - table_mask (float32 HxW): depth values (relative height) for the largest, 
          dominant surface cluster (assumed table)
        - obstacles_mask (list of float32 HxW): list of depth masks for the remaining 
          clusters, each in the same shape as the depth map
    """
  
    Z = depth_map.copy()
    
    Z = normalize_depth(Z, k)
    Y, X = np.indices(Z.shape)
    
    feats = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])

    clusterer = HDBSCAN(min_cluster_size=250, min_samples=150)
    clusterer.fit(feats)

    labels = clusterer.labels_
    labels = labels.reshape((len(depth_map), len(depth_map[0])))
    
    masks = []  # list of depth masks for each cluster and their size
    for lab in sorted(x for x in np.unique(labels) if x >= 0):
        binary_mask = (labels == lab).astype(np.uint8) * 255
        depth_mask = np.asanyarray((Z.copy() * (binary_mask/255)) / k)  # Remove height above table bias
        masks.append(depth_mask)


    sort_masks = sorted(masks, key=cv2.countNonZero, reverse=True)

    table_mask = np.asanyarray(sort_masks[0])

    # Handle case when there are no obstacles (only table detected)
    if len(sort_masks) < 2:
        return table_mask, []

    obstacles_mask = []

    # Build feature array for obstacle clustering (x, y, depth for all obstacle pixels)
    obstacles_feats = []
    for mask in sort_masks[1:]:
        obstacles_mask.append(mask)
        # Get pixel coordinates and depths for this obstacle mask
        mask_binary = (mask > 0)
        Y_obs, X_obs = np.where(mask_binary)
        Z_obs = mask[mask_binary]
        # Stack into feature array: [x, y, depth]
        feats_obs = np.column_stack([X_obs, Y_obs, Z_obs])
        obstacles_feats.append(feats_obs)
    
    # # Combine all obstacle features into single array
    # if len(obstacles_feats) > 0:
    #     all_obstacles_feats = np.vstack(obstacles_feats)
    #     cluster = HDBSCAN(min_cluster_size=5, min_samples=3)
    #     cluster.fit(all_obstacles_feats)
    # else:
    #     # No obstacles to cluster
    #     cluster = None
    #     all_obstacles_feats = np.array([]).reshape(0, 3)

    # if cluster is not None and len(all_obstacles_feats) > 0:
    #     labs = cluster.labels_
    #     numbLabs = max(labs) if len(labs) > 0 else 0
        
    #     # Reconstruct obstacle masks based on clustering labels
    #     # Note: This is a simplified version - the original code had issues
    #     # For now, we'll just return the obstacles_mask as-is since the clustering
    #     # result isn't being used in the return value anyway
    #     pass
    # else:
    #     labs = np.array([])
    #     numbLabs = 0

    # Note: obstacles variable is computed but not returned (preserving original behavior)
    # return np.asanyarray(sort_masks[0]), np.asanyarray(sort_masks[1:])
    return table_mask, obstacles_mask

