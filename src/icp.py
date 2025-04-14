import numpy as np
from scipy.spatial import cKDTree

def icp(base_points, src_points, max_iterations=50, tolerance=1e-6):
    """
    Align src_points to base_points using Iterative Closest Point (ICP).
    
    Parameters:
        base_points (np.ndarray): Nx3 array of base (target) point cloud.
        src_points (np.ndarray): Mx3 array of source (to align) point cloud.
        max_iterations (int): Maximum number of ICP iterations.
        tolerance (float): Convergence tolerance.
    
    Returns:
        aligned_points (np.ndarray): Aligned source point cloud.
        final_transform (np.ndarray): 4x4 homogeneous transformation matrix.
    """
    
    def best_fit_transform(A, B):
        # Compute centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        
        # Center the points
        AA = A - centroid_A
        BB = B - centroid_B
        
        # Compute covariance matrix
        H = AA.T @ BB
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure a proper rotation (no reflection)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        t = centroid_B - R @ centroid_A
        
        # Homogeneous transformation
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    src_h = np.hstack((src_points, np.ones((src_points.shape[0], 1))))
    prev_error = float('inf')
    final_transform = np.eye(4)

    for i in range(max_iterations):
        # Find the nearest neighbors in base_points for each point in src_points
        tree = cKDTree(base_points)
        distances, indices = tree.query(src_h[:, :3])
        matched_base = base_points[indices]

        # Compute transformation
        T = best_fit_transform(src_h[:, :3], matched_base)
        
        # Apply transformation
        src_h = (T @ src_h.T).T
        
        # Accumulate transformation
        final_transform = T @ final_transform

        # Check convergence
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    aligned_points = src_h[:, :3]
    return aligned_points, final_transform
