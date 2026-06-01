import numpy as np
import cv2
import matplotlib.pyplot as plt

# ==========================================
# 1. LOAD USER CALIBRATION DATA
# ==========================================
# Populated directly from your camera_calibration.yaml file
CAMERA_MATRIX = np.array([
    [624.88167624,   0.00000000, 326.83244795],
    [  0.00000000, 624.74722893, 261.64460573],
    [  0.00000000,   0.00000000,   1.00000000]
], dtype=np.float64)

DIST_COEFFS = np.array([
    0.0346969598, -0.1819008495, 0.0021179821, 0.0004696657, 0.2123163120
], dtype=np.float64)


def extract_laser_line_centroids(image, threshold_value=120):
    """
    Extracts the laser line from an image using a sub-pixel center-of-mass 
    approach to robustly handle scattering on rough surfaces.
    """
    # Convert to grayscale if image is BGR
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
        
    # Apply thresholding to eliminate ambient background noise
    _, masked_gray = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_TOZERO)
    
    height, width = masked_gray.shape
    u_indices = []
    v_centroids = []
    
    # Iterate through each column to find the center of mass of the laser peak
    for u in range(width):
        column_intensity = masked_gray[:, u].astype(np.float64)
        sum_intensity = np.sum(column_intensity)
        
        # Only process columns where a definitive laser line segment exists
        if sum_intensity > 0:
            v_indices = np.arange(height)
            # Center of mass equation: Sum(v * Intensity) / Sum(Intensity)
            v_centroid = np.sum(v_indices * column_intensity) / sum_intensity
            
            u_indices.append(u)
            v_centroids.append(v_centroid)
            
    return np.column_stack((u_indices, v_centroids))


def transform_pixels_to_3d_cloud(points_uv, H, theta_deg, Z_height):
    """
    Transforms sub-pixel (u, v) points into a true 3D point cloud [X, Y, Z]
    using the calibrated camera intrinsics and geometric setup.
    """
    if len(points_uv) == 0:
        return np.empty((0, 3))
        
    # Deconstruct camera matrix parameters
    fx = CAMERA_MATRIX[0, 0]
    fy = CAMERA_MATRIX[1, 1]
    cx = CAMERA_MATRIX[0, 2]
    cy = CAMERA_MATRIX[1, 2]
    
    # Geometric pre-computations
    theta = np.radians(theta_deg)
    tan_theta = np.tan(theta)
    cos_theta = np.cos(theta)
    
    u = points_uv[:, 0]
    v = points_uv[:, 1]
    v_diff = v - cy
    
    # Shared triangulation denominator
    denominator = (fy * tan_theta) + v_diff
    denominator = np.where(denominator == 0, 1e-6, denominator)  # Avoid zero division
    
    # 3D Closed-form solutions incorporating fx/fy ratio scaling
    x = H * (fy - (v_diff * tan_theta)) / denominator
    y = H * (fy / fx) * (cx - u) / (cos_theta * denominator)
    
    # Z is tied strictly to your current known rig altitude
    z = np.full_like(x, Z_height)
    
    return np.column_stack((x, y, z))


# ==========================================
# 2. RUNTIME EXECUTION PIPELINE
# ==========================================
def process_scanner_frame(frame_path, H_distance, theta_angle, current_Z):
    """
    Main pipeline function: Reads image -> Undistorts -> Extracts Line -> Outputs 3D Points
    """
    # Load frame
    frame = cv2.imread(frame_path)
    if frame is None:
        raise FileNotFoundError(f"Could not load image from path: {frame_path}")
        
    # Step A: Remove lens distortion using your YAML coefficients
    undistorted = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS)
    
    # Step B: Robust laser profile tracking for rough targets
    laser_pixels = extract_laser_line_centroids(undistorted, threshold_value=100)
    
    # Step C: Project to 3D physical coordinate space
    point_cloud_3d = transform_pixels_to_3d_cloud(laser_pixels, H_distance, theta_angle, current_Z)
    
    return point_cloud_3d


# Example Usage configuration:
if __name__ == "__main__":
    # Hypothetical dimensions: setup is 120mm above laser, tilted down 30 degrees, 
    # and the rig is currently 450mm off the floor baseline.
    H_setup = 125.0       
    theta_setup = 35.0    
    current_rig_Z = 0
    
    
    
    try:
        # Generate point cloud for a single snapshot profile
        pc_profile = process_scanner_frame("test4.jpg", H_setup, theta_setup, current_rig_Z)
        print(f"Successfully generated {pc_profile.shape[0]} 3D points from rough surface.")
        print("Sample point array (X, Y, Z):\n", pc_profile[:5])
        
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        scatter = ax.scatter(pc_profile[:,0],pc_profile[:,1],pc_profile[:,2])
        plt.show()


    except FileNotFoundError:
        print(f"Failed to find {0}")