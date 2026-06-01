import cv2
import numpy as np
import matplotlib.pyplot as plt

# --- Physical Setup Parameters ---
H = 125.0       # Distance from camera optical center down to the laser (in mm)
THETA_DEG = 35.0 # Angle the camera is pitched DOWN towards the laser (in degrees)

def load_calibration(filename="camera_calibration.yaml"):
    """Loads the intrinsic camera parameters from the saved file."""
    cv_file = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_coeffs = cv_file.getNode("distortion_coefficients").mat()
    cv_file.release()
    return camera_matrix, dist_coeffs

def get_laser_points_3d(frame, camera_matrix, dist_coeffs, H, theta_deg):
    """
    Finds the red laser line in an image and calculates its X, Y coordinates in mm.
    X-axis: Distance away from the camera.
    Y-axis: Horizontal distance relative to the camera center.
    """
    # 1. Isolate the Red Laser Line using HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Red hue wraps around the 180 mark in OpenCV
    mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
    mask2 = cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    # Mask the grayscale image to look only at red pixels
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    laser_img = cv2.bitwise_and(gray, gray, mask=red_mask)
    
    u_coords = []
    v_coords = []
    
    # 2. Extract the centerline of the laser (1 pixel per column for accuracy)
    for u in range(laser_img.shape[1]):
        col = laser_img[:, u]
        if np.max(col) > 50:  # Threshold to ignore noise
            v = np.argmax(col) # Find the brightest pixel in this column
            u_coords.append(u)
            v_coords.append(v)
            
    if len(u_coords) == 0:
        return np.array([]), np.array([]) # No laser found

    # Convert to format required by cv2.undistortPoints: shape (N, 1, 2)
    pts_2d = np.array([u_coords, v_coords], dtype=np.float32).T.reshape(-1, 1, 2)
    
    # 3. Undistort into normalized camera coordinates (removes fx, fy, cx, cy)
    # This automatically gives us x_n and y_n rays at Z=1
    undistorted = cv2.undistortPoints(pts_2d, camera_matrix, dist_coeffs)
    x_n = undistorted[:, 0, 0]
    y_n = undistorted[:, 0, 1]
    
    # 4. Perform Triangulation Math (Vectorized for speed)
    theta_rad = np.radians(theta_deg)
    
    # Rotate the rays to account for the camera pitching downwards
    v_y = y_n * np.cos(theta_rad) + np.sin(theta_rad)
    v_z = -y_n * np.sin(theta_rad) + np.cos(theta_rad)
    v_x = x_n
    
    # Calculate scale factor to intersect the horizontal laser plane at distance H
    # Note: We use np.clip to prevent division by zero if a ray is perfectly horizontal
    s = H / np.clip(v_y, 1e-6, None) 
    
    # 5. Map to User Coordinate System
    X_user = s * v_z # Depth away from camera
    Y_user = s * v_x # Horizontal axis
    
    return X_user, Y_user

# ==========================================
# Example Usage
# ==========================================
if __name__ == "__main__":
    # Load your matrices
    cam_matrix, dist = load_calibration("./Calibration/camera_calibration.yaml")
    
    # Load a test frame (or grab from cv2.VideoCapture(0))
    frame = cv2.imread("test2.jpg")
    
    if frame is not None:
        X_mm, Y_mm = get_laser_points_3d(frame, cam_matrix, dist, H, THETA_DEG)
        
        if len(X_mm) > 0:
            print(f"Successfully triangulated {len(X_mm)} points.")
            print(f"Centerpoint of laser is at:")
            print(f"Depth (X): {X_mm[len(X_mm)//2]:.2f} mm")
            print(f"Horizontal (Y): {Y_mm[len(Y_mm)//2]:.2f} mm")
        else:
            print("No red laser detected in the image.")

        print(f"X_mm: \"{X_mm}\"")
        print(f"Y_mm: \"{Y_mm}\"")
        plt.plot(Y_mm, X_mm)
        plt.show()