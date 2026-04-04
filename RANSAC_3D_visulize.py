import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
# Load the point cloud from .pcd file
pcd = o3d.io.read_point_cloud("point_cloud2.pcd")  # Replace with your file path

# Convert to numpy array
points = np.asarray(pcd.points)

# Scatter plot to visualize
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 2], points[:, 1], s=1, alpha=0.5)
ax.set_title("Original Point Cloud")
plt.show()

# Apply RANSAC to detect planes
plane_model, inliers = pcd.segment_plane(distance_threshold=0.2, 
                                         ransac_n=3, 
                                         num_iterations=1000)

# Extract plane equation
[a, b, c, d] = plane_model  # Ax + By + Cz + D = 0

print(f"Detected Plane Equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# Color the inlier (plane) points green and outliers blue
#inliers.paint_uniform_color([0, 1, 0])  # Green
#outlier.paint_uniform_color([0, 0, 1])  # Blue

# Separate inliers (plane points) and outliers
inlier_cloud = pcd.select_by_index(inliers)  # Points belonging to the plane
outlier_cloud = pcd.select_by_index(inliers, invert=True)  # Remaining points


# Compute angle w.r.t Z-axis (vertical plane)
normal_vector = np.array([a, b, c])
z_axis = np.array([0, 0, 1])  # Z-axis reference

# Compute angle using dot product formula
cos_theta = np.dot(normal_vector, z_axis) / (np.linalg.norm(normal_vector) * np.linalg.norm(z_axis))
angle_deg = np.degrees(np.arccos(cos_theta))

print(f"Plane Tilt Angle: {angle_deg:.2f}°")

# Check if the tilt is within 13 degrees
if angle_deg <= 13:
    print(" Plane is within the 13-degree limit.")
else:
    print(" Plane exceeds the 13-degree limit. Ignoring it.")

# Convert Open3D point clouds to NumPy for plotting
plane_points = np.asarray(inlier_cloud.points)
outlier_points = np.asarray(outlier_cloud.points)

# 3D Visualization
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot the detected plane (green)
ax.scatter(plane_points[:, 0], plane_points[:, 1], plane_points[:, 2], 
           color="green", label="Detected Plane (Inliers)", s=2)

# Plot the outliers (blue)
ax.scatter(outlier_points[:, 0], outlier_points[:, 1], outlier_points[:, 2], 
           color="blue", label="Outliers", s=2, alpha=0.3)

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
ax.set_title("Plane Detection with RANSAC")
ax.legend()
plt.show()

planes = []
while len(outlier_cloud.points) > 100:  # Stop when too few points remain
    plane_model, inliers = outlier_cloud.segment_plane(distance_threshold=0.2, 
                                                       ransac_n=3, 
                                                       num_iterations=1000)
    a, b, c, d = plane_model
    normal_vector = np.array([a, b, c])
    cos_theta = np.dot(normal_vector, z_axis) / (np.linalg.norm(normal_vector) * np.linalg.norm(z_axis))
    angle_deg = np.degrees(np.arccos(cos_theta))

    if angle_deg <= 13:
        print(f" Found plane within 13°: {angle_deg:.2f}°")
        planes.append((plane_model, inliers))

    # Remove detected plane points and continue
    outlier_cloud = outlier_cloud.select_by_index(inliers, invert=True)

# Return the best fitted plane
if planes:
    best_plane = planes[0]  # You can refine this selection
    print("Best plane equation:", best_plane[0])
else:
    print(" No plane within 13° found.")

    # Get the best plane
if planes:
    best_model, best_inliers = planes[0]
    best_plane_cloud = pcd.select_by_index(best_inliers)

    # 2D projection: compute oriented bounding box
    obb = best_plane_cloud.get_oriented_bounding_box()

    # Get box properties
    center = obb.center
    extent = obb.extent  # [width, height, thickness] depending on orientation

    # Sort the extents so we know which are the largest two
    sorted_extent = np.sort(extent)[-2:]  # Largest two for "square" check

    width, height = sorted_extent
    area = width * height
    square_ratio = min(width, height) / max(width, height)

    print(f"\n Bounding Box Extents: {extent}")
    print(f" Approx. Area: {area:.2f}")
    print(f" Center of Safe Spot: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")

    # Check if it's roughly square (allow 20% shape tolerance)
    if square_ratio >= 1.0:
        print(" This plane is square-like and can be considered a safe landing spot.")
    else:
        print(" This plane is not square enough — skipping.")
else:
    print(" No suitable plane found.")