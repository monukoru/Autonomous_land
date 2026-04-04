# This script generates the point cloud when intel camera is connected 
import pyrealsense2 as rs
import open3d as o3d
import os


# Start the RealSense camera
pipe = rs.pipeline()
pipe.start()

# Let auto-exposure settle, then grab a frame
for _ in range(20):
    frames = pipe.wait_for_frames()
    
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()

# Generate the point cloud and save temporarily as .ply
pc = rs.pointcloud()
pc.map_to(color_frame)
points = pc.calculate(depth_frame)
points.export_to_ply("temp.ply", color_frame)

# Stop the camera
pipe.stop()

# Convert the .ply to .pcd using Open3D
pcd = o3d.io.read_point_cloud("temp.ply")
o3d.io.write_point_cloud("my_scan4.pcd", pcd)

# Clean up the temporary file
os.remove("temp.ply")

print("Scan complete! Saved as my_scan.pcd")