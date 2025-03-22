import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2

# Initialize RealSense pipeline
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipe.start(config)

# Get depth sensor's scale (to convert depth values to meters)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Create alignment primitive to align depth to color
align = rs.align(rs.stream.color)

try:
    while True:
        # Wait for frames and align depth to color
        frames = pipe.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert depth to point cloud
        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)
        pc.map_to(color_frame)

        # Convert to numpy array
        vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3) * depth_scale
        colors = np.asanyarray(color_frame.get_data()).reshape(-1, 3) / 255.0

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vertices)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Save point cloud as PLY file
        o3d.io.write_point_cloud("realsense_3d_map.ply", pcd)

        # Show depth and color images
        stacked_images = np.hstack((color_image, cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)))
        cv2.imshow("Color and Depth View", stacked_images)

        print("3D Map Saved as realsense_3d_map.ply")
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
