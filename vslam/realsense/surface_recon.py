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

# Get camera intrinsics
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

# Create alignment primitive to align depth to color
align = rs.align(rs.stream.color)

# Define extrinsic transformation to correct orientation
# Rotates around X-axis to flip the point cloud
extrinsics = np.array([[1,  0,  0,  0],  
                       [0, -1,  0,  0],  # Flip Y-axis
                       [0,  0, -1,  0],  # Flip Z-axis
                       [0,  0,  0,  1]])  # Homogeneous transformation

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
        depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale  # Convert to meters
        color_image = np.asanyarray(color_frame.get_data())  # RGB frame

        # Get depth dimensions
        height, width = depth_image.shape

        # Generate pixel grid
        xx, yy = np.meshgrid(np.arange(width), np.arange(height))

        # Convert depth to 3D coordinates using camera intrinsics
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy

        # Compute real-world coordinates (X, Y, Z)
        Z = depth_image
        X = (xx - cx) * Z / fx
        Y = (yy - cy) * Z / fy

        # Stack into (N, 3) format for Open3D
        points_3d = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)

        # Apply filtering: Keep only points between **7cm (0.07m) and 50cm (0.50m)**
        mask = (Z.flatten() >= 0.07) & (Z.flatten() <= 0.5)
        points_3d = points_3d[mask]

        # Convert color frame to Open3D format
        colors = color_image.reshape(-1, 3) / 255.0  # Normalize to 0-1
        colors = colors[mask]  # Correctly map colors to valid depth points

        # Apply extrinsic transformation to correct orientation
        points_3d = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))  # Convert to homogeneous coordinates
        points_3d = (extrinsics @ points_3d.T).T[:, :3]  # Apply transformation

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Estimate normals (required for Poisson surface reconstruction)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        # Poisson surface reconstruction
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)

        # Save reconstructed mesh as PLY file
        o3d.io.write_triangle_mesh("vslam/realsense_surface_mesh.ply", mesh)

        # Show depth and color images
        stacked_images = np.hstack((color_image, cv2.applyColorMap(cv2.convertScaleAbs(depth_image * 255, alpha=0.03), cv2.COLORMAP_JET)))
        cv2.imshow("Color and Depth View", stacked_images)

        print("Reconstructed Surface Mesh Saved as realsense_surface_mesh.ply")

        # Show reconstructed surface in Open3D
        o3d.visualization.draw_geometries([mesh])

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
