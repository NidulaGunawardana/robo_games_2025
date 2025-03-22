import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# Create Open3D TSDF volume (for mapping)
volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=0.005,  # Higher resolution = smaller voxel size
    sdf_trunc=0.04,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
)

# Align depth with color
align = rs.align(rs.stream.color)

try:
    while True:
        # Get frames and align them
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert depth to numpy
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert frames to Open3D images
        depth_o3d = o3d.geometry.Image(depth_image)
        color_o3d = o3d.geometry.Image(color_image)

        # Create RGBD Image
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=False
        )

        # Camera Intrinsics (For D405)
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            intrinsics.width, intrinsics.height,
            intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy
        )

        # Integrate frame into TSDF volume (SLAM-based fusion)
        volume.integrate(rgbd, pinhole_camera_intrinsic, np.linalg.inv(np.eye(4)))

        # Display depth and color images
        stacked_images = np.hstack((color_image, cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)))
        cv2.imshow("SLAM Mapping - Color & Depth", stacked_images)

        # Press 'q' to stop mapping
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

    # Extract 3D model from TSDF
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    # Save and visualize the 3D Map
    o3d.io.write_triangle_mesh("slam_3d_map.ply", mesh)
    print("3D SLAM Map Saved as 'slam_3d_map.ply'")

    # Visualize 3D Map
    o3d.visualization.draw_geometries([mesh])
