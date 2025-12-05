import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import time
import matplotlib.pyplot as plt

import rosbag
from sensor_msgs.msg import Image, CameraInfo
import bisect

import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

import numpy as np
np.set_printoptions(suppress=True, precision=4)
import open3d as o3d
from sensor_msgs.msg import CameraInfo

def create_cloud_msg(points, frame_id="camera_link"):
    """
    points: Nx6 array [x, y, z, r, g, b] OR Nx3 array [x, y, z]
    """
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    if points.shape[1] == 6:
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('r', 12, PointField.FLOAT32, 1),
            PointField('g', 16, PointField.FLOAT32, 1),
            PointField('b', 20, PointField.FLOAT32, 1),
        ]
    else:
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

    cloud_msg = pc2.create_cloud(header, fields, points)
    return cloud_msg

def show_pointcloud_with_point(pcd, point_xyz, point_color=[1, 0, 0], sphere_radius=0.03):
    """
    Display a point cloud with a single highlighted 3D point (e.g., in red).

    Args:
        pcd (o3d.geometry.PointCloud): The original point cloud.
        point_xyz (list or np.ndarray): 3D coordinates of the point to highlight [x, y, z].
        point_color (list): RGB color of the point (default red).
        sphere_radius (float): Radius of the sphere used to show the point.
    """
    # Create a small sphere at the point location for visibility
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
    sphere.translate(point_xyz)
    sphere.paint_uniform_color(point_color)

    # Optional: add coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    # Visualize together
    o3d.visualization.draw_geometries([pcd, sphere, coord_frame],
                                      window_name="PointCloud with Highlighted Point")

def get_colored_pointcloud(color_img, depth_img, cam_info: CameraInfo, depth_scale=1000.0):
    """
    Display a colored point cloud using Open3D.
    
    Args:
        color_img (np.ndarray): Color image (BGR from OpenCV).
        depth_img (np.ndarray): Depth image (uint16 or float32).
        cam_info (CameraInfo): CameraInfo message with intrinsic parameters.
        depth_scale (float): Depth scale (e.g., 1000.0 if depth is in millimeters).
    """
    # Convert BGR (OpenCV) to RGB (Open3D)
    color_img_rgb = color_img[:, :, ::-1].copy()

    # breakpoint()

    # Normalize depth if it's in uint16 (e.g., millimeters)
    if depth_img.dtype == np.uint16:
        depth_img = depth_img.astype(np.float32) / depth_scale

    # Create Open3D images
    color_o3d = o3d.geometry.Image(color_img_rgb)
    depth_o3d = o3d.geometry.Image(depth_img)

    # Camera intrinsics from CameraInfo
    width = cam_info.width
    height = cam_info.height
    fx = cam_info.K[0]
    fy = cam_info.K[4]
    cx = cam_info.K[2]
    cy = cam_info.K[5]

    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

    # Create RGBD image
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1.0,  # Already normalized
        convert_rgb_to_intensity=False
    )

    # Create and visualize point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        intrinsic
    )

    # # Flip it for correct orientation
    # pcd.transform([[1, 0, 0, 0],
    #                [0, -1, 0, 0],
    #                [0, 0, -1, 0],
    #                [0, 0, 0, 1]])

    # Visualize
    # o3d.visualization.draw_geometries([pcd], window_name="Colored Point Cloud")
    # o3d.visualization.draw_geometries_with_editing([pcd])
    
    # Add camera coordinate frame at origin
    # geometries = [pcd]
    # show_camera_frame = True
    # if show_camera_frame:
    #     cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    #     cam_frame.transform(flip)  # Align frame with flipped point cloud
    #     geometries.append(cam_frame)

    # o3d.visualization.draw_geometries(geometries, window_name="Colored Point Cloud with Camera Frame")

    return pcd


def get_color_image_and_info():
    rospy.init_node('simple_image_listener', anonymous=True)
    bridge = CvBridge()
    
    image_msg = rospy.wait_for_message("/cam_L/color/image_raw", Image)
    depth_msg = rospy.wait_for_message('/cam_L/aligned_depth_to_color/image_raw', Image)
    info_msg = rospy.wait_for_message("/cam_L/color/camera_info", CameraInfo)
    
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    cv_depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    # cv_depth = None
    
    return cv_image, cv_depth, info_msg

# Function to find closest message
def find_closest_msg(times, msgs, target_time):
    idx = bisect.bisect_left(times, target_time)
    if idx == 0:
        return msgs[0], 0
    if idx >= len(times):
        return msgs[-1], len(times) - 1
    before = times[idx - 1]
    after = times[idx]
    if abs(before - target_time) <= abs(after - target_time):
        return msgs[idx - 1], idx - 1
    else:
        return msgs[idx], idx

def toggle_pause(vis):
    global paused
    paused = not paused
    print("⏸️ Paused" if paused else "▶️ Playing")
    return False

def next_frame(vis):
    global current_idx
    current_idx = (current_idx + 1) % len(pointclouds)
    update_pointcloud(pcd_vis, pointclouds[current_idx])
    vis.update_geometry(pcd_vis)
    return False

def update_pointcloud(target_pcd, source_pcd):
    target_pcd.points = source_pcd.points
    if source_pcd.has_colors():
        target_pcd.colors = source_pcd.colors


rospy.init_node("pointcloud_bag_writer", anonymous=True)
# Topics
depth_topic = "/cam_L/aligned_depth_to_color/image_raw"
color_topic = "/cam_L/color/image_raw"
info_topic  = "/cam_L/color/camera_info"

# Load all messages with their timestamps
bag = rosbag.Bag("/home/robotlearning2/infants/data/0/trial_007/trial_ros.bag")

color_msgs = []
color_times = []
depth_msgs = []
depth_times = []
info_msgs = []
info_times = []

for topic, msg, t in bag.read_messages(topics=[depth_topic, color_topic, info_topic]):
    t_sec = t.to_sec()
    if topic == color_topic:
        color_msgs.append(msg)
        color_times.append(t_sec)
    elif topic == depth_topic:
        depth_msgs.append(msg)
        depth_times.append(t_sec)
    elif topic == info_topic:
        info_msgs.append(msg)
        info_times.append(t_sec)

bag.close()

bag = rosbag.Bag('pointclouds.bag', 'w')
matched_depth_msgs, matched_color_msgs, matched_info_msgs = [], [], []
bridge = CvBridge()
pointclouds = []
# Loop over depth frames and get closest color + info
for i, (depth_msg, depth_time) in enumerate(zip(depth_msgs, depth_times)):
    matched_depth_msgs.append(depth_msg)
    print(f"Processing depth frame {i}")
    color_msg, color_idx = find_closest_msg(color_times, color_msgs, depth_time)
    matched_color_msgs.append(color_msg)
    info_msg, info_idx  = find_closest_msg(info_times, info_msgs, depth_time)
    matched_info_msgs.append(info_msg)
    print("indicies (depth, color, info): ", i, color_idx, info_idx)
    # breakpoint()

    cv_image = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
    cv_depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

    pcd = get_colored_pointcloud(cv_image, cv_depth, info_msg)
    # pcd_down = pcd.voxel_down_sample(voxel_size=0.05)
    # print("points: ", len(pcd.points), len(pcd_down.points))
    # pointclouds.append(pcd_down)
    # show_pointcloud_with_point(pcd, [0, 0, 0])
    # breakpoint()

    # Save pointcloud to bag
    cloud_np = np.asarray(pcd.points)
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)  # shape: (N, 3), floats in [0,1]
        colors = (colors * 255).astype(np.float32)  # convert to 0-255 if desired
        cloud_np = np.hstack((np.asarray(pcd.points), colors))  # shape: (N, 6)

    msg = create_cloud_msg(cloud_np, frame_id="cam_L_color_optical_frame")
    timestamp = rospy.Time.from_sec(depth_msg.header.stamp.to_sec())
    bag.write("/pointcloud", msg, t=timestamp)

breakpoint()
new_bag = rosbag.Bag("synced_data.bag", "w")
for depth_msg, color_msg, info_msg in zip(matched_depth_msgs, matched_color_msgs, matched_info_msgs):
    # pick one timestamp for all three (e.g. depth timestamp)
    t = rospy.Time.from_sec(depth_msg.header.stamp.to_sec())

    depth_msg.header.stamp = t
    color_msg.header.stamp = t
    info_msg.header.stamp = t

    new_bag.write("/cam_L/aligned_depth_to_color/image_raw", depth_msg, t)
    new_bag.write("/cam_L/color/image_raw", color_msg, t)
    new_bag.write("/cam_L/color/camera_info", info_msg, t)
new_bag.close()

breakpoint()
current_idx = 0
paused = False
frame_delay = 0.1  # seconds per frame

vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window("PointCloud Player (update_geometry)")
# Add a *single* PointCloud geometry to the visualizer
pcd_vis = o3d.geometry.PointCloud()
pcd_vis.points = pointclouds[0].points
if pointclouds[0].has_colors():
    pcd_vis.colors = pointclouds[0].colors
vis.add_geometry(pcd_vis)

# for pc in pointclouds:
#     pc_vis.points = pc.points
#     vis.update_geometry(pc_vis)
#     vis.poll_events()
#     vis.update_renderer()

# Register keys
vis.register_key_callback(ord(" "), toggle_pause)  # SPACE = pause/play
vis.register_key_callback(ord("N"), next_frame)    # N = step manually

# --- Playback loop ---
while True:
    if not paused:
        current_idx = (current_idx + 1) % len(pointclouds)  # loop automatically
        update_pointcloud(pcd_vis, pointclouds[current_idx])
        vis.update_geometry(pcd_vis)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(frame_delay)


    