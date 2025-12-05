import rospy
import cv2
import pytz
import rosbag
import bisect
import json
import yaml
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

from cv_bridge import CvBridge
from collections import defaultdict
from datetime import datetime

np.set_printoptions(suppress=True, precision=4)


def load_calibration_from_json(json_file_path):
    """Load K, R, T matrices from JSON calibration file"""
    with open(json_file_path, 'r') as f:
        data = json.load(f)
    
    # Extract left camera matrix (K1)
    K1 = np.array(data['left_camera_calibration_data']['camera_matrix'])
    
    # Extract right camera matrix (K2) - assuming similar structure
    K2 = np.array(data['right_camera_calibration_data']['camera_matrix'])
    
    # Extract distortion coefficients
    dist1 = np.array(data['left_camera_calibration_data']['dist_coeffs']).flatten()
    dist2 = np.array(data['right_camera_calibration_data']['dist_coeffs']).flatten()
    
    # Extract stereo calibration parameters (R and T)
    R = np.array(data['rot'])
    T = np.array(data['trans'])
    
    return K1, K2, dist1, dist2, R, T

def overlay_point_on_image(image, pixel, color=(0, 0, 255), radius=12, thickness=-1):
    """
    Draw a point on the image at given pixel coordinates.

    Args:
        image (np.ndarray): BGR image
        pixel (tuple or np.ndarray): (u, v) pixel coordinates
        color (tuple): BGR color
        radius (int): circle radius
        thickness (int): -1 fills the circle
    """
    img_out = image.copy()
    u, v = int(pixel[0]), int(pixel[1])
    cv2.circle(img_out, (u, v), radius, color, thickness)
    return img_out

def project_point_to_image(point_3d, K, dist_coeffs=None):
    """
    Project a 3D point (in camera frame) to 2D pixel coordinates.

    Args:
        point_3d (np.ndarray): shape (3,) or (N,3) -> 3D points in camera frame (X, Y, Z)
        K (np.ndarray): 3x3 intrinsic matrix [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        dist_coeffs (np.ndarray): optional, distortion coefficients (k1, k2, p1, p2, k3)

    Returns:
        np.ndarray: 2D pixel coordinates (u, v)
    """
    # Ensure correct shape
    points_3d = np.array(point_3d, dtype=np.float32).reshape(-1, 1, 3)
    
    # No rotation/translation since point is already in camera frame
    rvec = np.zeros((3, 1), dtype=np.float32)
    tvec = np.zeros((3, 1), dtype=np.float32)
    
    # breakpoint()
    # Use OpenCV’s projection function
    points_2d, _ = cv2.projectPoints(points_3d, rvec, tvec, K, dist_coeffs)
    return points_2d.reshape(-1, 2)

def find_closest_msg(times, msgs, target_time):
    idx = bisect.bisect_left(times, target_time)
    if idx == 0:
        return msgs[0], 0, times[0]
    if idx >= len(times):
        return msgs[-1], len(times) - 1, times[-1]
    before = times[idx - 1]
    after = times[idx]
    if abs(before - target_time) <= abs(after - target_time):
        return msgs[idx - 1], idx - 1, times[idx - 1]
    else:
        return msgs[idx], idx, times[idx]

# rospy.init_node("pointcloud_bag_writer", anonymous=True)

subject_id = "001"
trial_id = "010"
camera_name = "cam_L"
depth_topic = f"/{camera_name}/aligned_depth_to_color/image_raw"
color_topic = f"/{camera_name}/color/image_raw"
info_topic  = f"/{camera_name}/color/camera_info"

# Read relevant files
metadata_file = f"/home/robotlearning2/infants/data/{subject_id}/trial_{trial_id}/trial_metadata.yaml"
main_rosbag = rosbag.Bag(f"/home/robotlearning2/infants/data/{subject_id}/trial_{trial_id}/trial_ros.bag")
marker_rosbag = rosbag.Bag("/home/robotlearning2/infants/marker.bag")
K1, K2, dist1, dist2, R, T = load_calibration_from_json("/home/robotlearning2/stereo-calib/results/calibration_results.json")

# Set the intrinsic to what we obtain from ROS
cam_info = next(iter(main_rosbag.read_messages(topics=[info_topic]))).message
intrinsic = cam_info.K
intrinsic = np.array(intrinsic).reshape(3,3)

# ============================ Read RealSense color images ============================
color_msgs = []
color_times = []
time_diff_list = []
for i, (topic, msg, t) in enumerate(main_rosbag.read_messages(topics=[color_topic])):
    t_sec = t.to_sec()
    if i == 0:
        t_sec_prev = t_sec
    if topic == color_topic:
        color_msgs.append(msg)
        color_times.append(t_sec)

        # # This is for debugging
        # time_diff = t_sec - t_sec_prev
        # time_diff_list.append(time_diff)
        # t_sec_prev = t_sec

# plt.plot(time_diff_list)
# plt.show()
# breakpoint()

main_rosbag.close()
# ====================================================================================



# ============================ Read the marker positions ============================
marker_msgs = defaultdict(list)
marker_times = defaultdict(list)

num_markers = next(iter(marker_rosbag.read_messages(topics=["/metadata/num_markers"])), None).message.data
topics = [f"/marker_{i+1}" for i in range(num_markers)]
for topic, msg, t in marker_rosbag.read_messages(topics=topics): 
    t_sec = t.to_sec()
    marker_msgs[topic].append(msg)
    marker_times[topic].append(t_sec)
marker_rosbag.close()
breakpoint()
# ====================================================================================

# ============================ Visualize the marker positions on the image ============================
T_mc_wrt_mcR = np.array([
    [-0.952129, 0.032076, -0.304008, -207.692825 / 1000],
    [-0.281458, -0.480063, 0.830855, 1005.463013 / 1000],
    [-0.119293, 0.876647, 0.466110, 562.679993 / 1000],
    [0.0, 0.0, 0.0, 1.0]
]) 
# Account for image convention change
R_qualisys_img_to_rs_img = np.array([
    [1, 0, 0],
    [0, -1, 0],
    [0, 0, -1]
])

# TODO: use from the calibration csv later
R =  np.array([
    [ 0.784147061739405, 0.17962226509393947, -0.5940111341111212],
    [ -0.2951145340327227, 0.9499676303022973, -0.10231770707207441],
    [ 0.5459128111412177, 0.2555334484179329, 0.7979234671141918]])
T = np.array([[ 0.8211198894576127], [ 0.038764239053183214], [ 0.18265779769967244]])
trans_matrix = np.eye(4)
trans_matrix[:3, :3] = R
trans_matrix[:3, 3] = T.squeeze()

bridge = CvBridge()
marker_topic = "/marker_1"
with open(metadata_file, "r") as f:
    metadata = yaml.load(f, Loader=yaml.FullLoader)
if "ntp_offset" not in metadata:
        ntp_offset = 0.20
else:
    ntp_offset = metadata["ntp_offset"]

for i, (color_msg, color_time) in enumerate(zip(color_msgs, color_times)):
    color_time -= ntp_offset
    marker_msg, marker_idx, marker_time = find_closest_msg(marker_times[marker_topic], marker_msgs[marker_topic], color_time)
    time_diff = abs(color_time - marker_time)
    print("[DEBUG] time_diff: ", time_diff)
    # print("time_diff: ", time_diff)
    if time_diff > 0.01:
        continue

    print(marker_msg.point.x, marker_msg.point.y, marker_msg.point.z)
    point_wrt_mcR = np.array([
        marker_msg.point.x,
        marker_msg.point.y,
        marker_msg.point.z,
        1000.0
    ]) / 1000.0
    point_wrt_mc = np.linalg.inv(T_mc_wrt_mcR) @ point_wrt_mcR
    point_wrt_mc = R_qualisys_img_to_rs_img @ point_wrt_mc[:3]
    point_wrt_mc = np.append(point_wrt_mc, 1.0)
    point_wrt_rs = np.linalg.inv(trans_matrix) @ point_wrt_mc
    # print("point_wrt_rs: ", point_wrt_rs)

    img = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')

    pixel = project_point_to_image(point_wrt_rs[:3], intrinsic)
    if marker_msg.point.x == 0.0 and marker_msg.point.y == 0.0 and marker_msg.point.z == 0.0:
        img_overlay = img
    else:
        img_overlay = overlay_point_on_image(img, pixel[0], color=(0, 0, 255))

    # Show result
    cv2.imshow("Marker Overlay", img_overlay)
    key = cv2.waitKey(1)  # wait 1 ms for a key press, controls FPS
    if key == 27:  # press ESC to exit
        break
cv2.destroyAllWindows()
# ====================================================