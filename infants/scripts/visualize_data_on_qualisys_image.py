import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
import rosbag
from collections import defaultdict
import bisect
import json
import os
import csv

import numpy as np
np.set_printoptions(suppress=True, precision=4)
import open3d as o3d
from sensor_msgs.msg import CameraInfo
from misc_utils import *

import pytz
from datetime import datetime

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

# Function to find closest message
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


def main():
    
    # qualisys_video_name = "001_Miqus_2_31039.avi"
    # extract_images_from_video(video_path=qualisys_video_name, resize_frame=False)
    folder_path = "/home/robotlearning2/infants/qualisys_camera_images"
    image_paths = [f for f in os.listdir(folder_path) if f.endswith('.jpg')]
    image_paths.sort()
    
    # ================================ Read the marker positions for all markers ================================
    marker_rosbag = rosbag.Bag("marker.bag")
    marker_msgs = defaultdict(list)
    marker_times = defaultdict(list)
    num_markers = next(iter(marker_rosbag.read_messages(topics=["/metadata/num_markers"])), None).message.data
    num_markers = 2 
    topics = [f"/marker_{i+1}" for i in range(num_markers)]
    for i, (topic, msg, t) in enumerate(marker_rosbag.read_messages(topics=topics)): 
        t_sec = t.to_sec()
        if i == 0:
            start_timestamp = t_sec
        marker_msgs[topic].append(msg)
        print(msg.point.x)
        marker_times[topic].append(t_sec)
    marker_rosbag.close()
    # ============================================================================================================
    breakpoint()

    # T_mc_wrt_mcR = np.array([
    #     [-0.952129, 0.032076, -0.304008, -207.692825 / 1000],
    #     [-0.281458, -0.480063, 0.830855, 1005.463013 / 1000],
    #     [-0.119293, 0.876647, 0.466110, 562.679993 / 1000],
    #     [0.0, 0.0, 0.0, 1.0]
    # ])
    T_mc_wrt_mcR = np.array([
        [-0.995493, -0.043347, -0.084350, -10.093993 / 1000],
        [-0.057859, -0.427120, 0.902342, 1078.293457 / 1000],
        [-0.075142, 0.903156, 0.422687, 515.898926 / 1000],
        [0.0, 0.0, 0.0, 1.0]
    ])  
    # <transform x="-10.093993" y="1078.293457" z="515.898926" r11="-0.995493" r12="-0.057859" r13="-0.075142" r21="-0.043347" r22="-0.427120" r23="0.903156" r31="-0.084350" r32="0.902342" r33="0.422687"/>
    # Account for image convention change
    R_qualisys_img_to_rs_img = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])

    # Compute camera intrinsics. This assumes that the image is 1920x1088 
    scale_u = 1919 / 122879
    scale_v = 1087 / 69631
    fx_px = 107605.140625 * scale_u
    fy_px = 107598.804688 * scale_v
    cx_px = 62540.17578 * scale_u
    cy_px = 34756.410156 * scale_v
    K = np.array([
        [fx_px, 0, cx_px],
        [0, fy_px, cy_px],
        [0, 0, 1]
    ])

    marker_colors = [
        (0, 0, 255),    # Red
        (0, 255, 0),    # Green
        (255, 0, 0),    # Blue
        (0, 255, 255),  # Yellow
        (255, 0, 255)   # Magenta
    ]


    # TODO: Change this
    marker_topic = "/marker_2"
    # marker_topics = ["/marker_1", "/marker_2", "/marker_3", "/marker_4", "/marker_5"]
    marker_topics = ["/marker_2"]
    camera_fps = 30
    # for i, (img, color_time) in enumerate(zip(images, image_times)):
    for i, image_path in enumerate(image_paths):
        img = cv2.imread(os.path.join(folder_path, image_path))
        color_time = start_timestamp + i / camera_fps
        marker_msg, marker_idx, marker_time = find_closest_msg(marker_times[marker_topic], marker_msgs[marker_topic], color_time)
        time_diff = abs(color_time - marker_time)
        print("---", i, marker_idx, time_diff)
        # print("time_diff: ", time_diff)
        if time_diff > 0.01:
            continue
        # breakpoint()

        for i, marker_topic in enumerate(marker_topics):
            marker_msg = marker_msgs[marker_topic][marker_idx]
            marker_color = marker_colors[i % len(marker_colors)]

            # print(marker_msg.point.x, marker_msg.point.y, marker_msg.point.z)
            point_wrt_mcR = np.array([
                marker_msg.point.x,
                marker_msg.point.y,
                marker_msg.point.z,
                1000.0
            ]) / 1000.0
            point_wrt_mc = np.linalg.inv(T_mc_wrt_mcR) @ point_wrt_mcR
            # print("point_wrt_mc: ", point_wrt_mc)
            point_wrt_mc = R_qualisys_img_to_rs_img @ point_wrt_mc[:3]
            point_wrt_mc = np.append(point_wrt_mc, 1.0)

            pixel = project_point_to_image(point_wrt_mc[:3], K)
            if marker_msg.point.x == 0.0 and marker_msg.point.y == 0.0 and marker_msg.point.z == 0.0:
                img = img
            else:
                img = overlay_point_on_image(img, pixel[0], color=marker_color)

        # Show result
        cv2.imshow("Marker Overlay", img)
        key = cv2.waitKey(1)  # wait 1 ms for a key press, controls FPS
        if key == 27:  # press ESC to exit
            break
    cv2.destroyAllWindows()
    # ====================================================

if __name__ == "__main__":
    main()