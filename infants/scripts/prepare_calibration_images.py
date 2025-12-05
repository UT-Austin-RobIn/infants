import cv2
import os
import shutil
from pathlib import Path

from misc_utils import *

# extract_iamges_from_ros2(topic_name="/cam_L/color/image_raw",
#                          rosbag_path="/home/robotlearning2/infants/data/0/trial_004/trial_ros.bag", 
#                          output_folder="rs_L_images")
# extract_iamges_from_ros2(topic_name="/cam_M/color/image_raw",
#                          rosbag_path="/home/robotlearning2/infants/data/0/trial_004/trial_ros.bag", 
#                          output_folder="rs_M_images")


# # 1. Extract all images from Qualisys video
extract_images_from_video(resize_frame=False)

# # 2. Extract images from RS camera starting from Qualisys start timestamp
# # TODO: User needs to set this for now!
# time_str = "2025-10-02, 11:24:01.606"
# extract_images_from_ros(time_str=time_str, rosbag_path="/home/robotlearning2/infants/data/0/trial_003/trial_ros.bag")

# # 3. Obtain the offset 
# # rs frame where phone blinks = rs_blink ; qualisys frame where phone blinks = qualisys_blink
# # if rs_blink < qualisys_blink: negative ; else positive
# rs_offset_to_qualisys = int(input("Enter the time synchronization offset"))
# # rs_offset_to_qualisys = -15

# # 4. Rewrite all images of RS or Qualisys with this new offset
# if rs_offset_to_qualisys < 0:
#     # update Qualisys images
#     clean_and_rename_images(folder_path="qualisys_camera_images", rs_offset_to_qualisys=abs(rs_offset_to_qualisys))
# elif rs_offset_to_qualisys > 0:
#     # update RS images
#     clean_and_rename_images(folder_path="rs_images", rs_offset_to_qualisys=abs(rs_offset_to_qualisys))


# # 5. Obtain the start and end frames from Qualisys. Transfer (start, end) from qualisys_images and (start, end) from rs_images 
# # to calibration folders
# # start_index = int(input("Enter start index"))
# # end_index = int(input("Enter end index"))
# start_index = 150     # inclusive, e.g., 1 for '0001.jpg'
# end_index = 1450      # inclusive, e.g., 50 for '0050.jpg'
# filename_digits = 4 # e.g., 0001 has 4 digits

# # source_folder = Path("/home/robotlearning2/infants/rs_images")      # /home/robotlearning2/infants
# source_folder = Path("/home/robotlearning2/infants/rs_L_images")      # /home/robotlearning2/infants
# destination_folder = Path("/home/robotlearning2/stereo-calib/dataset/left")  # e.g., Path("./output_images")
# transfer_images(source_folder, destination_folder, start_index, end_index)

# # source_folder = Path("/home/robotlearning2/infants/qualisys_camera_images")      # /home/robotlearning2/infants
# source_folder = Path("/home/robotlearning2/infants/rs_M_images")      # /home/robotlearning2/infants
# destination_folder = Path("/home/robotlearning2/stereo-calib/dataset/right")  # e.g., Path("./output_images")
# transfer_images(source_folder, destination_folder, start_index, end_index)
