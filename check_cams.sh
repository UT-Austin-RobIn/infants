#!/bin/bash

rostopic hz /cam_R/color/image_raw /cam_L/color/image_raw /cam_R/aligned_depth_to_color/image_raw /cam_L/aligned_depth_to_color/image_raw /cam_M/color/image_raw /cam_M/aligned_depth_to_color/image_raw 
# rostopic hz /cam_L/color/image_raw /cam_L/aligned_depth_to_color/image_raw 
# rostopic hz /cam_L/color/image_raw /cam_L/aligned_depth_to_color/image_raw /cam_M/color/image_raw /cam_M/aligned_depth_to_color/image_raw 
