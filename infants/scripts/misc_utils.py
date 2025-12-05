import cv2
import os
import rosbag
import pytz
import shutil
from cv_bridge import CvBridge
from datetime import datetime

# def resize_qualisys_frame(img):
#     h, w = img.shape[:2]   # h=544, w=736

#     # Compute target 4:3 crop
#     target_w = int(h * 4 / 3)  # 544 * 4/3 ≈ 725
#     target_h = h               # keep full height

#     # Center crop width to target_w
#     start_x = (w - target_w) // 2
#     end_x = start_x + target_w
#     img_cropped = img[:, start_x:end_x]   # shape ~ (544, 724, 3)

#     # Resize to 640x480
#     img_resized = cv2.resize(img_cropped, (640, 480), interpolation=cv2.INTER_AREA)
    
#     return img_resized

def resize_qualisys_frame(img):
    h, w = img.shape[:2]   # h=544, w=736

    # Compute target 4:3 crop
    target_w = w  # 544 * 4/3 ≈ 725
    target_h = int(w * 9 / 16)               # keep full height

    # Center crop width to target_w
    start_x = (h - target_h) // 2
    end_x = start_x + target_h
    img_cropped = img[start_x:end_x, :]   # shape ~ (544, 724, 3)
    # breakpoint()

    # # Resize to 640x480
    # img_resized = cv2.resize(img_cropped, (640, 480), interpolation=cv2.INTER_AREA)
    
    return img_cropped

def extract_images_from_video(video_path="qualisys_video.avi", output_folder="qualisys_camera_images", resize_frame=True):
    # === Configuration ===
    frame_interval = 1                      # Save every frame (use higher value to skip frames)

    # === Create output folder if it doesn't exist ===
    os.makedirs(output_folder, exist_ok=True)

    # === Open the video ===
    cap = cv2.VideoCapture(video_path)
    frame_count = 0
    saved_count = 0

    while True:
        success, frame = cap.read()
        if not success:
            break  # End of video

        if frame_count % frame_interval == 0:
            frame_filename = os.path.join(output_folder, f'{saved_count:04d}.jpg')
            if resize_frame:
                frame = resize_qualisys_frame(frame)
            cv2.imwrite(frame_filename, frame)
            saved_count += 1

        frame_count += 1

    cap.release()
    print(f"Saved {saved_count} frames to '{output_folder}'")


def save_frame(msg, count, output_folder):
    bridge = CvBridge()
    # Convert ROS Image message to OpenCV image
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Build filename like 0000.jpg, 0001.jpg ...
    filename = os.path.join(output_folder, f"{count:04d}.jpg")

    # Save image
    cv2.imwrite(filename, cv_img)
    print(f"Saved {filename}")

def extract_images_from_ros(time_str,
                            topic_name="/cam_L/color/image_raw",
                             rosbag_path="/home/robotlearning2/infants/data/0/trial_001/trial_ros.bag",
                             output_folder="rs_images"):
    # ======== Inpsect rosbg ============
    bag = rosbag.Bag(rosbag_path)
    local_tz = pytz.timezone("America/Chicago")
    os.makedirs(output_folder, exist_ok=True)

    # Print summary info
    # print("Topics and message types:")
    # print(bag.get_type_and_topic_info())
    # print(bag.get_type_and_topic_info()[1].keys())

    # Convert timestamp from Qualisys video to Unix time
    # Step 1: Parse the string into a naive datetime object (no timezone yet)
    dt = datetime.strptime(time_str, "%Y-%m-%d, %H:%M:%S.%f")
    # (Optional) Step 2: Localize to your timezone, e.g., Austin TX (America/Chicago)
    # If you know the time is in local time and want correct epoch conversion:
    localized_dt = local_tz.localize(dt)
    # Step 3: Convert to Unix timestamp (UTC-based)
    qualisys_video_start_time = localized_dt.timestamp()
    print(f"Qualisys video Unix time: {qualisys_video_start_time}")

    # Iterate through all messages
    count = 0
    check_diff = True
    for topic, msg, t in bag.read_messages():
        # print(f"Topic: {topic}, Time: {t}")
        # breakpoint()
        if topic == topic_name:

            diff = abs(qualisys_video_start_time - t.to_sec())
            print("time diff: ", diff)
            if check_diff and diff < 0.02:
                breakpoint()
                check_diff = False
            
            if not check_diff:
                save_frame(msg, count, output_folder)
                count += 1
            
            # # Convert ROS time to float seconds
            # timestamp_secs = t.to_sec()
            # print(timestamp_secs)
            # # Convert to datetime object (UTC)
            # dt = datetime.fromtimestamp(timestamp_secs, local_tz)
            # # Format it as a string
            # formatted_time = dt.strftime("%Y-%m-%d %H:%M:%S")
            # print(f"Message time: {formatted_time}")
            # breakpoint()

        #     print(f"Message: {msg}")
        # break  # remove 'break' to see all messages

    bag.close()
    # ==================================

def clean_and_rename_images(folder_path, rs_offset_to_qualisys):
    # Get all JPG files sorted by name
    images = sorted([f for f in os.listdir(folder_path) if f.lower().endswith('.jpg')])

    # Delete the first N images
    for i in range(min(rs_offset_to_qualisys, len(images))):
        os.remove(os.path.join(folder_path, images[i]))
    print(f"Deleted {min(rs_offset_to_qualisys, len(images))} images.")

    # Get remaining images after deletion
    remaining_images = sorted([f for f in os.listdir(folder_path) if f.lower().endswith('.jpg')])

    # Rename remaining images to 0000.jpg, 0001.jpg, ...
    for idx, filename in enumerate(remaining_images):
        new_name = f"{idx:04d}.jpg"
        old_path = os.path.join(folder_path, filename)
        new_path = os.path.join(folder_path, new_name)
        os.rename(old_path, new_path)
    print(f"Renamed {len(remaining_images)} images starting from 0000.jpg.")

def transfer_images(source_folder, destination_folder, start_index, end_index, filename_digits=4):
    # === Ensure destination exists ===
    destination_folder.mkdir(parents=True, exist_ok=True)

    # === Get and sort all .jpg files ===
    image_files = sorted(source_folder.glob("*.jpg"))

    # === Filter and copy files within the index range ===
    counter = 0
    for img_path in image_files:
        stem = img_path.stem  # e.g., '0001'
        
        try:
            index = int(stem)
        except ValueError:
            print(f"Skipping {img_path.name}: filename does not contain a valid number.")
            continue

        if start_index <= index <= end_index:
            new_name = f"img_{str(counter).zfill(filename_digits)}.jpg"
            destination_path = destination_folder / new_name
            shutil.copy(img_path, destination_path)
            print(f"Copied: {img_path.name} → {new_name}")
            counter += 1


def extract_iamges_from_ros2(topic_name="/cam_L/color/image_raw",
                             rosbag_path="/home/robotlearning2/infants/data/0/trial_001/trial_ros.bag",
                             output_folder="rs_images"):
    # ======== Inpsect rosbg ============
    bag = rosbag.Bag(rosbag_path)
    os.makedirs(output_folder, exist_ok=True)

    # Iterate through all messages
    count = 0
    check_diff = True
    for topic, msg, t in bag.read_messages():
        # print(f"Topic: {topic}, Time: {t}")
        # breakpoint()
        if topic == topic_name:
            save_frame(msg, count, output_folder)
            count += 1
    bag.close()
    # ==================================