import os
import sys
import cv2
import rosbag
import rospy
import threading
import numpy as np
from cv_bridge import CvBridge
from pydub import AudioSegment
from pydub.playback import play
import time

# Topics to replay
CAMERA_TOPICS = [
    "/cam_L/color/image_raw",
    "/cam_L/aligned_depth_to_color/image_raw",
    "/cam_R/color/image_raw",
    "/cam_R/aligned_depth_to_color/image_raw",
    "/cam_M/color/image_raw",
    "/cam_M/aligned_depth_to_color/image_raw",
]

BLANK_IMAGE = np.zeros((480, 640, 3), dtype=np.uint8)  # fallback if no data

def load_rosbag_images(bag_path):
    bridge = CvBridge()
    images_by_time = {}

    print("Loading images from rosbag...")

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=CAMERA_TOPICS):
            timestamp = msg.header.stamp.to_sec()

            if timestamp not in images_by_time:
                images_by_time[timestamp] = {}

            try:
                if msg.encoding == "16UC1":
                    img_cv = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    img_cv = cv2.normalize(img_cv, None, 0, 255, cv2.NORM_MINMAX)
                    img_cv = cv2.cvtColor(img_cv.astype(np.uint8), cv2.COLOR_GRAY2BGR)
                else:
                    img_cv = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                print(f"Failed to decode image from {topic}: {e}")
                img_cv = BLANK_IMAGE

            images_by_time[timestamp][topic] = img_cv

    print(f"Loaded {len(images_by_time)} timestamps.")
    return images_by_time

def play_audio(mp3_path):
    try:
        audio = AudioSegment.from_mp3(mp3_path)
        play(audio)
    except Exception as e:
        print(f"Failed to play audio: {e}")

def replay_trial(trial_path):
    bag_path = os.path.join(trial_path, "trial_ros.bag")
    mp3_path = os.path.join(trial_path, "trial_audio.wav")

    if not os.path.exists(bag_path):
        print(f"Missing bag file: {bag_path}")
        return
    if not os.path.exists(mp3_path):
        print(f"Missing audio file: {mp3_path}")
        return

    # --- Show rosbag info summary ---
    print("\n--- Rosbag Info Summary ---")
    with rosbag.Bag(bag_path, 'r') as bag:
        info = bag.get_type_and_topic_info()
        for topic, topic_info in info.topics.items():
            print(f"{topic}: {topic_info.message_count} messages")

    print("\nPress SPACE to begin playback, or 'q' to quit.")
    while True:
        img = np.full((200, 600, 3), 50, dtype=np.uint8)
        cv2.putText(img, "Press SPACE to begin playback", (30, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.imshow("Replay", img)
        key = cv2.waitKey(0)
        if key == 32:  # SPACE
            break
        elif key == ord('q'):
            cv2.destroyAllWindows()
            return

    # --- Load and play data ---
    images_by_time = load_rosbag_images(bag_path)
    timestamps = sorted(images_by_time.keys())

    # Launch audio in parallel
    audio_thread = threading.Thread(target=play_audio, args=(mp3_path,))
    audio_thread.start()

    last_time = None
    last_valid_image = {topic: BLANK_IMAGE for topic in CAMERA_TOPICS}  
    for t in timestamps:
        images = images_by_time[t]
        tiles = []

        for topic in CAMERA_TOPICS:
            if topic in images:
                img = images[topic]
                last_valid_image[topic] = img
            else:
                img = last_valid_image[topic]
           
            img = cv2.resize(img, (320, 240))  # resize for display
            tiles.append(img)

        # Arrange 3x2 grid
        top = np.hstack(tiles[:3])
        bottom = np.hstack(tiles[3:])
        frame = np.vstack([top, bottom])

        cv2.imshow("Replay", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        if last_time is not None:
            dt = t - last_time
            time.sleep(dt)
        last_time = t

    cv2.destroyAllWindows()
    audio_thread.join()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python replay_trial.py <path_to_trial_directory>")
        sys.exit(1)

    trial_path = sys.argv[1]
    replay_trial(trial_path)
