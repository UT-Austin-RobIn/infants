import sys
import os
from bagpy import bagreader

def extract_topics(bag_path):
    if not os.path.exists(bag_path):
        print(f"[ERROR] Bag file not found: {bag_path}")
        return

    print(f"[INFO] Reading bag: {bag_path}")
    b = bagreader(bag_path)

    # List all available topics
    print("[INFO] Topics in the bag:")
    print(b.topic_table)

    # Topics to extract
    topics_to_extract = [
        '/cam_L/color/image_raw',
        '/cam_R/color/image_raw',
        '/cam_M/color/image_raw',
        '/cam_L/aligned_depth_to_color/image_raw',
        '/cam_R/aligned_depth_to_color/image_raw',
        '/cam_M/aligned_depth_to_color/image_raw',
        '/cam_L/color/camera_info',
        '/cam_R/color/camera_info',
        '/cam_M/color/camera_info'
    ]

    for topic in topics_to_extract:
        try:
            csv_file = b.message_by_topic(topic)
            csv_path = '%s/%s.csv' % (bag_path[:-4], topic[1:].replace('/', '-'))
            os.system('tar -czvf %s.tar.gz %s' % (csv_path, csv_path))
            os.system('rm %s' % csv_path)
            print(f"[INFO] Saved topic {topic} to: {csv_file}")
        except Exception as e:
            print(f"[WARNING] Could not extract {topic}: {e}")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python extract_topics.py path_to_rosbag.bag")
    else:
        extract_topics(sys.argv[1])
