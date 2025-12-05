#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

def callback(msg, args):
    bag, topic = args
    bag.write(topic, msg)

if __name__ == "__main__":
    rospy.init_node("record_realsense_to_bag")

    # Output rosbag file
    bag_filename = "realsense_recording.bag"
    bag = rosbag.Bag(bag_filename, "w")

    # Topics to record (adjust to your camera setup)
    topics = [
        "/cam_L/color/image_raw",
        "/cam_L/color/camera_info",
        # "/cam_L/depth/image_rect_raw",
        # "/cam_L/depth/camera_info"
    ]

    # Subscribe to topics
    subs = []
    for topic in topics:
        # breakpoint()
        msg_type = rospy.get_published_topics(topic)[0][1]  # lookup type
        if "Image" in msg_type:
            subs.append(rospy.Subscriber(topic, Image, callback, (bag, topic)))
        elif "CameraInfo" in msg_type:
            subs.append(rospy.Subscriber(topic, CameraInfo, callback, (bag, topic)))

    rospy.loginfo(f"Recording topics {topics} into {bag_filename}")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Stopping recording.")
    finally:
        bag.close()
        rospy.loginfo(f"Saved rosbag: {bag_filename}")
