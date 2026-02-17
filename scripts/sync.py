#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer

def callback(img0, img1):
    """
    This function is called when synchronized messages arrive.
    img0: Image from camera 0
    img1: Image from camera 1
    """
    rospy.loginfo(f"Synchronized images received: cam0={img0.header.stamp}, cam1={img1.header.stamp}")

    # Example: republish or process images here
    # For instance, you could convert to OpenCV using cv_bridge

def main():
    rospy.init_node('camera_sync_node')

    # Subscribers to camera topics
    sub0 = Subscriber('/camera/color/image_raw', Image)
    sub1 = Subscriber('/camera/image_raw', Image)

    # ApproximateTimeSynchronizer
    # queue_size = number of messages to buffer
    # slop = max allowed timestamp difference in seconds
    ats = ApproximateTimeSynchronizer([sub0, sub1],
                                      queue_size=10,
                                      slop=0.05)  # 50 ms tolerance
    ats.registerCallback(callback)

    rospy.loginfo("Camera sync node started. Waiting for synchronized images...")
    rospy.spin()

if __name__ == '__main__':
    main()
