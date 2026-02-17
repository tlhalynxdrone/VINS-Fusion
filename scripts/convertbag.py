#!/usr/bin/env python
import rosbag
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
input_bag = "/home/tlha/Work/bags/AR1/rosbag_2025-06-26-14-19-31_AR1_modified_02.bag"
output_bag = "/home/tlha/Work/bags/AR1/rosbag_2025-06-26-14-19-31_AR1_modified_02_raw.bag"
compressed_topic = "/camera/color/image_raw/compressed"
raw_topic = "/camera/color/image_raw"

bridge = CvBridge()

with rosbag.Bag(output_bag, 'w') as outbag:
    with rosbag.Bag(input_bag, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            if topic == compressed_topic:
                # convert compressed to cv::Mat
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                # convert cv::Mat to raw Image msg
                img_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                img_msg.header = msg.header
                outbag.write(raw_topic, img_msg, t)
            else:
                # copy all other topics
                outbag.write(topic, msg, t)
