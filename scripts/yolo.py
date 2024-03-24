#!/usr/bin/env python3

import rospy, rospkg
import cv2, torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

model = YOLO('yolov8n.pt')

latest_image = None

def image_subscriber(image_msg):
    bridge = CvBridge()

    try:
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
        global latest_image
        latest_image = frame

    except CvBridgeError as e:
        rospy.logerr(e)

def image_subscriber_node():
    rospy.init_node('yolo_node', anonymous=True)
    
    topic = rospy.get_param('~image')
    rospy.Subscriber(topic, Image, image_subscriber)
    verbose = rospy.get_param('~verbose')
    publish = rospy.get_param('~publish')

    if publish:
        publisher = rospy.Publisher('/yolo_image', Image, queue_size=1)

    while not rospy.is_shutdown():
        if latest_image is not None:
            frame = latest_image
            results = model(frame)[0]
            
            plotted = results.plot()

            if verbose:
                cv2.imshow("Received Image", plotted)
                cv2.waitKey(1)

            # if publish:
            #     bridge = CvBridge()
            #     publisher.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

if __name__ == '__main__':
    try:
        image_subscriber_node()
    except rospy.ROSInterruptException:
        pass
