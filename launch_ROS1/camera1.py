#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_publisher():
    rospy.init_node('camera_node', anonymous=True)
    image_pub = rospy.Publisher('camera_image', Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)  # 0 for default camera (adjust as needed)

    rate = rospy.Rate(10)  # Adjust the rate as per your requirements

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # Convert the image to ROS format and fill in the header
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()  # Set the timestamp
            img_msg.header.frame_id = "camera_frame"  # Replace with your frame ID

            image_pub.publish(img_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass

