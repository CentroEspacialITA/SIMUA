#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def laptop_gps_publisher():
    rospy.init_node('laptop_gps_publisher', anonymous=True)
    pub = rospy.Publisher('laptop_gps', NavSatFix, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        # Replace with actual code to get laptop coordinates
        latitude = 37.7749
        longitude = -122.4194

        # Publish coordinates
        msg = NavSatFix()
        msg.latitude = latitude
        msg.longitude = longitude
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        laptop_gps_publisher()
    except rospy.ROSInterruptException:
        pass
