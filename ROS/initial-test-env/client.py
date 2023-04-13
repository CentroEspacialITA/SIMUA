#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def raspberry_gps_subscriber():
    rospy.init_node('raspberry_gps_subscriber', anonymous=True)
    rospy.Subscriber('laptop_gps', NavSatFix, callback)
    rospy.spin()

def callback(msg):
    rospy.loginfo('Received coordinates: (%f, %f)', msg.latitude, msg.longitude)

if __name__ == '__main__':
    try:
        raspberry_gps_subscriber()
    except rospy.ROSInterruptException:
        pass
