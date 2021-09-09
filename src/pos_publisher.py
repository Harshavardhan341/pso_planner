#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

if __name__=='__main__':
    rospy.init_node("pos_publisher")
    pos_pub = rospy.Publisher("/desired_pos",Point,queue_size = 10)
    position = Point()
    position.x = -2
    position.y = 5
    rospy.sleep(2.0)
    

    pos_pub.publish(position)
    
