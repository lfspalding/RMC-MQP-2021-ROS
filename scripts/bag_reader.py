#!/usr/bin/env python
# license removed for brevity
import rospy

from nav_msgs.msg import OccupancyGrid

def talker():
    pub = rospy.Publisher('grid_map', OccupancyGrid, queue_size=10)
    rospy.init_node('bag_reader', anonymous=True)

