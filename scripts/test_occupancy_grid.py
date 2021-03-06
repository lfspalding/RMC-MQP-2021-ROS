#!/usr/bin/env python
# license removed for brevity
import rospy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('fake_occupancy_grid', OccupancyGrid, queue_size=10)
    rospy.init_node('fake_occupancy_grid', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    grid = OccupancyGrid()
    # see if you can change metadata to calling on a parameter of rviz?
    grid.info.height = 20
    grid.info.width = 20
    values = [-1] * (20*20)
    # do some stuff to values here
    grid.data = values
    while not rospy.is_shutdown():
        pub.publish(grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass