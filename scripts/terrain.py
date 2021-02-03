#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point


def terrain():
    pub_obs = rospy.Publisher('obstacle', GridCells, queue_size=10)
    pub_clr = rospy.Publisher('clear_space', GridCells, queue_size=10)
    rospy.init_node('terrain', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    o1 = Point(1, 1, 0)
    o2 = Point(2, 2, 0)
    o3 = Point(10, 10, 0)
    obstacles = [o1, o2, o3]
    c1 = Point(1, 2, 0)
    c2 = Point(3, 5, 0)
    c3 = Point(0, 0, 0)
    clear_space = [c1, c2, c3]
    while not rospy.is_shutdown():
        obs = GridCells()
        obs.cell_width = 1
        obs.cell_height = 1
        obs.header.frame_id = 'map'
        obs.cells = obstacles
        rospy.loginfo(str(obs))
        pub_obs.publish(obs)
        clr = GridCells()
        clr.cell_width = 1
        clr.cell_height = 1
        clr.header.frame_id = 'map'
        clr.cells = clear_space
        rospy.loginfo(str(clr))
        pub_clr.publish(clr)
        rate.sleep()

if __name__ == '__main__':
    try:
        terrain()
    except rospy.ROSInterruptException:
        pass