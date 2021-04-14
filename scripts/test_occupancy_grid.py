#!/usr/bin/env python
# license removed for brevity
import rospy

from nav_msgs.msg import OccupancyGrid


def talker():
    pub = rospy.Publisher('fake_occupancy_grid', OccupancyGrid, queue_size=10)
    rospy.init_node('fake_occupancy_grid', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    grid = OccupancyGrid()
    # see if you can change metadata to calling on a parameter of rviz?
    grid.info.height = 20
    grid.info.width = 20
    values = [-1] * (20 * 20)
    # do some stuff to values here
    grid.data = values
    time_up = rospy.get_time() + 3
    height = 5
    width = 3
    while not rospy.is_shutdown():
        if rospy.get_time() > time_up:
            known_cells = []
            for i in range(height):
                for j in range(width):
                    known_cells.append(10 + i * 20 + j - 1)
                    known_cells.append(10 + i * 20 - j)
            for c in known_cells:
                if values[c] < 0:
                    if (c == 32 or c == 31 or c == 53 or c == 52 or c == 103 or c == 104 or c == 124 or c == 125 or
                            c == 146 or c == 145 or c == 144 or c == 333 or c == 334 or c == 353 or c == 335
                            or c == 222 or c == 223 or c == 224 or c == 243 or c == 263):
                        values[c] = 80
                    else:
                        values[c] = 20
            if height < 20:
                height += 1
            if width < 11:
                width += 1
            time_up = rospy.get_time() + 1
        pub.publish(grid)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
