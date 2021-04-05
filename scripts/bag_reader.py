#!/usr/bin/env python
# license removed for brevity
import rospy
import rosbag

from nav_msgs.msg import OccupancyGrid

def talker():
    pub = rospy.Publisher('grid_map', OccupancyGrid, queue_size=10)
    rospy.init_node('bag_reader', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    bag_file = "my_bagfile_1.bag"
    bag_in = rosbag.Bag(bag_file)

    data_holder = []

    for topic, msg, t in bag_in.read_messages():
        if topic == "mytopic":
            data_holder.append(msg)

    while not rospy.is_shutdown():
        pub.publish(data_holder)
        rate.sleep()




