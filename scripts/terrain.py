#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point
from astar import astar


def make_gridcells(data):
    obs = GridCells()
    obs.cell_width = 1
    obs.cell_height = 1
    obs.header.frame_id = 'map'
    obs.cells = data
    return obs


def make_points(tuples):
    point_arr = []
    if tuples is not None:
        for t in tuples:
            point_arr.append(Point(t[0] + .5, t[1] + .5, 0))
    return point_arr


class TerrainMap:
    def __init__(self):
        rospy.init_node('terrain', anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz

        sub_grid = rospy.Subscriber('fake_occupancy_grid', OccupancyGrid, self.ogrid_callback)

        self.pub_obs = rospy.Publisher('obstacle', GridCells, queue_size=10)
        self.pub_clr = rospy.Publisher('clear_space', GridCells, queue_size=10)
        self.pub_path = rospy.Publisher('path', GridCells, queue_size=10)

        self.raw_map_data = []
        self.raw_map = 0
        self.width = 0
        self.height = 0
        self.resolution = 0

        self.obstacles = []
        self.clear = []
        self.threshold = 40

    def ogrid_callback(self, grid_map):
        self.raw_map_data = grid_map.data
        self.raw_map = grid_map
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution
        # rospy.loginfo("New grid received:")  # + str(self.raw_map))

    def sort_cells(self, min, max):
        cells = []
        for i, n in enumerate(self.raw_map_data):
            if min < n < max:
                x = i % self.width
                y = i // self.width
                cells.append(Point(x + .5, y + .5, 0))
        # rospy.loginfo(cells)
        return cells

    def path_gen(self):
        start = (10, 0)
        end = (3, 18)
        maze = []
        temp = []
        row = []
        for i in range(self.height):
            for j in range(self.width):
                temp.append(self.raw_map_data[i*self.width + j])
            for ele in temp:
                if ele > self.threshold:
                    row.append(1)
                else:
                    row.append(0)
            maze.append(row)
            temp = []
            row = []
        rospy.loginfo(maze)
        tuple_path = astar(maze, start, end)
        path = make_points(tuple_path)
        return make_gridcells(path)

    def run(self):
        # path_len = 1
        self.obstacles = self.sort_cells(40, 100) #these are probablility/possibly height values, check with John
        self.clear = self.sort_cells(0, 40)
        path = self.path_gen()

        time_up = rospy.get_time() + 2
        while not rospy.is_shutdown():
            if rospy.get_time() > time_up:
                # if path_len < 20:
                #     path_len += 1
                self.obstacles = self.sort_cells(40, 100)
                self.clear = self.sort_cells(0, 40)
                path = self.path_gen()
                time_up = rospy.get_time() + 1

            self.pub_obs.publish(make_gridcells(self.obstacles))
            self.pub_clr.publish(make_gridcells(self.clear))
            self.pub_path.publish(path)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        terrain = TerrainMap()
        terrain.run()

    except rospy.ROSInterruptException:
        pass