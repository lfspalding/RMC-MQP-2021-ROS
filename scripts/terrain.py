#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point

# generates fake path, switch out for A* later
def path_gen(length):
    points = []
    start = [5, 4]
    for i in range(length):
        points.append(Point(start[0] - .5, start[1] - .5, 0))
        if i % 6 == 0:
            start = [start[0] - 1, start[1]]
        elif i % 5 == 0:
            start = [start[0], start[1] - 1]
        elif i % 4 == 0:
            start = [start[0] - 1, start[1]]
        elif i % 3 == 0:
            start = [start[0], start[1] - 1]
        elif i % 2 == 0:
            start = [start[0] - 1, start[1]]
    return points


def make_gridcells(data):
    obs = GridCells()
    obs.cell_width = 1
    obs.cell_height = 1
    obs.header.frame_id = 'map'
    obs.cells = data
    return obs


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

    def ogrid_callback(self, grid_map):
        self.raw_map_data = grid_map.data
        self.raw_map = grid_map
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution
        rospy.loginfo("New grid received:")  # + str(self.raw_map))

    def sort_cells(self, min, max):
        cells = []
        for i, n in enumerate(self.raw_map_data):
            if min < n < max:
                x = i % self.width
                y = i // self.width
                cells.append(Point(x + .5, y + .5, 0))
        rospy.loginfo(cells)
        return cells

    def astar(self, start, end):
        pass
        # TODO

    def run(self):
        path_len = 1
        self.obstacles = self.sort_cells(40, 100) #these are probablility/possibly height values, check with John
        self.clear = self.sort_cells(0, 40)
        # path_points = path_gen(path_len)

        time_up = rospy.get_time() + 2
        while not rospy.is_shutdown():
            if rospy.get_time() > time_up:
                if path_len < 20:
                    path_len += 1
                self.obstacles = self.sort_cells(40, 100)
                self.clear = self.sort_cells(0, 40)
                # path_points = path_gen(path_len)
                time_up = rospy.get_time() + 1

            self.pub_obs.publish(make_gridcells(self.obstacles))
            self.pub_clr.publish(make_gridcells(self.clear))
            # self.pub_path.publish(path)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        terrain = TerrainMap()
        terrain.run()

    except rospy.ROSInterruptException:
        pass