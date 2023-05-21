#!/usr/bin/python3

import tf_conversions
import typing
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header
from itertools import product
from scipy import interpolate
from astar import astar
from math import atan2, pi


NODE_NAME = "path"
TOPIC_NAME = "dqb/path"
QUEUE_SIZE = 10
ROS_RATE_HZ = 0.25


class NavigationPath(object):
    ROBOT_ANGLE_OFFSET = pi / 2
    ROBOT_RADIUS = 5
    MAP_TOPIC = "/map"
    GOAL_TOPIC = "/move_base_simple/goal"
    SOURCE_FRAME = "base_footprint"
    TARGET_FRAME = "map"
    PATH_FRAME = "odom"

    MAP_OBSTACLE = 100
    MAP_PADDING = 90
    MAP_FREE = 0
    MAP_UNKNOWN = -1

    def __init__(self) -> None:
        self.map_subscriber = rospy.Subscriber(
            self.MAP_TOPIC, OccupancyGrid, self.map_callback
        )
        self.goal_subscriber = rospy.Subscriber(
            self.GOAL_TOPIC, PoseStamped, self.goal_callback
        )
        self.tf_listener = tf.TransformListener()

        self.step_count = 0
        self.is_shutdown = False

        self.publisher = rospy.Publisher("dqb/path", Path, queue_size=10)

    def map_callback(self, occupancy_grid: OccupancyGrid) -> None:
        self.map = occupancy_grid
        self.map.data = np.array(occupancy_grid.data).reshape(
            (occupancy_grid.info.width, occupancy_grid.info.height)
        )

        # черная магия
        self.map.data = np.flip(np.rot90(self.map.data), 0)

        # добавление паддинга препятствиям
        corners = self.find_bounding_box(self.map.data)
        for i in range(corners[0][0], corners[1][0]):
            for j in range(corners[0][1], corners[1][1]):
                if self.map.data[i][j] <= self.MAP_PADDING:
                    continue
                for point in self.points_in_circle(i, j, self.ROBOT_RADIUS):
                    if self.map.data[point[0]][point[1]] < self.MAP_PADDING:
                        self.map.data[point[0]][point[1]] = self.MAP_PADDING

    def goal_callback(self, goal: PoseStamped) -> None:
        goal_grid = self.global_to_grid(goal.pose.position.x, goal.pose.position.y)
        if self.map.data[goal_grid[0]][goal_grid[1]] >= self.MAP_PADDING:
            print("Goal is in obstacle")
            self.is_goal = False
            self.goal = None
            return

        self.goal = goal
        self.goal.pose.position = Point(x=goal_grid[0], y=goal_grid[1], z=0)
        self.is_goal = True
        self.step()

    def find_bounding_box(self, np_array):
        x, y = np.where(np_array != self.MAP_UNKNOWN)
        top_left = x.min(), y.min()
        bottom_right = x.max(), y.max()
        return [top_left, bottom_right]

    def points_in_circle(self, center_x, center_y, radius):
        for x, y in product(range(int(radius) + 1), repeat=2):
            if x**2 + y**2 <= radius**2:
                yield from set(
                    (
                        (center_x + x, center_y + y),
                        (center_x + x, center_y - y),
                        (center_x - x, center_y + y),
                        (center_x - x, center_y - y),
                    )
                )

    def grid_to_global(self, x: int, y: int) -> typing.List[float]:
        x_global = x * self.map.info.resolution + self.map.info.origin.position.x
        y_global = y * self.map.info.resolution + self.map.info.origin.position.y
        return [x_global, y_global]

    def global_to_grid(self, x: float, y: float) -> typing.List[int]:
        x_grid = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        y_grid = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        return [x_grid, y_grid]

    def path_smoothing(self, path):
        path = path[::4]
        x, y = zip(*path)
        f, u = interpolate.splprep([x, y], s=0)
        x_arr, y_arr = interpolate.splev(np.linspace(0, 1, 100), f)
        smooth = []
        for i, item in enumerate(x_arr):
            smooth.append((item, y_arr[i]))

        return smooth

    def step(self) -> None:
        self.step_count += 1

        (trans, rot) = self.tf_listener.lookupTransform(
            source_frame=self.SOURCE_FRAME,
            target_frame=self.TARGET_FRAME,
            time=rospy.Time(0),
        )

        rot = tf_conversions.transformations.quaternion_multiply(
            rot,
            tf_conversions.transformations.quaternion_from_euler(
                ai=0, aj=0, ak=self.ROBOT_ANGLE_OFFSET
            ),
        )

        robot_grid = self.global_to_grid(trans[0], trans[1])

        self.robot = Pose(
            position=Point(x=robot_grid[0], y=robot_grid[1], z=0),
            orientation=Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3]),
        )

        if not self.is_goal:
            self.path = Path(
                header=Header(stamp=rospy.Time.now(), frame_id=self.PATH_FRAME),
                poses=[],
            )
            self.publisher.publish(self.path)
            return

        astar_path = astar(
            self.map.data,
            (self.robot.position.x, self.robot.position.y),
            (self.goal.pose.position.x, self.goal.pose.position.y),
            allow_diagonal_movement=True,
        )

        if not astar_path:
            print(
                "Warning: Path between (%s; %s) and (%s; %s) not found"
                % (
                    self.robot.position.x,
                    self.robot.position.y,
                    self.goal.pose.position.x,
                    self.goal.pose.position.y,
                ),
            )
            self.is_goal = False
            return

        astar_path = self.path_smoothing(astar_path)

        poses = []
        prev, curr, index = None, None, 0
        for node in astar_path:
            node_global = self.grid_to_global(node[0], node[1])
            curr = Point(x=node_global[0], y=node_global[1], z=0)
            angle_to_curr = 0
            if prev:
                angle_to_curr = atan2(
                    curr.y - prev.y,
                    curr.x - prev.x,
                )
            quaternion = tf_conversions.transformations.quaternion_from_euler(
                ai=0, aj=0, ak=angle_to_curr
            )
            quaternion = Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            )
            pose = PoseStamped(
                header=Header(seq=index),
                pose=Pose(position=curr, orientation=quaternion),
            )
            poses.append(pose)
            prev = curr
            index += 1

        self.path = Path(
            header=Header(stamp=rospy.Time.now(), frame_id=self.PATH_FRAME), poses=poses
        )

        self.publisher.publish(self.path)


if __name__ == "__main__":
    try:
        rospy.init_node(NODE_NAME)
        publisher = rospy.Publisher(TOPIC_NAME, Path, queue_size=QUEUE_SIZE)
        update_rate = rospy.Rate(ROS_RATE_HZ)

        navigation_path = NavigationPath()

        while not rospy.is_shutdown() or not navigation_path.is_shutdown:
            try:
                navigation_path.step()
            except:
                continue

            update_rate.sleep()
    except rospy.ROSInterruptException:
        pass
