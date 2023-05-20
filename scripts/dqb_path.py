#!/usr/bin/python3

import rospy
import tf
import tf_conversions
import numpy as np
import astar
import typing
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header
from math import atan2, pi

# Path:
# - Header
# - PoseStamped[]
# - - Header
# - - Pose
# - - - Point
# - - - - x -> float
# - - - - y -> float
# - - - - z -> float
# - - - Quaternion
# - - - - x -> float
# - - - - y -> float
# - - - - z -> float
# - - - - w -> float


NODE_NAME = "path"
TOPIC_NAME = "dqb/path"
QUEUE_SIZE = 10
ROS_RATE_HZ = 1


class NavigationPath(object):
    ROBOT_ANGLE_OFFSET = pi / 2
    MAP_TOPIC = "/map"
    GOAL_TOPIC = "/move_base_simple/goal"
    SOURCE_FRAME = "base_footprint"
    TARGET_FRAME = "map"
    PATH_FRAME = "odom"

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
        )[::-1]

    def goal_callback(self, goal: PoseStamped) -> None:
        self.goal = goal
        self.is_goal = True

    def grid_to_global(x: int, y: int) -> typing.List[float]:
        x = 0.0
        y = 1.0
        return [x, y]

    def global_to_grid(x: float, y: float) -> typing.List[int]:
        x = 0
        y = 1
        return [x, y]

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

        self.robot = Pose(
            position=Point(x=trans[0], y=trans[1], z=trans[2]),
            orientation=Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3]),
        )

        poses = []

        tick = 0
        # цикл по построенному пути
        for i in range(3):
            point = Point(x=tick + i, y=tick + i, z=0)
            angle_to_current = 0
            if i != 0:
                angle_to_current = atan2(
                    point.y - poses[i - 1].pose.position.y,
                    point.x - poses[i - 1].pose.position.x,
                )

            qt = tf_conversions.transformations.quaternion_from_euler(
                ai=0, aj=0, ak=angle_to_current
            )
            quaternion = Quaternion(x=qt[0], y=qt[1], z=qt[2], w=qt[3])
            pose = PoseStamped(
                header=Header(seq=i),
                pose=Pose(position=point, orientation=quaternion),
            )
            poses.append(pose)

        tick += 1
        self.path = Path(
            header=Header(stamp=rospy.Time.now(), frame_id=self.PATH_FRAME), poses=poses
        )

        self.publisher.publish(self.path)


if __name__ == "__main__":
    try:
        rospy.init_node(NODE_NAME)
        publisher = rospy.Publisher(TOPIC_NAME, Path, queue_size=QUEUE_SIZE)
        update_rate = rospy.Rate(ROS_RATE_HZ)  # hz

        navigation_path = NavigationPath()

        while not rospy.is_shutdown() or not navigation_path.is_shutdown:
            try:
                navigation_path.step()
            except:
                continue

            update_rate.sleep()
    except rospy.ROSInterruptException:
        pass
