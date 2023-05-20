#!/usr/bin/python3

import rospy
import tf
import tf_conversions
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
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

ROBOT_ANGLE_OFFSET = pi / 2


if __name__ == "__main__":
    try:
        rospy.init_node("path")
        publisher = rospy.Publisher("dqb/path", Path, queue_size=10)
        update_rate = rospy.Rate(0.2)  # hz

        tf_listener = tf.TransformListener()

        tick = 0
        while not rospy.is_shutdown():
            try:
                (trans, rot) = tf_listener.lookupTransform(
                    source_frame="map",
                    target_frame="base_footprint",
                    time=rospy.Time(0),
                )
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue

            robot_angle = tf_conversions.transformations.quaternion_multiply(
                rot,
                tf_conversions.transformations.quaternion_from_euler(
                    ai=0, aj=0, ak=ROBOT_ANGLE_OFFSET
                ),
            )

            poses = []

            # цикл по построенному пути
            for i in range(3):
                # TODO: сначала сгладить траекторию

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

            path = Path(
                header=Header(stamp=rospy.Time.now(), frame_id="odom"), poses=poses
            )

            publisher.publish(path)

            tick += 1
            update_rate.sleep()
    except rospy.ROSInterruptException:
        pass
