#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Publishes three diagonal-line back-and-forth trajectories on separate topics.
Each drone flies a diagonal line, reversing direction at the ends.
The diagonals are oriented so the paths cross in the middle.

Trajectory per drone:
  - Flies from point A to point B and back, repeatedly.
  - Uses smoothstep for gentle acceleration/deceleration.
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


class TripleDiagonalGoalPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.start_delay = float(rospy.get_param("~start_delay", 3.0))
        self.publish_yaw_tangent = bool(rospy.get_param("~publish_yaw_tangent", True))
        self.wait_for_start_signal = bool(rospy.get_param("~wait_for_start_signal", True))
        self.start_topic = rospy.get_param("~start_topic", "/start_figure8")
        self.period = float(rospy.get_param("~period", 12.0))  # seconds for one full A->B->A

        # Drone 0: diagonal from (-3, -3) to (3, 3)
        self.topic0 = rospy.get_param("~topic0", "/quad_0/goal")
        self.A0 = [float(x) for x in rospy.get_param("~A0", [-3.0, -3.0, 1.2])]
        self.B0 = [float(x) for x in rospy.get_param("~B0", [3.0, 3.0, 1.2])]

        # Drone 1: diagonal from (-3, 3) to (3, -3) -- crosses drone 0
        self.topic1 = rospy.get_param("~topic1", "/quad_1/goal")
        self.A1 = [float(x) for x in rospy.get_param("~A1", [-3.0, 3.0, 1.5])]
        self.B1 = [float(x) for x in rospy.get_param("~B1", [3.0, -3.0, 1.5])]

        # Drone 2: diagonal from (0, -4) to (0, 4) -- vertical line through center
        self.topic2 = rospy.get_param("~topic2", "/quad_2/goal")
        self.A2 = [float(x) for x in rospy.get_param("~A2", [0.0, -4.0, 1.8])]
        self.B2 = [float(x) for x in rospy.get_param("~B2", [0.0, 4.0, 1.8])]

        self.hold_x = float(rospy.get_param("~hold_x", 0.0))
        self.hold_y = float(rospy.get_param("~hold_y", 0.0))
        self.hold_z = float(rospy.get_param("~hold_z", 0.0))

        self.started = not self.wait_for_start_signal
        self.start_ref_time = rospy.Time.now().to_sec()

        self.pub0 = rospy.Publisher(self.topic0, PoseStamped, queue_size=20)
        self.pub1 = rospy.Publisher(self.topic1, PoseStamped, queue_size=20)
        self.pub2 = rospy.Publisher(self.topic2, PoseStamped, queue_size=20)

        if self.wait_for_start_signal:
            rospy.Subscriber(self.start_topic, Bool, self._start_callback, queue_size=1)

        rospy.loginfo("[triple_diagonal] topics=(%s, %s, %s) period=%.1fs", self.topic0, self.topic1, self.topic2, self.period)

    def _start_callback(self, msg):
        if (not self.started) and msg.data:
            self.started = True
            self.start_ref_time = rospy.Time.now().to_sec()
            rospy.loginfo("[triple_diagonal] received start signal")

    @staticmethod
    def _sample_AB(A, B, t, period):
        """Sample position and velocity along A<->B with smoothstep."""
        half = period / 2.0
        cycle = math.fmod(t, period)
        if cycle < half:
            alpha = cycle / half
            s = alpha * alpha * (3.0 - 2.0 * alpha)
            ds = 6.0 * alpha * (1.0 - alpha) / half
        else:
            alpha = (cycle - half) / half
            s = 1.0 - alpha * alpha * (3.0 - 2.0 * alpha)
            ds = -6.0 * alpha * (1.0 - alpha) / half

        pos = [A[i] + (B[i] - A[i]) * s for i in range(3)]
        vel = [(B[i] - A[i]) * ds for i in range(3)]
        return pos, vel

    def _make_msg(self, pos, yaw):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation = Quaternion(*q)
        return msg

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not self.started:
                hold = [self.hold_x, self.hold_y, self.hold_z]
                msg0 = self._make_msg(hold, 0.0)
                msg1 = self._make_msg(hold, 0.0)
                msg2 = self._make_msg(hold, 0.0)
            else:
                t = max(0.0, rospy.Time.now().to_sec() - self.start_ref_time - self.start_delay)

                pos0, vel0 = self._sample_AB(self.A0, self.B0, t, self.period)
                pos1, vel1 = self._sample_AB(self.A1, self.B1, t, self.period)
                pos2, vel2 = self._sample_AB(self.A2, self.B2, t, self.period)

                yaw0 = math.atan2(vel0[1], vel0[0]) if self.publish_yaw_tangent and (abs(vel0[0]) > 1e-3 or abs(vel0[1]) > 1e-3) else 0.0
                yaw1 = math.atan2(vel1[1], vel1[0]) if self.publish_yaw_tangent and (abs(vel1[0]) > 1e-3 or abs(vel1[1]) > 1e-3) else 0.0
                yaw2 = math.atan2(vel2[1], vel2[0]) if self.publish_yaw_tangent and (abs(vel2[0]) > 1e-3 or abs(vel2[1]) > 1e-3) else 0.0

                msg0 = self._make_msg(pos0, yaw0)
                msg1 = self._make_msg(pos1, yaw1)
                msg2 = self._make_msg(pos2, yaw2)

            self.pub0.publish(msg0)
            self.pub1.publish(msg1)
            self.pub2.publish(msg2)
            rate.sleep()


def main():
    rospy.init_node("triple_diagonal_goal_publisher", anonymous=False)
    TripleDiagonalGoalPublisher().run()


if __name__ == "__main__":
    main()
