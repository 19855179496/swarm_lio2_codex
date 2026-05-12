#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Publishes three independent circle trajectories on separate topics.
Each drone follows its own circle with different radius, center, and phase.

Trajectory per drone:
  x = center_x + radius * cos(omega * t + phase)
  y = center_y + radius * sin(omega * t + phase)
  z = height_z
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


class TripleCircleGoalPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.start_delay = float(rospy.get_param("~start_delay", 3.0))
        self.publish_yaw_tangent = bool(rospy.get_param("~publish_yaw_tangent", True))
        self.wait_for_start_signal = bool(rospy.get_param("~wait_for_start_signal", True))
        self.start_topic = rospy.get_param("~start_topic", "/start_figure8")

        # Drone 0 circle
        self.topic0 = rospy.get_param("~topic0", "/quad_0/goal")
        self.cx0 = float(rospy.get_param("~center_x0", 0.0))
        self.cy0 = float(rospy.get_param("~center_y0", 0.0))
        self.r0 = float(rospy.get_param("~radius0", 3.0))
        self.h0 = float(rospy.get_param("~height_z0", 1.2))
        self.omega0 = float(rospy.get_param("~omega0", 0.3))
        self.phase0 = float(rospy.get_param("~phase0", 0.0))

        # Drone 1 circle
        self.topic1 = rospy.get_param("~topic1", "/quad_1/goal")
        self.cx1 = float(rospy.get_param("~center_x1", 0.0))
        self.cy1 = float(rospy.get_param("~center_y1", 0.0))
        self.r1 = float(rospy.get_param("~radius1", 3.0))
        self.h1 = float(rospy.get_param("~height_z1", 1.2))
        self.omega1 = float(rospy.get_param("~omega1", 0.3))
        self.phase1 = float(rospy.get_param("~phase1", 2.094))  # 2*pi/3

        # Drone 2 circle
        self.topic2 = rospy.get_param("~topic2", "/quad_2/goal")
        self.cx2 = float(rospy.get_param("~center_x2", 0.0))
        self.cy2 = float(rospy.get_param("~center_y2", 0.0))
        self.r2 = float(rospy.get_param("~radius2", 3.0))
        self.h2 = float(rospy.get_param("~height_z2", 1.2))
        self.omega2 = float(rospy.get_param("~omega2", 0.3))
        self.phase2 = float(rospy.get_param("~phase2", 4.189))  # 4*pi/3

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

        rospy.loginfo("[triple_circle] topics=(%s, %s, %s) rate=%.1fHz", self.topic0, self.topic1, self.topic2, self.rate_hz)
        rospy.loginfo("[triple_circle] r=(%.2f, %.2f, %.2f) omega=(%.3f, %.3f, %.3f)",
                      self.r0, self.r1, self.r2, self.omega0, self.omega1, self.omega2)
        rospy.loginfo("[triple_circle] phase=(%.2f, %.2f, %.2f) rad", self.phase0, self.phase1, self.phase2)

    def _start_callback(self, msg):
        if (not self.started) and msg.data:
            self.started = True
            self.start_ref_time = rospy.Time.now().to_sec()
            rospy.loginfo("[triple_circle] received start signal")

    def _make_msg(self, x, y, z, yaw, frame_id):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation = Quaternion(*q)
        return msg

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not self.started:
                msg0 = self._make_msg(self.hold_x, self.hold_y, self.hold_z, 0.0, self.frame_id)
                msg1 = self._make_msg(self.hold_x, self.hold_y, self.hold_z, 0.0, self.frame_id)
                msg2 = self._make_msg(self.hold_x, self.hold_y, self.hold_z, 0.0, self.frame_id)
            else:
                t = max(0.0, rospy.Time.now().to_sec() - self.start_ref_time - self.start_delay)

                # Drone 0
                a0 = self.omega0 * t + self.phase0
                x0 = self.cx0 + self.r0 * math.cos(a0)
                y0 = self.cy0 + self.r0 * math.sin(a0)
                dx0 = -self.r0 * self.omega0 * math.sin(a0)
                dy0 = self.r0 * self.omega0 * math.cos(a0)
                yaw0 = math.atan2(dy0, dx0) if self.publish_yaw_tangent else 0.0

                # Drone 1
                a1 = self.omega1 * t + self.phase1
                x1 = self.cx1 + self.r1 * math.cos(a1)
                y1 = self.cy1 + self.r1 * math.sin(a1)
                dx1 = -self.r1 * self.omega1 * math.sin(a1)
                dy1 = self.r1 * self.omega1 * math.cos(a1)
                yaw1 = math.atan2(dy1, dx1) if self.publish_yaw_tangent else 0.0

                # Drone 2
                a2 = self.omega2 * t + self.phase2
                x2 = self.cx2 + self.r2 * math.cos(a2)
                y2 = self.cy2 + self.r2 * math.sin(a2)
                dx2 = -self.r2 * self.omega2 * math.sin(a2)
                dy2 = self.r2 * self.omega2 * math.cos(a2)
                yaw2 = math.atan2(dy2, dx2) if self.publish_yaw_tangent else 0.0

                msg0 = self._make_msg(x0, y0, self.h0, yaw0, self.frame_id)
                msg1 = self._make_msg(x1, y1, self.h1, yaw1, self.frame_id)
                msg2 = self._make_msg(x2, y2, self.h2, yaw2, self.frame_id)

            self.pub0.publish(msg0)
            self.pub1.publish(msg1)
            self.pub2.publish(msg2)
            rate.sleep()


def main():
    rospy.init_node("triple_circle_goal_publisher", anonymous=False)
    TripleCircleGoalPublisher().run()


if __name__ == "__main__":
    main()
