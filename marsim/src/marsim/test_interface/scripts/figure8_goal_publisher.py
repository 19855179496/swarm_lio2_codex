#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


class Figure8GoalPublisher:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/goal")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        # Lissajous figure-8 trajectory:
        # x = cx + a * sin(w t)
        # y = cy + b * sin(2 w t)
        self.center_x = float(rospy.get_param("~center_x", 0.0))
        self.center_y = float(rospy.get_param("~center_y", 0.0))
        self.height_z = float(rospy.get_param("~height_z", 1.2))
        self.amp_x = float(rospy.get_param("~amp_x", 3.0))
        self.amp_y = float(rospy.get_param("~amp_y", 1.8))
        self.omega = float(rospy.get_param("~omega", 0.35))

        self.start_delay = float(rospy.get_param("~start_delay", 3.0))
        self.publish_yaw_tangent = bool(rospy.get_param("~publish_yaw_tangent", True))

        self.wait_for_start_signal = bool(rospy.get_param("~wait_for_start_signal", True))
        self.start_topic = rospy.get_param("~start_topic", "/start_figure8")
        self.hold_x = float(rospy.get_param("~hold_x", self.center_x))
        self.hold_y = float(rospy.get_param("~hold_y", self.center_y))
        self.hold_z = float(rospy.get_param("~hold_z", 0.0))
        self.t0 = rospy.Time.now().to_sec()
        self.start_ref_time = self.t0
        self.started = (not self.wait_for_start_signal)

        self.pub = rospy.Publisher(self.topic, PoseStamped, queue_size=20)
        if self.wait_for_start_signal:
            rospy.Subscriber(self.start_topic, Bool, self._start_callback, queue_size=1)

        rospy.loginfo("[figure8_goal_publisher] topic=%s frame=%s rate=%.1fHz", self.topic, self.frame_id, self.rate_hz)
        rospy.loginfo(
            "[figure8_goal_publisher] center=(%.2f, %.2f) z=%.2f amp=(%.2f, %.2f) omega=%.3f",
            self.center_x,
            self.center_y,
            self.height_z,
            self.amp_x,
            self.amp_y,
            self.omega,
        )
        rospy.loginfo(
            "[figure8_goal_publisher] wait_for_start_signal=%s start_topic=%s hold=(%.2f, %.2f, %.2f)",
            str(self.wait_for_start_signal),
            self.start_topic,
            self.hold_x,
            self.hold_y,
            self.hold_z,
        )

    def _start_callback(self, msg):
        if (not self.started) and msg.data:
            self.started = True
            self.start_ref_time = rospy.Time.now().to_sec()
            rospy.loginfo("[figure8_goal_publisher] received start signal, begin trajectory")

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if not self.started:
                x, y, z = self.hold_x, self.hold_y, self.hold_z
                yaw = 0.0
            else:
                t = max(0.0, now - self.start_ref_time - self.start_delay)

                wt = self.omega * t
                x = self.center_x + self.amp_x * math.sin(wt)
                y = self.center_y + self.amp_y * math.sin(2.0 * wt)
                z = self.height_z

                # Derivative for tangent yaw.
                dx = self.amp_x * self.omega * math.cos(wt)
                dy = self.amp_y * 2.0 * self.omega * math.cos(2.0 * wt)
                yaw = math.atan2(dy, dx) if self.publish_yaw_tangent else 0.0

            q = quaternion_from_euler(0.0, 0.0, yaw)

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation = Quaternion(*q)

            self.pub.publish(msg)
            rate.sleep()


def main():
    rospy.init_node("figure8_goal_publisher", anonymous=False)
    Figure8GoalPublisher().run()


if __name__ == "__main__":
    main()
