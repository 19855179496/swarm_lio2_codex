#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Publishes a circle trajectory as PoseStamped on a shared topic.
All drones follow the same circle shape; cascadePID adds per-drone init offset.

Trajectory:
  x = center_x + radius * cos(omega * t)
  y = center_y + radius * sin(omega * t)
  z = height_z
"""

import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


class CircleGoalPublisher:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/goal")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.center_x = float(rospy.get_param("~center_x", 0.0))
        self.center_y = float(rospy.get_param("~center_y", 0.0))
        self.height_z = float(rospy.get_param("~height_z", 1.2))
        self.radius = float(rospy.get_param("~radius", 3.0))
        self.omega = float(rospy.get_param("~omega", 0.3))
        self.start_delay = float(rospy.get_param("~start_delay", 3.0))
        self.publish_yaw_tangent = bool(rospy.get_param("~publish_yaw_tangent", True))
        self.wait_for_start_signal = bool(rospy.get_param("~wait_for_start_signal", True))
        self.start_topic = rospy.get_param("~start_topic", "/start_figure8")
        self.hold_x = float(rospy.get_param("~hold_x", self.center_x))
        self.hold_y = float(rospy.get_param("~hold_y", self.center_y))
        self.hold_z = float(rospy.get_param("~hold_z", 0.0))

        self.started = not self.wait_for_start_signal
        self.start_ref_time = rospy.Time.now().to_sec()
        self.pub = rospy.Publisher(self.topic, PoseStamped, queue_size=20)
        if self.wait_for_start_signal:
            rospy.Subscriber(self.start_topic, Bool, self._start_callback, queue_size=1)
        rospy.loginfo("[circle_goal] topic=%s r=%.2f omega=%.3f z=%.2f", self.topic, self.radius, self.omega, self.height_z)

    def _start_callback(self, msg):
        if (not self.started) and msg.data:
            self.started = True
            self.start_ref_time = rospy.Time.now().to_sec()
            rospy.loginfo("[circle_goal] start signal received")

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not self.started:
                x, y, z, yaw = self.hold_x, self.hold_y, self.hold_z, 0.0
            else:
                t = max(0.0, rospy.Time.now().to_sec() - self.start_ref_time - self.start_delay)
                a = self.omega * t
                x = self.center_x + self.radius * math.cos(a)
                y = self.center_y + self.radius * math.sin(a)
                z = self.height_z
                dx = -self.radius * self.omega * math.sin(a)
                dy = self.radius * self.omega * math.cos(a)
                yaw = math.atan2(dy, dx) if self.publish_yaw_tangent else 0.0

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            q = quaternion_from_euler(0.0, 0.0, yaw)
            msg.pose.orientation = Quaternion(*q)
            self.pub.publish(msg)
            rate.sleep()


def main():
    rospy.init_node("circle_goal_publisher", anonymous=False)
    CircleGoalPublisher().run()


if __name__ == "__main__":
    main()
