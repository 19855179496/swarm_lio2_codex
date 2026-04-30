#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

import math

import rospy
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _vec3_list_param(name, default_value):
    raw = rospy.get_param(name, default_value)
    if not isinstance(raw, list) or len(raw) != 3:
        raise ValueError("param {} must be a list with 3 elements".format(name))
    return (float(raw[0]), float(raw[1]), float(raw[2]))


def _waypoints_param(name, default_value):
    raw = rospy.get_param(name, default_value)
    if not isinstance(raw, list) or len(raw) == 0:
        raise ValueError("param {} must be a non-empty list".format(name))

    waypoints = []
    for idx, item in enumerate(raw):
        if not isinstance(item, list) or len(item) != 3:
            raise ValueError("param {}[{}] must be [x, y, z]".format(name, idx))
        waypoints.append((float(item[0]), float(item[1]), float(item[2])))
    return waypoints


def _gain_param(name, default_value):
    raw = rospy.get_param(name, default_value)
    if not isinstance(raw, list) or len(raw) != 3:
        raise ValueError("param {} must have 3 elements".format(name))
    return [float(raw[0]), float(raw[1]), float(raw[2])]


class RouteSampler:
    def __init__(self, waypoints, segment_duration, dwell_duration, loop, smooth_profile):
        self.waypoints = waypoints
        self.segment_duration = max(1e-3, float(segment_duration))
        self.dwell_duration = max(0.0, float(dwell_duration))
        self.loop = loop
        self.smooth_profile = smooth_profile

        seg_num = max(0, len(waypoints) - 1)
        self.block_duration = self.segment_duration + self.dwell_duration
        self.total_duration = seg_num * self.block_duration + self.dwell_duration

    @staticmethod
    def _smoothstep_with_derivative(alpha):
        a = max(0.0, min(1.0, alpha))
        s = a * a * (3.0 - 2.0 * a)
        ds = 6.0 * a * (1.0 - a)
        return s, ds

    def sample(self, t_sec):
        if len(self.waypoints) == 1:
            p = self.waypoints[0]
            return p, (0.0, 0.0, 0.0), 0

        t = max(0.0, float(t_sec))
        if self.loop and self.total_duration > 1e-6:
            t = math.fmod(t, self.total_duration)
        else:
            t = min(t, self.total_duration)

        for idx in range(len(self.waypoints) - 1):
            t0 = idx * self.block_duration
            if t < t0 + self.segment_duration:
                alpha = (t - t0) / self.segment_duration
                p0 = self.waypoints[idx]
                p1 = self.waypoints[idx + 1]
                if self.smooth_profile:
                    s, ds = self._smoothstep_with_derivative(alpha)
                else:
                    s = alpha
                    ds = 1.0
                pos = (
                    p0[0] + (p1[0] - p0[0]) * s,
                    p0[1] + (p1[1] - p0[1]) * s,
                    p0[2] + (p1[2] - p0[2]) * s,
                )
                vel = (
                    (p1[0] - p0[0]) * ds / self.segment_duration,
                    (p1[1] - p0[1]) * ds / self.segment_duration,
                    (p1[2] - p0[2]) * ds / self.segment_duration,
                )
                return pos, vel, idx

            if t < t0 + self.block_duration:
                p = self.waypoints[idx + 1]
                return p, (0.0, 0.0, 0.0), idx

        p = self.waypoints[-1]
        return p, (0.0, 0.0, 0.0), len(self.waypoints) - 2


class DualWaypointMissionPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.start_delay = float(rospy.get_param("~start_delay", 1.0))
        self.wait_for_start_signal = _as_bool(rospy.get_param("~wait_for_start_signal", False))
        self.start_topic = rospy.get_param("~start_topic", "/start_waypoint_mission")
        self.publish_yaw_tangent = _as_bool(rospy.get_param("~publish_yaw_tangent", True))

        self.topic_uav0 = rospy.get_param("~topic_uav0", "/quad_0/planning/pos_cmd")
        self.topic_uav1 = rospy.get_param("~topic_uav1", "/quad_1/planning/pos_cmd")

        default_uav0 = [
            [0.0, 0.0, 1.2],
            [0.8, 2.4, 1.2],
            [-2.5, 1.8, 1.2],
            [-0.8, 1.2, 1.2],
            [0.8, 2.2, 1.2],
            [-2.5, 1.4, 1.2],
            [0.0, 0.0, 1.2],
        ]
        default_uav1 = [
            [-3.0, 0.0, 1.5],
            [-2.2, -2.4, 1.5],
            [0.4, -1.8, 1.5],
            [-1.6, -1.2, 1.5],
            [-2.2, -2.2, 1.5],
            [0.4, -1.4, 1.5],
            [-3.0, 0.0, 1.5],
        ]
        self.waypoints_uav0 = _waypoints_param("~waypoints_uav0", default_uav0)
        self.waypoints_uav1 = _waypoints_param("~waypoints_uav1", default_uav1)

        self.segment_duration = float(rospy.get_param("~segment_duration", 3.0))
        self.dwell_duration = float(rospy.get_param("~dwell_duration", 0.8))
        self.loop_mission = _as_bool(rospy.get_param("~loop_mission", True))
        self.smooth_profile = _as_bool(rospy.get_param("~smooth_profile", True))

        self.time_offset_uav0 = float(rospy.get_param("~time_offset_uav0", 0.0))
        self.time_offset_uav1 = float(rospy.get_param("~time_offset_uav1", 1.5))
        self.min_pair_distance = float(rospy.get_param("~min_pair_distance", 1.35))
        self.safety_hold_altitude_uav1 = float(rospy.get_param("~safety_hold_altitude_uav1", 2.2))

        self.kx = _gain_param("~kx", [7.0, 7.0, 6.2])
        self.kv = _gain_param("~kv", [4.0, 4.0, 4.0])

        hold_z = float(rospy.get_param("~hold_z", 0.0))
        default_hold0 = [self.waypoints_uav0[0][0], self.waypoints_uav0[0][1], hold_z]
        default_hold1 = [self.waypoints_uav1[0][0], self.waypoints_uav1[0][1], hold_z]
        self.hold_uav0 = _vec3_list_param("~hold_uav0", default_hold0)
        self.hold_uav1 = _vec3_list_param("~hold_uav1", default_hold1)
        self.hold_yaw_uav0 = float(rospy.get_param("~hold_yaw_uav0", 0.0))
        self.hold_yaw_uav1 = float(rospy.get_param("~hold_yaw_uav1", 0.0))

        self.pub_uav0 = rospy.Publisher(self.topic_uav0, PositionCommand, queue_size=20)
        self.pub_uav1 = rospy.Publisher(self.topic_uav1, PositionCommand, queue_size=20)

        self.started = not self.wait_for_start_signal
        self.start_ref_time = rospy.Time.now().to_sec()
        if self.wait_for_start_signal:
            rospy.Subscriber(self.start_topic, Bool, self._start_callback, queue_size=1)

        self.route0 = RouteSampler(
            self.waypoints_uav0,
            self.segment_duration,
            self.dwell_duration,
            self.loop_mission,
            self.smooth_profile,
        )
        self.route1 = RouteSampler(
            self.waypoints_uav1,
            self.segment_duration,
            self.dwell_duration,
            self.loop_mission,
            self.smooth_profile,
        )

        self.last_phase0 = -1
        self.last_phase1 = -1
        self.safety_event_counter = 0

        rospy.loginfo(
            "[dual_waypoint_mission] topic_uav0=%s topic_uav1=%s rate=%.1fHz",
            self.topic_uav0,
            self.topic_uav1,
            self.rate_hz,
        )
        rospy.loginfo(
            "[dual_waypoint_mission] wait_for_start_signal=%s start_topic=%s start_delay=%.2f",
            str(self.wait_for_start_signal),
            self.start_topic,
            self.start_delay,
        )
        rospy.loginfo(
            "[dual_waypoint_mission] segment=%.2fs dwell=%.2fs loop=%s",
            self.segment_duration,
            self.dwell_duration,
            str(self.loop_mission),
        )
        rospy.loginfo(
            "[dual_waypoint_mission] smooth_profile=%s offset=(%.2f, %.2f) min_pair_distance=%.2f safety_hold_alt_uav1=%.2f",
            str(self.smooth_profile),
            self.time_offset_uav0,
            self.time_offset_uav1,
            self.min_pair_distance,
            self.safety_hold_altitude_uav1,
        )

    def _start_callback(self, msg):
        if (not self.started) and msg.data:
            self.started = True
            self.start_ref_time = rospy.Time.now().to_sec()
            rospy.loginfo("[dual_waypoint_mission] received start signal, mission starts")

    @staticmethod
    def _norm3(vec):
        return math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])

    @staticmethod
    def _dist3(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _build_cmd(self, now, pos, vel, yaw, trajectory_id):
        msg = PositionCommand()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        msg.position.x = pos[0]
        msg.position.y = pos[1]
        msg.position.z = pos[2]

        msg.velocity.x = vel[0]
        msg.velocity.y = vel[1]
        msg.velocity.z = vel[2]

        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0

        msg.jerk.x = 0.0
        msg.jerk.y = 0.0
        msg.jerk.z = 0.0

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        msg.attitude.x = 0.0
        msg.attitude.y = 0.0
        msg.attitude.z = 0.0

        msg.thrust.x = 0.0
        msg.thrust.y = 0.0
        msg.thrust.z = 0.0

        msg.yaw = yaw
        msg.yaw_dot = 0.0
        msg.vel_norm = self._norm3(vel)
        msg.acc_norm = 0.0

        msg.kx = list(self.kx)
        msg.kv = list(self.kv)
        msg.trajectory_id = int(max(0, trajectory_id))
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY

        return msg

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            now_sec = now.to_sec()

            if not self.started:
                pos0 = self.hold_uav0
                pos1 = self.hold_uav1
                vel0 = (0.0, 0.0, 0.0)
                vel1 = (0.0, 0.0, 0.0)
                yaw0 = self.hold_yaw_uav0
                yaw1 = self.hold_yaw_uav1
                tid0 = 0
                tid1 = 0
            else:
                t = max(0.0, now_sec - self.start_ref_time - self.start_delay)
                t0 = max(0.0, t + self.time_offset_uav0)
                t1 = max(0.0, t + self.time_offset_uav1)

                pos0, vel0, phase0 = self.route0.sample(t0)
                pos1, vel1, phase1 = self.route1.sample(t1)

                yaw0 = self.hold_yaw_uav0
                if self.publish_yaw_tangent and self._norm3(vel0) > 1e-3:
                    yaw0 = math.atan2(vel0[1], vel0[0])

                yaw1 = self.hold_yaw_uav1
                if self.publish_yaw_tangent and self._norm3(vel1) > 1e-3:
                    yaw1 = math.atan2(vel1[1], vel1[0])

                pair_dist = self._dist3(pos0, pos1)
                if pair_dist < self.min_pair_distance:
                    safe_z = max(
                        self.safety_hold_altitude_uav1,
                        pos0[2] + self.min_pair_distance + 0.2,
                        pos1[2],
                    )
                    pos1 = (pos1[0], pos1[1], safe_z)
                    vel1 = (0.0, 0.0, 0.0)
                    yaw1 = self.hold_yaw_uav1
                    self.safety_event_counter += 1
                    if self.safety_event_counter <= 5 or self.safety_event_counter % 20 == 0:
                        rospy.logwarn(
                            "[dual_waypoint_mission] safety guard triggered: dist=%.3f < %.3f, hold uav1 at z=%.2f (count=%d)",
                            pair_dist,
                            self.min_pair_distance,
                            safe_z,
                            self.safety_event_counter,
                        )

                if phase0 != self.last_phase0 or phase1 != self.last_phase1:
                    rospy.loginfo(
                        "[dual_waypoint_mission] t=%.2f phase_uav0=%d phase_uav1=%d pos0=(%.2f, %.2f, %.2f) pos1=(%.2f, %.2f, %.2f)",
                        t,
                        phase0,
                        phase1,
                        pos0[0],
                        pos0[1],
                        pos0[2],
                        pos1[0],
                        pos1[1],
                        pos1[2],
                    )
                    self.last_phase0 = phase0
                    self.last_phase1 = phase1

                tid0 = phase0 + 1
                tid1 = phase1 + 1

            self.pub_uav0.publish(self._build_cmd(now, pos0, vel0, yaw0, tid0))
            self.pub_uav1.publish(self._build_cmd(now, pos1, vel1, yaw1, tid1))

            rate.sleep()


def main():
    rospy.init_node("dual_waypoint_mission_publisher", anonymous=False)
    DualWaypointMissionPublisher().run()


if __name__ == "__main__":
    main()
