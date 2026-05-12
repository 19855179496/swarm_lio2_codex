#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random

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
        raise ValueError("param {} must be [x,y,z]".format(name))
    return (float(raw[0]), float(raw[1]), float(raw[2]))


def _region_param(name, default_value):
    raw = rospy.get_param(name, default_value)
    if not isinstance(raw, list) or len(raw) != 6:
        raise ValueError("param {} must be [xmin,xmax,ymin,ymax,zmin,zmax]".format(name))
    xmin, xmax, ymin, ymax, zmin, zmax = [float(x) for x in raw]
    if xmin >= xmax or ymin >= ymax or zmin >= zmax:
        raise ValueError("param {} invalid bounds".format(name))
    return (xmin, xmax, ymin, ymax, zmin, zmax)


def _norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


class RandomWalker:
    def __init__(self, name, init_pos, region, speed_min, speed_max, min_step, hold_time):
        self.name = name
        self.pos = init_pos
        self.region = region
        self.speed_min = speed_min
        self.speed_max = speed_max
        self.min_step = min_step
        self.hold_time = hold_time
        self.speed = self._sample_speed()
        self.target = self._sample_target(self.pos)
        self.hold_until = 0.0

    def _sample_speed(self):
        return random.uniform(self.speed_min, self.speed_max)

    def _sample_target(self, ref):
        xmin, xmax, ymin, ymax, zmin, zmax = self.region
        for _ in range(100):
            t = (
                random.uniform(xmin, xmax),
                random.uniform(ymin, ymax),
                random.uniform(zmin, zmax),
            )
            if _norm3((t[0] - ref[0], t[1] - ref[1], t[2] - ref[2])) >= self.min_step:
                return t
        return (
            random.uniform(xmin, xmax),
            random.uniform(ymin, ymax),
            random.uniform(zmin, zmax),
        )

    def step(self, dt, now_sec, reach_radius):
        if now_sec < self.hold_until:
            return self.pos, (0.0, 0.0, 0.0)

        dx = self.target[0] - self.pos[0]
        dy = self.target[1] - self.pos[1]
        dz = self.target[2] - self.pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < reach_radius:
            self.pos = self.target
            self.speed = self._sample_speed()
            self.target = self._sample_target(self.pos)
            self.hold_until = now_sec + self.hold_time
            return self.pos, (0.0, 0.0, 0.0)

        ux, uy, uz = dx / dist, dy / dist, dz / dist
        vx, vy, vz = self.speed * ux, self.speed * uy, self.speed * uz
        step_len = min(self.speed * dt, dist)
        self.pos = (
            self.pos[0] + ux * step_len,
            self.pos[1] + uy * step_len,
            self.pos[2] + uz * step_len,
        )
        return self.pos, (vx, vy, vz)


class TripleRandomAreaMissionPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.start_delay = float(rospy.get_param("~start_delay", 3.0))
        self.wait_for_start_signal = _as_bool(rospy.get_param("~wait_for_start_signal", True))
        self.start_topic = rospy.get_param("~start_topic", "/start_figure8")
        self.publish_yaw_tangent = _as_bool(rospy.get_param("~publish_yaw_tangent", True))
        self.seed = int(rospy.get_param("~seed", 20260512))
        random.seed(self.seed)

        self.topic_uav0 = rospy.get_param("~topic_uav0", "/quad_0/planning/pos_cmd")
        self.topic_uav1 = rospy.get_param("~topic_uav1", "/quad_1/planning/pos_cmd")
        self.topic_uav2 = rospy.get_param("~topic_uav2", "/quad_2/planning/pos_cmd")

        self.hold_uav0 = _vec3_list_param("~hold_uav0", [0.0, 0.0, 1.2])
        self.hold_uav1 = _vec3_list_param("~hold_uav1", [2.0, 2.0, 1.2])
        self.hold_uav2 = _vec3_list_param("~hold_uav2", [-2.0, 2.0, 1.2])
        self.hold_yaw_uav0 = float(rospy.get_param("~hold_yaw_uav0", 0.0))
        self.hold_yaw_uav1 = float(rospy.get_param("~hold_yaw_uav1", 0.0))
        self.hold_yaw_uav2 = float(rospy.get_param("~hold_yaw_uav2", 0.0))

        self.region_uav0 = _region_param("~region_uav0", [-3.0, 3.0, -3.0, 3.0, 1.0, 2.0])
        self.region_uav1 = _region_param("~region_uav1", [-1.0, 5.0, -1.0, 5.0, 1.0, 2.2])
        self.region_uav2 = _region_param("~region_uav2", [-5.0, 1.0, -1.0, 5.0, 1.0, 2.4])

        self.speed_min = float(rospy.get_param("~speed_min", 0.4))
        self.speed_max = float(rospy.get_param("~speed_max", 1.2))
        self.min_target_step = float(rospy.get_param("~min_target_step", 0.8))
        self.reach_radius = float(rospy.get_param("~reach_radius", 0.25))
        self.hold_time = float(rospy.get_param("~hold_time", 0.5))
        self.min_pair_distance = float(rospy.get_param("~min_pair_distance", 1.0))

        self.kx = [float(x) for x in rospy.get_param("~kx", [7.0, 7.0, 6.2])]
        self.kv = [float(x) for x in rospy.get_param("~kv", [4.0, 4.0, 4.0])]

        self.pub_uav0 = rospy.Publisher(self.topic_uav0, PositionCommand, queue_size=20)
        self.pub_uav1 = rospy.Publisher(self.topic_uav1, PositionCommand, queue_size=20)
        self.pub_uav2 = rospy.Publisher(self.topic_uav2, PositionCommand, queue_size=20)

        self.started = not self.wait_for_start_signal
        self.start_ref_time = rospy.Time.now().to_sec()
        if self.wait_for_start_signal:
            rospy.Subscriber(self.start_topic, Bool, self._start_callback, queue_size=1)

        self.walkers = [
            RandomWalker("uav0", self.hold_uav0, self.region_uav0, self.speed_min, self.speed_max, self.min_target_step, self.hold_time),
            RandomWalker("uav1", self.hold_uav1, self.region_uav1, self.speed_min, self.speed_max, self.min_target_step, self.hold_time),
            RandomWalker("uav2", self.hold_uav2, self.region_uav2, self.speed_min, self.speed_max, self.min_target_step, self.hold_time),
        ]

        self.prev_loop_time = rospy.Time.now().to_sec()
        rospy.loginfo(
            "[triple_random_area_mission] seed=%d topics=(%s,%s,%s) wait_for_start_signal=%s",
            self.seed, self.topic_uav0, self.topic_uav1, self.topic_uav2, str(self.wait_for_start_signal)
        )

    def _start_callback(self, msg):
        if (not self.started) and msg.data:
            self.started = True
            self.start_ref_time = rospy.Time.now().to_sec()
            self.prev_loop_time = self.start_ref_time
            rospy.loginfo("[triple_random_area_mission] received start signal")

    def _build_cmd(self, now, pos, vel, yaw, trajectory_id):
        msg = PositionCommand()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.position.x, msg.position.y, msg.position.z = pos
        msg.velocity.x, msg.velocity.y, msg.velocity.z = vel
        msg.acceleration.x = msg.acceleration.y = msg.acceleration.z = 0.0
        msg.jerk.x = msg.jerk.y = msg.jerk.z = 0.0
        msg.angular_velocity.x = msg.angular_velocity.y = msg.angular_velocity.z = 0.0
        msg.attitude.x = msg.attitude.y = msg.attitude.z = 0.0
        msg.thrust.x = msg.thrust.y = msg.thrust.z = 0.0
        msg.yaw = yaw
        msg.yaw_dot = 0.0
        msg.vel_norm = _norm3(vel)
        msg.acc_norm = 0.0
        msg.kx = list(self.kx)
        msg.kv = list(self.kv)
        msg.trajectory_id = int(max(1, trajectory_id))
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return msg

    @staticmethod
    def _dist(a, b):
        dx, dy, dz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _apply_simple_safety(self, positions, velocities):
        for i in range(3):
            for j in range(i + 1, 3):
                if self._dist(positions[i], positions[j]) < self.min_pair_distance:
                    velocities[j] = (0.0, 0.0, 0.0)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            now_sec = now.to_sec()

            if not self.started:
                positions = [self.hold_uav0, self.hold_uav1, self.hold_uav2]
                velocities = [(0.0, 0.0, 0.0)] * 3
                yaws = [self.hold_yaw_uav0, self.hold_yaw_uav1, self.hold_yaw_uav2]
            else:
                if now_sec - self.start_ref_time < self.start_delay:
                    positions = [self.hold_uav0, self.hold_uav1, self.hold_uav2]
                    velocities = [(0.0, 0.0, 0.0)] * 3
                    yaws = [self.hold_yaw_uav0, self.hold_yaw_uav1, self.hold_yaw_uav2]
                else:
                    dt = max(1e-3, min(0.2, now_sec - self.prev_loop_time))
                    positions = []
                    velocities = []
                    for walker in self.walkers:
                        p, v = walker.step(dt, now_sec, self.reach_radius)
                        positions.append(p)
                        velocities.append(v)
                    self._apply_simple_safety(positions, velocities)

                    yaws = []
                    for i in range(3):
                        vx, vy = velocities[i][0], velocities[i][1]
                        if self.publish_yaw_tangent and (abs(vx) > 1e-3 or abs(vy) > 1e-3):
                            yaws.append(math.atan2(vy, vx))
                        else:
                            yaws.append(0.0)

            self.pub_uav0.publish(self._build_cmd(now, positions[0], velocities[0], yaws[0], 1))
            self.pub_uav1.publish(self._build_cmd(now, positions[1], velocities[1], yaws[1], 1))
            self.pub_uav2.publish(self._build_cmd(now, positions[2], velocities[2], yaws[2], 1))

            self.prev_loop_time = now_sec
            rate.sleep()


def main():
    rospy.init_node("triple_random_area_mission_publisher", anonymous=False)
    TripleRandomAreaMissionPublisher().run()


if __name__ == "__main__":
    main()
