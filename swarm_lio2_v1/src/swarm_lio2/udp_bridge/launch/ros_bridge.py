#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from swarm_msgs.msg import QuadStatePub, GlobalExtrinsicStatus
from collections import defaultdict

class ROSBridge:
    def __init__(self):
        self.drone_id = rospy.get_param("~drone_id", 0)
        self.teammate_id = rospy.get_param("~teammate_id", -1)

        # Default behavior for simulation: forward every message once.
        # This avoids one-way relay when only one bridge process is running.
        self.relay_all = rospy.get_param("~relay_all", True)
        self.drop_self = rospy.get_param("~drop_self", False)
        self.filter_teammate_only = rospy.get_param("~filter_teammate_only", False)
        self.drop_out_of_order = rospy.get_param("~drop_out_of_order", True)
        self.max_delay_ms_quad = float(rospy.get_param("~max_delay_ms_quad", 400.0))
        self.max_delay_ms_ge = float(rospy.get_param("~max_delay_ms_ge", 400.0))
        self.queue_size = int(rospy.get_param("~queue_size", 50))

        self.local_quad_topic = rospy.get_param("~local_quad_topic", "/quadstate_to_teammate")
        self.local_ge_topic = rospy.get_param("~local_ge_topic", "/global_extrinsic_to_teammate")
        self.output_quad_topic = rospy.get_param("~output_quad_topic", "/quadstate_from_teammate")
        self.output_ge_topic = rospy.get_param("~output_ge_topic", "/global_extrinsic_from_teammate")

        self.pub_quad = rospy.Publisher(self.output_quad_topic, QuadStatePub, queue_size=self.queue_size)
        self.pub_ge = rospy.Publisher(self.output_ge_topic, GlobalExtrinsicStatus, queue_size=self.queue_size)

        rospy.Subscriber(self.local_quad_topic, QuadStatePub, self.quad_callback, queue_size=self.queue_size)
        rospy.Subscriber(self.local_ge_topic, GlobalExtrinsicStatus, self.ge_callback, queue_size=self.queue_size)

        rospy.loginfo(" -- [ROS Bridge]: Started")
        rospy.loginfo(" -- [DRONE ID]: %d", self.drone_id)
        rospy.loginfo(" -- [TEAMMATE ID]: %d", self.teammate_id)
        rospy.loginfo(" -- [relay_all]: %s", str(self.relay_all))
        rospy.loginfo(" -- [drop_self]: %s", str(self.drop_self))
        rospy.loginfo(" -- [filter_teammate_only]: %s", str(self.filter_teammate_only))
        rospy.loginfo(" -- [drop_out_of_order]: %s", str(self.drop_out_of_order))
        rospy.loginfo(" -- [max_delay_ms_quad]: %.1f", self.max_delay_ms_quad)
        rospy.loginfo(" -- [max_delay_ms_ge]: %.1f", self.max_delay_ms_ge)
        rospy.loginfo(" -- [queue_size]: %d", self.queue_size)
        rospy.loginfo(" -- [QUADSTATE] pub=%s sub=%s", self.output_quad_topic, self.local_quad_topic)
        rospy.loginfo(" -- [GLOBAL EXTRINSIC] pub=%s sub=%s", self.output_ge_topic, self.local_ge_topic)

        self.quad_sent_count = 0
        self.quad_fuse_count = 0
        self.ge_sent_count = 0
        self.ge_fuse_count = 0
        self.quad_count_by_id = defaultdict(int)
        self.ge_count_by_id = defaultdict(int)
        self.last_quad_stamp_by_id = {}
        self.last_ge_stamp_by_id = {}
        self.quad_drop_delay = 0
        self.quad_drop_order = 0
        self.ge_drop_delay = 0
        self.ge_drop_order = 0

        self.log_every_quad = int(rospy.get_param("~log_every_quad", 50))
        self.log_every_ge = int(rospy.get_param("~log_every_ge", 20))

    def _should_forward(self, msg_drone_id):
        if self.relay_all:
            if self.drop_self and msg_drone_id == self.drone_id:
                return False
            return True

        if self.filter_teammate_only and self.teammate_id >= 0:
            return msg_drone_id == self.teammate_id

        if self.drop_self:
            return msg_drone_id != self.drone_id

        return True

    @staticmethod
    def _delay_ms(stamp):
        if stamp is None or stamp.to_sec() <= 0.0:
            return -1.0
        return (rospy.Time.now() - stamp).to_sec() * 1000.0

    def _drop_out_of_order(self, stamp, drone_id, last_stamp_dict):
        if not self.drop_out_of_order:
            return False
        if stamp is None:
            return False
        stamp_sec = stamp.to_sec()
        if stamp_sec <= 0.0:
            return False
        last_stamp = last_stamp_dict.get(drone_id)
        if last_stamp is not None and stamp_sec <= last_stamp + 1e-6:
            return True
        last_stamp_dict[drone_id] = stamp_sec
        return False
    
    # ========== QUADSTATE ==========
    def quad_callback(self, msg):
        drone_id = msg.drone_id

        if not self._should_forward(drone_id):
            return

        if self._drop_out_of_order(msg.header.stamp, drone_id, self.last_quad_stamp_by_id):
            self.quad_drop_order += 1
            return

        delay_ms = self._delay_ms(msg.header.stamp)
        if self.max_delay_ms_quad > 0.0 and delay_ms >= 0.0 and delay_ms > self.max_delay_ms_quad:
            self.quad_drop_delay += 1
            return

        self.quad_fuse_count += 1
        self.quad_count_by_id[drone_id] += 1
        self.pub_quad.publish(msg)

        if drone_id == self.drone_id:
            self.quad_sent_count += 1

        per_uav_count = self.quad_count_by_id[drone_id]
        if self.log_every_quad > 0 and per_uav_count % self.log_every_quad == 0:
            rospy.loginfo(
                "[ROS Bridge] quadstate forwarded from UAV%d, per_uav=%d total=%d, delay %.2f ms, drop(delay/order)=%d/%d",
                drone_id,
                per_uav_count,
                self.quad_fuse_count,
                delay_ms,
                self.quad_drop_delay,
                self.quad_drop_order,
            )
    
    # ========== GLOBAL EXTRINSIC ==========
    def ge_callback(self, msg):
        drone_id = msg.drone_id

        if not self._should_forward(drone_id):
            return

        if self._drop_out_of_order(msg.header.stamp, drone_id, self.last_ge_stamp_by_id):
            self.ge_drop_order += 1
            return

        delay_ms = self._delay_ms(msg.header.stamp)
        if self.max_delay_ms_ge > 0.0 and delay_ms >= 0.0 and delay_ms > self.max_delay_ms_ge:
            self.ge_drop_delay += 1
            return

        self.ge_fuse_count += 1
        self.ge_count_by_id[drone_id] += 1
        self.pub_ge.publish(msg)

        if drone_id == self.drone_id:
            self.ge_sent_count += 1

        per_uav_count = self.ge_count_by_id[drone_id]
        if self.log_every_ge > 0 and per_uav_count % self.log_every_ge == 0:
            rospy.loginfo(
                "[ROS Bridge] global_extrinsic forwarded from UAV%d, per_uav=%d total=%d, delay %.2f ms, drop(delay/order)=%d/%d",
                drone_id,
                per_uav_count,
                self.ge_fuse_count,
                delay_ms,
                self.ge_drop_delay,
                self.ge_drop_order,
            )

if __name__ == "__main__":
    rospy.init_node("ros_bridge", anonymous=True)
    bridge = ROSBridge()
    rospy.spin()
