#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
from swarm_msgs.msg import GlobalExtrinsicStatus


class GlobalExtrinsicRecorder:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/global_extrinsic_to_teammate")
        self.output_path = rospy.get_param("~output", "global_extrinsic_log.txt")
        self.append = bool(rospy.get_param("~append", True))

        output_dir = os.path.dirname(self.output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        mode = "a" if self.append else "w"
        self.fp = open(self.output_path, mode, buffering=1)
        self.fp.write(
            "# wall_time header_stamp from_id to_id "
            "rot_deg_x rot_deg_y rot_deg_z trans_x trans_y trans_z "
            "world_to_gravity_deg_x world_to_gravity_deg_y world_to_gravity_deg_z\n"
        )

        rospy.Subscriber(self.topic, GlobalExtrinsicStatus, self.callback, queue_size=1000)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("Recording global extrinsic from [%s] to [%s]", self.topic, self.output_path)

    def callback(self, msg):
        wall_time = rospy.Time.now().to_sec()
        stamp = msg.header.stamp.to_sec()
        from_id = int(msg.drone_id)

        for ext in msg.extrinsic:
            line = (
                f"{wall_time:.6f} {stamp:.6f} {from_id:d} {int(ext.teammate_id):d} "
                f"{float(ext.rot_deg[0]):.6f} {float(ext.rot_deg[1]):.6f} {float(ext.rot_deg[2]):.6f} "
                f"{float(ext.trans[0]):.6f} {float(ext.trans[1]):.6f} {float(ext.trans[2]):.6f} "
                f"{float(ext.world_to_gravity_deg[0]):.6f} {float(ext.world_to_gravity_deg[1]):.6f} {float(ext.world_to_gravity_deg[2]):.6f}\n"
            )
            self.fp.write(line)

    def on_shutdown(self):
        if hasattr(self, "fp") and self.fp:
            self.fp.flush()
            self.fp.close()


if __name__ == "__main__":
    rospy.init_node("global_extrinsic_recorder")
    GlobalExtrinsicRecorder()
    rospy.spin()
