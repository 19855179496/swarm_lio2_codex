
import rospy
import math
import numpy as np
from quadrotor_msgs.msg import PositionCommand

def figure_eight_publisher():
    rospy.init_node('figure_eight_commander', anonymous=True)
    uav_id = "0"
    pub = rospy.Publisher(f'/quad_{uav_id}/planning/pos_cmd', PositionCommand, queue_size=10)
    rate = rospy.Rate(30)  # 提高控制频率
    
    # 8字轨迹参数
    A = 2.0      # 轨迹大小
    omega = 0.2 # 角速度（比圆周更慢）
    Z = 2.0      # 飞行高度
    
    start_time = rospy.get_time()
    rospy.loginfo("开始8字轨迹控制...")

    while not rospy.is_shutdown():
        t = rospy.get_time() - start_time
        
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = f"quad{uav_id}/world"
        
        # 8字轨迹参数方程
        # x = A * sin(omega * t)
        # y = A * sin(omega * t) * cos(omega * t)
        cmd.position.x = A * math.sin(omega * t)
        cmd.position.y = A * math.sin(omega * t) * math.cos(omega * t)
        cmd.position.z = Z
        
        # 速度计算（导数）
        cmd.velocity.x = A * omega * math.cos(omega * t)
        cmd.velocity.y = A * omega * (math.cos(omega * t)**2 - math.sin(omega * t)**2)
        cmd.velocity.z = 0.0
        
        # 加速度计算（用于更平滑的运动）
        cmd.acceleration.x = -A * omega**2 * math.sin(omega * t)
        cmd.acceleration.y = -4 * A * omega**2 * math.sin(omega * t) * math.cos(omega * t)
        cmd.acceleration.z = 0.0
        
        # 偏航角控制：始终朝向运动方向
        cmd.yaw = math.atan2(cmd.velocity.y, cmd.velocity.x)
        
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        figure_eight_publisher()
    except rospy.ROSInterruptException:
        pass
