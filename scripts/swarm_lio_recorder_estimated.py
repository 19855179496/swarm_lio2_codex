
import rospy
from nav_msgs.msg import Odometry

# quad0的回调函数
def callback_quad0(msg):
    timestamp = msg.header.stamp.to_sec()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    
    data_line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
        timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
    
    with open("estimated_quad0_tum.txt", "a") as f:
        f.write(data_line)

# quad1的回调函数
def callback_quad1(msg):
    timestamp = msg.header.stamp.to_sec()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    
    data_line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
        timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
    
    with open("estimated_quad1_tum.txt", "a") as f:
        f.write(data_line)

# quad2的回调函数
def callback_quad2(msg):
    timestamp = msg.header.stamp.to_sec()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    
    data_line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
        timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
    
    with open("estimated_quad2_tum.txt", "a") as f:
        f.write(data_line)

if __name__ == "__main__":
    rospy.init_node("tum_recorder_estimated")
    
    # 为每架无人机订阅不同的话题，指定不同的回调函数
    rospy.Subscriber("/quad0/lidar_slam/odom", Odometry, callback_quad0)
    rospy.Subscriber("/quad1/lidar_slam/odom", Odometry, callback_quad1)
    rospy.Subscriber("/quad2/lidar_slam/odom", Odometry, callback_quad2)
    
    print("开始记录多机估计轨迹...")
    print("quad0: estimated_quad0_tum.txt")
    print("quad1: estimated_quad1_tum.txt")
    print("quad2: estimated_quad2_tum.txt")
    
    rospy.spin()
