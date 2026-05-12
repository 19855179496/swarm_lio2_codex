
import rospy
from nav_msgs.msg import Odometry


# quad_0的回调函数
def callback_quad0(msg):
    timestamp = msg.header.stamp.to_sec()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    
    data_line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
        timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
    
    with open("groundtruth_quad0_tum.txt", "a") as f:
        f.write(data_line)

# quad_1的回调函数
def callback_quad1(msg):
    timestamp = msg.header.stamp.to_sec()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    
    data_line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
        timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
    
    with open("groundtruth_quad1_tum.txt", "a") as f:
        f.write(data_line)

# quad_2的回调函数
def callback_quad2(msg):
    timestamp = msg.header.stamp.to_sec()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    
    data_line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
        timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
    
    with open("groundtruth_quad2_tum.txt", "a") as f:
        f.write(data_line)

if __name__ == "__main__":
    rospy.init_node("tum_recorder_groundtruth")
    
    # 注意：真值话题有下划线 "/quad_0/lidar_slam/odom"
    rospy.Subscriber("/quad_0/lidar_slam/odom", Odometry, callback_quad0)
    rospy.Subscriber("/quad_1/lidar_slam/odom", Odometry, callback_quad1)
    rospy.Subscriber("/quad_2/lidar_slam/odom", Odometry, callback_quad2)
    
    print("开始记录多机真值轨迹...")
    print("quad_0: groundtruth_quad0_tum.txt")
    print("quad_1: groundtruth_quad1_tum.txt")
    print("quad_2: groundtruth_quad2_tum.txt")
    
    rospy.spin()
