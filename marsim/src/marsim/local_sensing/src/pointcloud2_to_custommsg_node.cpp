#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>

class PointCloud2ToCustomMsgNode {
public:
  PointCloud2ToCustomMsgNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("input_topic", input_topic_, "/quad0_pcl_render_node/sensor_cloud");
    pnh_.param<std::string>("output_topic", output_topic_, "sensor_custom");
    pnh_.param<double>("scan_period", scan_period_, 0.1);
    pnh_.param<int>("lidar_id", lidar_id_, 1);
    pnh_.param<int>("line_id", line_id_, 0);
    pnh_.param<int>("tag", tag_, 0x10);

    sub_cloud_ = nh_.subscribe(input_topic_, 10, &PointCloud2ToCustomMsgNode::cloudCallback, this);
    pub_custom_ = nh_.advertise<livox_ros_driver::CustomMsg>(output_topic_, 10);

    ROS_INFO("[pc2_to_custommsg] input: %s, output: %s", input_topic_.c_str(), output_topic_.c_str());
  }

private:
  static inline uint8_t intensityToReflectivity(float intensity) {
    // If intensity is normalized to [0,1], map it to [0,255]. Otherwise clamp directly.
    float scaled = intensity;
    if (scaled >= 0.0f && scaled <= 1.0f) {
      scaled *= 255.0f;
    }
    const int v = static_cast<int>(std::lround(scaled));
    return static_cast<uint8_t>(std::max(0, std::min(255, v)));
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    livox_ros_driver::CustomMsg out;
    out.header = msg->header;
    out.timebase = static_cast<uint64_t>(msg->header.stamp.toNSec());
    out.lidar_id = static_cast<uint8_t>(std::max(0, std::min(255, lidar_id_)));
    out.rsvd[0] = 0;
    out.rsvd[1] = 0;
    out.rsvd[2] = 0;

    const std::size_t n = cloud.points.size();
    out.points.reserve(n);

    const double scan_period_ns = std::max(0.0, scan_period_) * 1e9;
    for (std::size_t i = 0; i < n; ++i) {
      const auto &pt = cloud.points[i];
      livox_ros_driver::CustomPoint cpt;
      cpt.x = pt.x;
      cpt.y = pt.y;
      cpt.z = pt.z;
      cpt.reflectivity = intensityToReflectivity(pt.intensity);
      cpt.tag = static_cast<uint8_t>(std::max(0, std::min(255, tag_)));
      cpt.line = static_cast<uint8_t>(std::max(0, std::min(255, line_id_)));

      if (n > 1) {
        const double ratio = static_cast<double>(i) / static_cast<double>(n - 1);
        cpt.offset_time = static_cast<uint32_t>(ratio * scan_period_ns);
      } else {
        cpt.offset_time = 0;
      }

      out.points.push_back(cpt);
    }

    out.point_num = static_cast<uint32_t>(out.points.size());
    pub_custom_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_cloud_;
  ros::Publisher pub_custom_;

  std::string input_topic_;
  std::string output_topic_;
  double scan_period_;
  int lidar_id_;
  int line_id_;
  int tag_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud2_to_custommsg_node");
  PointCloud2ToCustomMsgNode node;
  ros::spin();
  return 0;
}
