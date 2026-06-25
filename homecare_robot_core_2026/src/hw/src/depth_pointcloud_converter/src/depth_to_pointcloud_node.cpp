// depth_to_pointcloud_node.cpp
//
// InuSensor 드라이버가 발행하는 depth 이미지(sensor_msgs/Image, 16UC1, mm)와
// CameraInfo를 받아 sensor_msgs/PointCloud2 (XYZ)로 변환해 발행하는 독립 노드.
//
// - 입력 QoS: SensorDataQoS(best_effort) → 드라이버의 best_effort 발행과 매칭
// - 출력 QoS: Reliable(depth 5) → RViz 기본 QoS(Reliable)에서 바로 보이도록
// - 출력 프레임: depth 이미지의 frame_id(inusensor_depth, 광학 좌표계)를 그대로 사용
//
// 드라이버 코드는 전혀 수정하지 않는다.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

using std::placeholders::_1;

namespace depth_pointcloud_converter
{

class DepthToPointCloudNode : public rclcpp::Node
{
public:
  explicit DepthToPointCloudNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("depth_to_pointcloud_node", options)
  {
    depth_topic_       = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info");
    output_topic_      = declare_parameter<std::string>("output_topic", "/camera/depth/points");
    // 비워두면 depth 이미지 header.frame_id를 그대로 사용한다(권장).
    output_frame_      = declare_parameter<std::string>("output_frame", "");
    // 16UC1 값이 mm 이므로 m 변환 계수 0.001
    depth_scale_       = declare_parameter<double>("depth_scale", 0.001);
    range_min_         = declare_parameter<double>("range_min", 0.1);
    range_max_         = declare_parameter<double>("range_max", 5.0);
    // 성능을 위한 다운샘플링(1 = 모든 픽셀 사용)
    row_step_          = std::max(1, static_cast<int>(declare_parameter<int>("row_step", 1)));
    col_step_          = std::max(1, static_cast<int>(declare_parameter<int>("col_step", 1)));

    auto sensor_qos = rclcpp::SensorDataQoS();  // best_effort, keep_last, depth 5

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      std::bind(&DepthToPointCloudNode::infoCallback, this, _1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, sensor_qos,
      std::bind(&DepthToPointCloudNode::depthCallback, this, _1));

    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(5));

    RCLCPP_INFO(get_logger(), "depth_to_pointcloud_node started");
    RCLCPP_INFO(get_logger(), "  depth_topic       : %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  camera_info_topic : %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  output_topic      : %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  depth_scale       : %.4f (16UC1 -> m)", depth_scale_);
    RCLCPP_INFO(get_logger(), "  range [m]         : %.2f ~ %.2f", range_min_, range_max_);
    RCLCPP_INFO(get_logger(), "  downsample (r,c)  : %d, %d", row_step_, col_step_);
  }

private:
  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(info_mutex_);
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    have_info_ = (fx_ != 0.0 && fy_ != 0.0);
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    double fx, fy, cx, cy;
    {
      std::lock_guard<std::mutex> lk(info_mutex_);
      if (!have_info_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for valid camera_info on %s ...", camera_info_topic_.c_str());
        return;
      }
      fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    }

    if (msg->encoding != "16UC1") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Unexpected depth encoding '%s' (expected 16UC1) - skipping", msg->encoding.c_str());
      return;
    }

    const uint32_t width  = msg->width;
    const uint32_t height = msg->height;
    if (width == 0 || height == 0 || msg->data.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    if (!output_frame_.empty()) {
      cloud.header.frame_id = output_frame_;
    }
    cloud.height = 1;            // unorganized (NaN 없이 유효 포인트만)
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    // 최악의 경우(모든 픽셀 유효) 크기로 미리 확보 후, 끝에서 실제 개수로 축소.
    const size_t max_points =
      static_cast<size_t>((height + row_step_ - 1) / row_step_) *
      static_cast<size_t>((width  + col_step_ - 1) / col_step_);
    modifier.resize(max_points);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    const float scale = static_cast<float>(depth_scale_);
    const float rmin  = static_cast<float>(range_min_);
    const float rmax  = static_cast<float>(range_max_);
    const float fxf = static_cast<float>(fx);
    const float fyf = static_cast<float>(fy);
    const float cxf = static_cast<float>(cx);
    const float cyf = static_cast<float>(cy);

    size_t count = 0;
    for (uint32_t v = 0; v < height; v += row_step_) {
      const uint16_t * depth_row =
        reinterpret_cast<const uint16_t *>(msg->data.data() + static_cast<size_t>(v) * msg->step);
      for (uint32_t u = 0; u < width; u += col_step_) {
        const uint16_t raw = depth_row[u];
        if (raw == 0) {
          continue;  // 무효 픽셀
        }
        const float z = static_cast<float>(raw) * scale;
        if (z < rmin || z > rmax) {
          continue;
        }
        // 광학 좌표계: x=오른쪽, y=아래, z=정면
        *it_x = (static_cast<float>(u) - cxf) * z / fxf;
        *it_y = (static_cast<float>(v) - cyf) * z / fyf;
        *it_z = z;
        ++it_x; ++it_y; ++it_z;
        ++count;
      }
    }

    modifier.resize(count);  // 실제 유효 포인트 수로 축소(width/row_step 자동 갱신)

    points_pub_->publish(cloud);
  }

  // params
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double depth_scale_{0.001};
  double range_min_{0.1};
  double range_max_{5.0};
  int row_step_{1};
  int col_step_{1};

  // camera intrinsics (from CameraInfo)
  std::mutex info_mutex_;
  bool have_info_{false};
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
};

}  // namespace depth_pointcloud_converter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<depth_pointcloud_converter::DepthToPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
