// // src/EskyDetector.cpp
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>
// #include <opencv2/aruco/charuco.hpp>
// #include <opencv2/highgui.hpp>
// #include <limits>

// using std::placeholders::_1;

// class EskyDetector : public rclcpp::Node {
// public:
//   EskyDetector() : Node("esky_detector") {
//     subscription_ = create_subscription<sensor_msgs::msg::Image>(
//       "webcam/image_raw", 10, std::bind(&EskyDetector::imageCallback, this, _1)
//     );
//     found_pub_ = create_publisher<std_msgs::msg::Bool>("esky/found", 10);
//     pos_pub_   = create_publisher<geometry_msgs::msg::Point>("esky/position", 10);
//     centered_pub_ = create_publisher<std_msgs::msg::Bool>("esky/centered", 10);

//     dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

//     fx_ = declare_parameter("fx", 525.0);
//     fy_ = declare_parameter("fy", 525.0);
//     cx_ = declare_parameter("cx", 319.5);
//     cy_ = declare_parameter("cy", 239.5);
//     marker_size_0_1_ = declare_parameter("marker_size_top", 0.113);  // meters
//     marker_size_2_ = declare_parameter("marker_size_front", 0.170);  // meters
//   }

// private:
//   void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//     cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
//     std::vector<int> ids;
//     std::vector<std::vector<cv::Point2f>> corners;

//     cv::aruco::detectMarkers(frame, dictionary_, corners, ids);
//     std_msgs::msg::Bool flag, centered_flag;
//     geometry_msgs::msg::Point pos_msg;
//     flag.data = false;
//     centered_flag.data = false;
//     pos_msg.x = 0;
//     pos_msg.y = 0;
//     pos_msg.z = 0;

//     cv::aruco::drawDetectedMarkers(frame, corners, ids);

//     if (!ids.empty()) {
//       flag.data = true;

//       std::map<int, cv::Point2f> marker_centers;
//       std::map<int, double> marker_sizes_px;
//       std::map<int, std::vector<cv::Point2f>> marker_corners;
//       for (size_t i = 0; i < ids.size(); ++i) {
//         cv::Point2f center(0, 0);
//         for (auto& pt : corners[i]) center += pt;
//         center *= (1.0f / 4.0f);
//         marker_centers[ids[i]] = center;
//         marker_corners[ids[i]] = corners[i];

//         double avg_size = (float)(cv::norm(corners[i][0] - corners[i][1]) +
//                                   cv::norm(corners[i][1] - corners[i][2]) +
//                                   cv::norm(corners[i][2] - corners[i][3]) +
//                                   cv::norm(corners[i][3] - corners[i][0])) / 4.0;
//         marker_sizes_px[ids[i]] = avg_size;
//       }

//       if (marker_centers.count(0) && marker_centers.count(1)) {
//         auto top_center = 0.5 * (marker_centers[0] + marker_centers[1]);
//         double avg_px = 0.5 * (marker_sizes_px[0] + marker_sizes_px[1]);
//         computePosition(top_center, avg_px, marker_size_0_1_, 0, pos_msg);
//         centered_flag.data = true;
//       } else if (marker_centers.count(0)) {
//         computePosition(marker_centers[0], marker_sizes_px[0], marker_size_0_1_, 0, pos_msg);
//       } else if (marker_centers.count(1)) {
//         computePosition(marker_centers[1], marker_sizes_px[1], marker_size_0_1_, 1, pos_msg);
//       } else if (marker_centers.count(2)) {
//         computePosition(marker_centers[2], marker_sizes_px[2], marker_size_2_, 2, pos_msg);
//       }

//       std::string coord_text = "X: " + std::to_string(pos_msg.x).substr(0, 5) +
//                                ", Y: " + std::to_string(pos_msg.y).substr(0, 5) +
//                                ", Z: " + std::to_string(pos_msg.z).substr(0, 5);
//       cv::putText(frame, coord_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
//                   cv::Scalar(0, 255, 0), 2);
//     }

//     found_pub_->publish(flag);
//     centered_pub_->publish(centered_flag);
//     pos_pub_->publish(pos_msg);
//     cv::imshow("Aruco Detection Debug", frame);
//     cv::waitKey(1);
//   }

//   void computePosition(const cv::Point2f& pixel, double marker_px_size, double real_size_m, int id, geometry_msgs::msg::Point& P) {
//     if (marker_px_size < 1e-2) {
//       RCLCPP_WARN(this->get_logger(), "[ID %d] Marker size in pixels too small — skipping", id);
//       P.x = P.y = P.z = std::numeric_limits<double>::infinity();
//       return;
//     }
//     double Z = fx_ * real_size_m / marker_px_size;
//     P.x = (pixel.x - cx_) * Z / fx_;
//     P.y = (pixel.y - cy_) * Z / fy_;
//     P.z = Z;
//     RCLCPP_INFO(get_logger(), "Aruco ID %d → Position: [%.3f, %.3f, %.3f]", id, P.x, P.y, P.z);
//   }

//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr found_pub_;
//   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pos_pub_;
//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr centered_pub_;

//   cv::Ptr<cv::aruco::Dictionary> dictionary_;
//   double fx_, fy_, cx_, cy_;
//   double marker_size_0_1_, marker_size_2_;
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<EskyDetector>());
//   rclcpp::shutdown();
//   return 0;
// }

// File: src/EskyDetector.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <limits>

using std::placeholders::_1;

class EskyDetector : public rclcpp::Node {
public:
  EskyDetector() : Node("esky_detector") {
    // Subscribe to camera images
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/webcam/image_raw", 10, std::bind(&EskyDetector::imageCallback, this, _1));
    // Publishers for esky presence and position
    found_pub_ = create_publisher<std_msgs::msg::Bool>("/esky/found", 10);
    pos_pub_   = create_publisher<geometry_msgs::msg::Point>("/esky/position", 10);

    // ArUco dictionary and camera parameters
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    fx_ = declare_parameter("fx", 525.0);
    fy_ = declare_parameter("fy", 525.0);
    cx_ = declare_parameter("cx", 319.5);
    cy_ = declare_parameter("cy", 239.5);
    marker_size_top_   = declare_parameter("marker_size_top", 0.113);  // meters (markers 0,1)
    marker_size_front_ = declare_parameter("marker_size_front", 0.170); // meters (marker 2)
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS image to OpenCV
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    // Detect ArUco markers in the frame
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids);

    std_msgs::msg::Bool found_msg;
    found_msg.data = false;
    geometry_msgs::msg::Point pos_msg;
    pos_msg.x = pos_msg.y = pos_msg.z = 0.0;

    // Draw detected markers for visualization
    cv::aruco::drawDetectedMarkers(frame, corners, ids);

    if (!ids.empty()) {
      found_msg.data = true;
      // Calculate center and size in pixels for each detected marker
      std::map<int, cv::Point2f> marker_center;
      std::map<int, double> marker_size_px;
      for (size_t i = 0; i < ids.size(); ++i) {
        // Compute the center of marker i
        cv::Point2f center(0.f, 0.f);
        for (const auto& pt : corners[i]) {
          center += pt;
        }
        center *= 0.25f;
        marker_center[ids[i]] = center;
        // Average side length of marker i in pixels
        double size_px = (cv::norm(corners[i][0] - corners[i][1]) +
                          cv::norm(corners[i][1] - corners[i][2]) +
                          cv::norm(corners[i][2] - corners[i][3]) +
                          cv::norm(corners[i][3] - corners[i][0])) / 4.0;
        marker_size_px[ids[i]] = size_px;
      }
      // Determine position using available markers
      if (marker_center.count(0) && marker_center.count(1)) {
        // Both top markers detected: use midpoint of markers 0 and 1
        cv::Point2f top_center = 0.5f * (marker_center[0] + marker_center[1]);
        double avg_size = 0.5 * (marker_size_px[0] + marker_size_px[1]);
        computePosition(top_center, avg_size, marker_size_top_, pos_msg);
      } else if (marker_center.count(0)) {
        computePosition(marker_center[0], marker_size_px[0], marker_size_top_, pos_msg);
      } else if (marker_center.count(1)) {
        computePosition(marker_center[1], marker_size_px[1], marker_size_top_, pos_msg);
      } else if (marker_center.count(2)) {
        computePosition(marker_center[2], marker_size_px[2], marker_size_front_, pos_msg);
      }
      // Overlay the computed coordinates on the image (for debugging)
      char coord_text[100];
      std::snprintf(coord_text, sizeof(coord_text), "X: %.2f, Y: %.2f, Z: %.2f",
                    pos_msg.x, pos_msg.y, pos_msg.z);
      cv::putText(frame, coord_text, cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }

    // Publish whether the esky is found and its position
    found_pub_->publish(found_msg);
    pos_pub_->publish(pos_msg);

    // Show debug window with markers and position
    cv::imshow("Aruco Detection Debug", frame);
    cv::waitKey(1);
  }

  // Compute 3D position from marker center (pixel) and known size
  void computePosition(const cv::Point2f& pixel_center, double marker_px_size,
                       double marker_real_size, geometry_msgs::msg::Point& P) {
    if (marker_px_size < 1e-2) {
      RCLCPP_WARN(get_logger(), "Marker size too small for reliable position");
      P.x = P.y = P.z = std::numeric_limits<double>::infinity();
      return;
    }
    double Z = fx_ * marker_real_size / marker_px_size;
    P.x = (pixel_center.x - cx_) * Z / fx_;
    P.y = (pixel_center.y - cy_) * Z / fy_;
    P.z = Z;
    RCLCPP_INFO(get_logger(), "Esky position [%.3f, %.3f, %.3f] (m)", P.x, P.y, P.z);
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr found_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pos_pub_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  double fx_, fy_, cx_, cy_;
  double marker_size_top_, marker_size_front_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EskyDetector>());
  rclcpp::shutdown();
  return 0;
}
