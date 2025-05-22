// src/EskyDetector.cpp
// src/EskyDetector.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <random>

using namespace std;
using std::placeholders::_1;

class EskyDetector : public rclcpp::Node {
public:
  EskyDetector()
  : Node("esky_detector"), frame_count_(0),
    gen_(std::random_device{}()), noise_dist_(-0.001, 0.001)
  {
    // Threshold & morphology params
    declare_parameter<int>("threshold_block_size", 11);
    declare_parameter<double>("threshold_C", 2.0);
    declare_parameter<int>("morph_kernel_size", 5);
    // Detection smoothing
    declare_parameter<double>("min_area_ratio", 0.05);
    declare_parameter<int>("smoothing_frames", 5);
    // Real-world esky dims (top face) in meters
    declare_parameter<double>("esky_width", 0.395);
    declare_parameter<double>("esky_height", 0.260);
    // Camera intrinsics
    declare_parameter<double>("fx", 525.0);
    declare_parameter<double>("fy", 525.0);
    declare_parameter<double>("cx", 319.5);
    declare_parameter<double>("cy", 239.5);

    // Subscriptions & publishers
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "webcam/image_raw", 10,
      std::bind(&EskyDetector::imageCallback, this, _1)
    );
    ok_pub_  = create_publisher<std_msgs::msg::Bool>("esky/detected", 10);
    pos_pub_ = create_publisher<geometry_msgs::msg::Point>("esky/position", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert to OpenCV
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // 1) Grayscale + adaptive threshold
    cv::Mat gray, thresh;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    int bs = get_parameter("threshold_block_size").as_int();
    double C = get_parameter("threshold_C").as_double();
    cv::adaptiveThreshold(gray, thresh, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV,
                          bs, C);
    // 2) Closing
    int ksz = get_parameter("morph_kernel_size").as_int();
    cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ksz,ksz)));

    // 3) Find quad
    vector<vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double minArea = img.cols * img.rows * get_parameter("min_area_ratio").as_double();
    double maxArea = 0;
    vector<cv::Point> bestPoly;
    for(auto &c: contours) {
      double area = cv::contourArea(c);
      if(area < minArea) continue;
      vector<cv::Point> poly;
      cv::approxPolyDP(c, poly, 0.02*cv::arcLength(c,true), true);
      if(poly.size()==4 && cv::isContourConvex(poly) && area>maxArea) {
        bestPoly = poly; maxArea = area;
      }
    }
    // 4) Smooth
    bool found = !bestPoly.empty();
    int smooth = get_parameter("smoothing_frames").as_int();
    frame_count_ = found ? frame_count_+1 : max(0, frame_count_-1);
    bool stable = (frame_count_ >= smooth);
    // Publish found
    std_msgs::msg::Bool flag; flag.data = stable;
    ok_pub_->publish(flag);
    if(!stable) { RCLCPP_INFO(get_logger(), "Esky not yet stable"); return; }

    // 5) Pixel bounds
    int minx=INT_MAX, maxx=0, miny=INT_MAX, maxy=0;
    for(auto &p: bestPoly) {
      minx=min(minx,p.x); maxx=max(maxx,p.x);
      miny=min(miny,p.y); maxy=max(maxy,p.y);
    }
    int pix_w = max(1, maxx-minx);
    int pix_h = max(1, maxy-miny);

    // 6) Depth via width & height
    double fx = get_parameter("fx").as_double();
    double fy = get_parameter("fy").as_double();
    double real_w = get_parameter("esky_width").as_double();
    double real_h = get_parameter("esky_height").as_double();
    double Z_w = fx * real_w / double(pix_w);
    double Z_h = fy * real_h / double(pix_h);
    double Z = 0.5*(Z_w+Z_h) + noise_dist_(gen_);

    // 7) Lateral offsets
    double cx = get_parameter("cx").as_double();
    double cy = get_parameter("cy").as_double();
    double center_x = (minx+maxx)/2.0;
    double center_y = (miny+maxy)/2.0;
    double X = (center_x - cx) * Z / fx;
    double Y = (center_y - cy) * Z / fy;

    // 8) Publish position
    geometry_msgs::msg::Point P; P.x=X; P.y=Y; P.z=Z;
    pos_pub_->publish(P);
    RCLCPP_INFO(get_logger(), "Esky at x=%.3f m, y=%.3f m, z=%.3f m", X, Y, Z);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   ok_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pos_pub_;
  int frame_count_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> noise_dist_;
};

int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<EskyDetector>());
  rclcpp::shutdown();
  return 0;
}
