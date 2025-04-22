#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <functional>

using namespace cv;
using namespace std;
using json = nlohmann::json;

// mask out everything except the largest 4‑point polygon (the esky top)
static Mat mask_esky(const Mat& in) {
  Mat gray, edges;
  cvtColor(in, gray, COLOR_BGR2GRAY);
  GaussianBlur(gray, gray, Size(5,5), 1.5);
  Canny(gray, edges, 50, 150);

  vector<vector<Point>> ctrs;
  findContours(edges, ctrs, RETR_LIST, CHAIN_APPROX_SIMPLE);

  double bestA=0; vector<Point> bestQ;
  for (auto &c : ctrs) {
    vector<Point> a;
    approxPolyDP(c, a, arcLength(c,true)*0.02, true);
    if (a.size()==4 && isContourConvex(a)) {
      double A = fabs(contourArea(a));
      if (A>bestA) { bestA=A; bestQ=a; }
    }
  }
  if (bestQ.empty()) return in;
  Mat mask = Mat::zeros(in.size(), CV_8U);
  fillPoly(mask, vector<vector<Point>>{bestQ}, Scalar(255));
  Mat out; in.copyTo(out,mask);
  return out;
}

class LocationNode : public rclcpp::Node {
public:
  LocationNode(): Node("location_node") {
    // 1) load best_params.json
    dp_ = 1.2; minDist_ = 30; p1_=200; p2_=20; r0_=8; r1_=40;
    ifstream ifs("best_params.json");
    if (ifs) {
      json j; ifs >> j; auto&p = j["params"];
      dp_      = p["dp"].get<double>();
      minDist_ = p["minDist"].get<double>();
      p1_      = p["param1"].get<int>();
      p2_      = p["param2"].get<int>();
      r0_      = p["minRadius"].get<int>();
      r1_      = p["maxRadius"].get<int>();
      RCLCPP_INFO(get_logger(),
        "Loaded Hough dp=%.2f minDist=%.1f p1=%d p2=%d r=[%d..%d]",
        dp_,minDist_,p1_,p2_,r0_,r1_);
    } else {
      RCLCPP_WARN(get_logger(),
        "Could not open best_params.json; using defaults");
    }

    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
      "/webcam/image_raw", 10,
      bind(&LocationNode::img_cb, this, placeholders::_1));
    sub_box_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/bottle_boxes", 10,
      bind(&LocationNode::box_cb, this, placeholders::_1));
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "bottle_poses",10);

    namedWindow("Lid Detection", WINDOW_AUTOSIZE);
  }

private:
  void img_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_img_ = msg;
  }

  void box_cb(const vision_msgs::msg::Detection2DArray::SharedPtr boxes) {
    if (!last_img_) return;
    Mat frame = cv_bridge::toCvCopy(last_img_,"bgr8")->image;

    // mask esky top
    Mat top = mask_esky(frame);

    // HoughCircles
    Mat gray; cvtColor(top, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9,9), 2.2);
    vector<Vec3f> circles;
    HoughCircles(gray, circles, HOUGH_GRADIENT,
                 dp_, minDist_, p1_, p2_, r0_, r1_);

    // draw & display
    for (auto &c : circles) {
      circle(frame, Point(cvRound(c[0]),cvRound(c[1])),
             cvRound(c[2]), Scalar(0,255,0), 2);
    }
    imshow("Lid Detection", frame); waitKey(1);

    // publish PoseArray (empty poses for now)
    geometry_msgs::msg::PoseArray out;
    out.header = boxes->header;
    for (size_t i=0; i<circles.size(); ++i)
      out.poses.push_back(geometry_msgs::msg::Pose());
    pub_pose_->publish(out);
  }

  // parameters
  double dp_, minDist_;
  int    p1_, p2_, r0_, r1_;

  // state + ROS handles
  sensor_msgs::msg::Image::SharedPtr last_img_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_box_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<LocationNode>());
  rclcpp::shutdown();
  destroyAllWindows();
  return 0;
}
