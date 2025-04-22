// src/hough_fusion_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <algorithm>
using namespace cv;
using namespace std;
using json = nlohmann::json;

class HoughFusionNode : public rclcpp::Node {
public:
  HoughFusionNode() : Node("hough_fusion_node") {
    // --- 1) declare & load parameters ---
    declare_parameter<std::string>("input_image", "");
    declare_parameter<std::string>("params_file", "best_params.json");
    get_parameter("input_image", input_image_);
    get_parameter("params_file", params_file_);
    if (!loadParams(params_file_))
      RCLCPP_WARN(get_logger(), "Using hard‐coded defaults");

    // --- 2) GUI + trackbars ---
    namedWindow(win_, WINDOW_AUTOSIZE);
    // trackbars drive p1_, p2_, r0_, r1_ live
    createTrackbar("Canny p1",    win_, &p1_, max_p1_);
    createTrackbar("Accu thresh", win_, &p2_, max_p2_);
    createTrackbar("minRadius",   win_, &r0_, max_r_);
    createTrackbar("maxRadius",   win_, &r1_, max_r_);

    // --- 3) publisher ---
    pub_ = create_publisher<geometry_msgs::msg::PoseArray>("bottle_poses", 10);

    // --- 4) either single‐image or live subscribe ---
    if (input_image_.empty()) {
      sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/webcam/image_raw", 10,
        [this](sensor_msgs::msg::Image::SharedPtr msg) {
          Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
          processFrame(frame, msg->header.stamp);
        });
    } else {
      Mat img = imread(input_image_);
      if (img.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to load %s", input_image_.c_str());
      } else {
        // run until any key
        while (rclcpp::ok()) {
          processFrame(img, builtin_interfaces::msg::Time());
          if (waitKey(30) >= 0) break;
        }
        rclcpp::shutdown();
      }
    }
  }

private:
  // --- Load best_params.json into dp_, minDist_, p1_, p2_, r0_, r1_ ---
  bool loadParams(const string &f) {
    ifstream in(f);
    if (!in.is_open()) return false;
    json j; in >> j;
    auto p = j["params"];
    dp_      = p.value("dp",      1.2 );
    minDist_ = p.value("minDist", 30.0);
    p1_      = p.value("param1", 93 );
    p2_      = p.value("param2", 51  );
    r0_      = p.value("minRadius", 6  );
    r1_      = p.value("maxRadius", 34 );
    RCLCPP_INFO(get_logger(),
      "Loaded Hough params dp=%.2f minDist=%.1f p1=%d p2=%d r=[%d..%d]",
      dp_, minDist_, p1_, p2_, r0_, r1_);
    return true;
  }

  // --- mask out the biggest quad (esky top) & return its 4 corners in `quad` ---
  bool maskEsky(const Mat &in, Mat &out, vector<Point2f> &quad) {
    Mat gray, edges;
    cvtColor(in, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5,5), 1.5);
    Canny(gray, edges, 50, 150);
    vector<vector<Point>> ctrs;
    findContours(edges, ctrs, RETR_LIST, CHAIN_APPROX_SIMPLE);

    double bestA = 0;
    vector<Point> best;
    for (auto &c : ctrs) {
      vector<Point> a;
      approxPolyDP(c, a, arcLength(c,true)*0.02, true);
      if (a.size()==4 && isContourConvex(a)) {
        double A = fabs(contourArea(a));
        if (A > bestA) { bestA = A; best = a; }
      }
    }
    if (best.empty()) return false;

    // fill mask
    Mat mask = Mat::zeros(in.size(), CV_8U);
    fillPoly(mask, vector<vector<Point>>{best}, Scalar(255));
    in.copyTo(out, mask);

    // convert best→quad of Point2f in TL,TR,BR,BL order
    // first find centroid and then sort by angle
    Moments m = moments(best);
    Point2f c(m.m10/m.m00, m.m01/m.m00);
    vector<pair<double,Point2f>> ang_pts;
    for (auto &pt : best) {
      double a = atan2(pt.y - c.y, pt.x - c.x);
      ang_pts.push_back({a, Point2f(pt)});
    }
    sort(ang_pts.begin(), ang_pts.end(),
      [](auto &l, auto &r){ return l.first < r.first; });
    // now they’re in CCW starting from -pi..pi; rotate so TL first
    // TL has smallest x+y
    int i0 = 0; double minsum = 1e9;
    for (int i=0;i<4;i++){
      double s = ang_pts[i].second.x + ang_pts[i].second.y;
      if (s<minsum){ minsum=s; i0=i; }
    }
    quad.resize(4);
    for (int k=0;k<4;k++){
      quad[k] = ang_pts[(i0+k)%4].second;
    }
    return true;
  }

  // --- the main per‐frame workhorse ---
  void processFrame(const Mat &frame, const builtin_interfaces::msg::Time &stamp) {
    Mat masked;
    vector<Point2f> quad;
    bool found = maskEsky(frame, masked, quad);

    // warp to top‑down
    Mat top = masked;
    const float W = 600, H = 400;
    if (found) {
      vector<Point2f> dst = {{0,0},{W,0},{W,H},{0,H}};
      Mat Hwarp = getPerspectiveTransform(quad, dst);
      warpPerspective(masked, top, Hwarp, Size(int(W),int(H)));
    }

    // grayscale + blur for Hough
    Mat gray;
    cvtColor(top, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9,9), 2.2);

    // multi‑threshold Hough
    vector<Vec3f> all;
    for (int t : {p2_-20, p2_-10, p2_, p2_+10, p2_+20}) {
      int TT = std::clamp(t, 1, max_p2_);
      vector<Vec3f> tmp;
      HoughCircles(gray, tmp, HOUGH_GRADIENT,
                   dp_, minDist_,
                   p1_, TT, r0_, r1_);
      all.insert(all.end(), tmp.begin(), tmp.end());
    }
    // simple de‑dup: keep if >10px from all kept
    vector<Vec3f> keep;
    for (auto &c : all) {
      bool ok=true;
      for (auto &k : keep) {
        if (norm(Point2f(c[0],c[1]) - Point2f(k[0],k[1])) < 10) {
          ok=false; break;
        }
      }
      if (ok) keep.push_back(c);
    }

    // draw & build PoseArray
    Mat disp = frame.clone();
    geometry_msgs::msg::PoseArray poses;
    poses.header.stamp = stamp;
    for (auto &c : keep) {
      // if warped—map back:
      Point2f center(c[0],c[1]);
      if (found) {
        // invert warp:
        vector<Point2f> inv = {center};
        Mat Hwarp = getPerspectiveTransform(quad,
                          vector<Point2f>{{0,0},{W,0},{W,H},{0,H}});
        Mat Hinv = Hwarp.inv();
        perspectiveTransform(inv, inv, Hinv);
        center = inv[0];
      }
      float r = c[2]* (found ? (frame.rows/H) : 1.0f);

      circle(disp, center, int(r), Scalar(0,255,0), 2);
      // publish dummy z=0
      geometry_msgs::msg::Pose p;
      p.position.x = center.x;
      p.position.y = center.y;
      p.position.z = 0;
      poses.poses.push_back(p);
    }

    imshow(win_, disp);
    pub_->publish(poses);
    waitKey(1);
  }

  // members
  string input_image_, params_file_, win_ = "Esky+Lids";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;

  // Hough params
  double dp_    = 1.2, minDist_ = 30;
  int    p1_    = 200, p2_      = 20,
         r0_    =  8, r1_      = 40;
  const int max_p1_ = 500, max_p2_ = 200, max_r_ = 200;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<HoughFusionNode>());
  rclcpp::shutdown();
  destroyAllWindows();
  return 0;
}
