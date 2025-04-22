#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

class BrandNode : public rclcpp::Node
{
public:
  BrandNode() : Node("brand_node")
  {
    orb_ = cv::ORB::create();
    load_templates();

    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
      "/webcam/image_raw",  10,
      std::bind(&BrandNode::img_cb, this, std::placeholders::_1)
    );
    sub_box_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/bottle_boxes", 10,
      std::bind(&BrandNode::box_cb, this, std::placeholders::_1)
    );
    pub_ = create_publisher<std_msgs::msg::String>("bottle_brand", 10);
  }

private:
  void load_templates()
  {
    for (auto &b : {"corona","heineken"}) {
      cv::Mat t = cv::imread("/home/user/templates/"+std::string(b)+".png",
                             cv::IMREAD_GRAYSCALE);
      if (t.empty()) {
        RCLCPP_WARN(get_logger(), "Failed to load template for %s", b);
        continue;
      }
      std::vector<cv::KeyPoint> kp;
      cv::Mat des;
      orb_->detectAndCompute(t, {}, kp, des);
      tpl_desc_[b] = des;
    }
  }

  void img_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_img_ = msg;
  }

  void box_cb(const vision_msgs::msg::Detection2DArray::SharedPtr boxes)
  {
    if (!last_img_) return;
    auto frame = cv_bridge::toCvCopy(last_img_, "bgr8")->image;

    for (auto &det : boxes->detections) {
      // skip non‑bottle classes
      if (det.results.empty() ||
          det.results[0].hypothesis.class_id != "bottle")
        continue;

      auto &bb = det.bbox;
      cv::Rect r(
        bb.center.position.x - bb.size_x/2,
        bb.center.position.y - bb.size_y/2,
        bb.size_x, bb.size_y
      );
      r &= cv::Rect(0,0,frame.cols,frame.rows);
      cv::Mat crop = frame(r).clone();

      // ONLY ORB matching now:
      std::string brand = sift_orb_id(crop);
      if (brand == "Unknown") brand = ocr_id(crop);

      std_msgs::msg::String out;
      out.data = brand;
      pub_->publish(out);
    }
  }

  std::string sift_orb_id(const cv::Mat &img)
  {
    std::vector<cv::KeyPoint> kp;
    cv::Mat des;
    orb_->detectAndCompute(img, {}, kp, des);
    if (des.empty()) return "Unknown";

    cv::BFMatcher bf(cv::NORM_HAMMING);
    for (auto &p : tpl_desc_) {
      std::vector<cv::DMatch> matches;
      bf.match(p.second, des, matches);
      int good = 0;
      for (auto &d : matches) if (d.distance < 30) good++;
      if (good > 25) return p.first;
    }
    return "Unknown";
  }

  std::string ocr_id(const cv::Mat &img)
  {
    // TODO: integrate Tesseract or another OCR library here.
    // For now, always return Unknown.
    return "Unknown";
  }

  cv::Ptr<cv::ORB>                                orb_;
  std::map<std::string, cv::Mat>                  tpl_desc_;
  sensor_msgs::msg::Image::SharedPtr              last_img_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            sub_img_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_box_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrandNode>());
  rclcpp::shutdown();
  return 0;
}





// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp>
// #include <iostream>
// #include <vector>

// using namespace cv;
// using namespace std;

// void checkForMatch(const Mat& refImage, const Mat& targetImage, const string& targetName) {
//     Ptr<ORB> detector = ORB::create();
//     vector<KeyPoint> keypoints1, keypoints2;
//     Mat descriptors1, descriptors2;


//     detector->detectAndCompute(refImage, noArray(), keypoints1, descriptors1);
//     detector->detectAndCompute(targetImage, noArray(), keypoints2, descriptors2);


//     BFMatcher matcher(NORM_HAMMING);
//     vector<DMatch> matches;
//     matcher.match(descriptors1, descriptors2, matches);

//     double max_dist = 0; double min_dist = 100;
//     for (int i = 0; i < descriptors1.rows; i++) {
//         double dist = matches[i].distance;
//         if (dist < min_dist) min_dist = dist;
//         if (dist > max_dist) max_dist = dist;
//     }

//     vector<DMatch> good_matches;
//     for (int i = 0; i < descriptors1.rows; i++) {
//         if (matches[i].distance <= max(2*min_dist, 30.0)) {
//             good_matches.push_back(matches[i]);
//         }
//     }

//     double matched_percent = (double)good_matches.size() / keypoints1.size() * 100.0;

//     if (matched_percent < 20.0) {
//         cout << targetName << ": Corona bottle found." << endl;
//     } else {
//         cout << targetName << ": No Corona bottles found." << endl;
//     }
// }

// int main() {
//     string link = "/home/rhys/ros2_ws/src/RS2/Visual/";
//     Mat refImage = imread(link + "coronabottle.jpg", IMREAD_GRAYSCALE);
//     vector<string> targetImages = {link + "corona1.jpg", link + "corona2.jpg", link + "Sapporo.jpg"};

//     if (refImage.empty()) {
//         cout << "Error loading reference image." << endl;
//         return -1;
//     }

//     for (const string& targetName : targetImages) {
//         Mat targetImage = imread(targetName, IMREAD_GRAYSCALE);
//         if (targetImage.empty()) {
//             cout << "Error loading image: " << targetName << endl;
//             continue;
//         }
//         checkForMatch(refImage, targetImage, targetName);
//     }

//     return 0;
// }
