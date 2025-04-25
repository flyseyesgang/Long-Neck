// ─── src/BrandRecognition.cpp ──────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <sys/stat.h>

using std::placeholders::_1;

class BrandRecognition : public rclcpp::Node {
public:
  BrandRecognition()
  : Node("brand_recognition")
  {
    /* folder with ALL augmented crops
       e.g.   brands_aug/<Brand>/crop1_00.png
                          …/crop1_39.png                       */
    std::string dir = declare_parameter<std::string>(
        "brands_dir", "brands_aug");

    load_templates(dir);

    /* listen for 128×128 bottle-crops (mono or BGR) */
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "cropped_bottle", 10, std::bind(&BrandRecognition::cb,this,_1)
    );
    pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      "brand_detections", 10);
  }

private:
  /* ---------------------------------------------------------------------- */
  struct Template { std::string name; cv::Mat desc; };

  void load_templates(const std::string& root)
  {
    DIR* dp = opendir(root.c_str());
    if(!dp){
      RCLCPP_FATAL(get_logger(), "Cannot open brands_dir: %s", root.c_str());
      rclcpp::shutdown(); return;
    }
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    struct dirent* e;
    while((e=readdir(dp))){
      if(e->d_type != DT_DIR) continue;
      std::string brand = e->d_name;
      if(brand=="."||brand=="..") continue;

      std::string brand_dir = root + "/" + brand;
      DIR* d2 = opendir(brand_dir.c_str());
      if(!d2) continue;
      struct dirent* f;
      while((f=readdir(d2))){
        if(f->d_type!=DT_REG) continue;
        std::string path = brand_dir + "/" + f->d_name;
        cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if(img.empty()) continue;
        std::vector<cv::KeyPoint> kp; cv::Mat des;
        orb->detectAndCompute(img, {}, kp, des);
        if(!des.empty())
          templates_.push_back({brand, des});
      }
      closedir(d2);
    }
    closedir(dp);
    RCLCPP_INFO(get_logger(),"Loaded %zu templates from %s",
                templates_.size(), root.c_str());
  }

  /* ---------------------------------------------------------------------- */
  void cb(const sensor_msgs::msg::Image::SharedPtr m)
  {
    cv::Mat g;
    if(m->encoding=="mono8")
      g = cv_bridge::toCvCopy(m, "mono8")->image;
    else{
      cv::Mat bgr = cv_bridge::toCvCopy(m, "bgr8")->image;
      cv::cvtColor(bgr, g, cv::COLOR_BGR2GRAY);
    }
    if (g.cols != 128 || g.rows != 128)    {      // <── add
        cv::resize(g, g, {128, 128});   }
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> kp; cv::Mat des;
    orb->detectAndCompute(g, {}, kp, des);
    RCLCPP_INFO(get_logger(), "desc=%d×%d  kp=%zu",
            des.rows, des.cols, kp.size());
    if(des.empty()) return;

    cv::BFMatcher matcher{cv::NORM_HAMMING};
    vision_msgs::msg::Detection2DArray out;  out.header = m->header;

    for (auto &tpl : templates_) {
        std::vector<cv::DMatch> matches;
        matcher.match(des, tpl.desc, matches);

        double sum = 0;
        for (auto &d : matches) sum += d.distance;
        double avg   = sum / matches.size();   // lower = better
        double score = 100.0 - avg;            // our heuristic

        /*  👉 DEBUG  */
        RCLCPP_INFO(get_logger(),
            "[%s] avg=%.1f  score=%.1f",
            tpl.name.c_str(), avg, score);

        if (score > 85.0) {                       // tweak if needed
        vision_msgs::msg::Detection2D det;
        det.header  = m->header;
        det.results.resize(1);
        det.results[0].hypothesis.class_id = tpl.name;
        det.results[0].hypothesis.score    = score/100.;
        out.detections.push_back(det);
        break;                                  // first hit is fine
      }
    }
    if(!out.detections.empty()) pub_->publish(out);
  }

  /* data */
  std::vector<Template> templates_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
};
/* ------------------------------------------------------------------------ */
int main(int c,char**v){
  rclcpp::init(c,v);
  rclcpp::spin(std::make_shared<BrandRecognition>());
  rclcpp::shutdown();
  return 0;
}
