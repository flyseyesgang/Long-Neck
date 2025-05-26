// ─── src/CropNode.cpp ────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CropNode : public rclcpp::Node {
public:
  CropNode() : Node("crop_node")
  {
    /* subscribe to the original camera image */
    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
        "webcam/image_raw",        // or whatever topic your camera publishes
        10,
        [this](sensor_msgs::msg::Image::SharedPtr msg){ last_ = std::move(msg); });

    /* subscribe to YOLO detections that contain bboxes                 *
     * (Detector publishes sensor_msgs/Image but you turned them into   *
     *  vision_msgs/Detection2DArray in the fusion stage)               */
    sub_det_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        "yolo_detections",
        10,
        std::bind(&CropNode::onDet, this, std::placeholders::_1));

    /* publisher for 128×128 crops */
    pub_crop_ = create_publisher<sensor_msgs::msg::Image>("cropped_bottle", 10);
  }
private:
  void onDet(const vision_msgs::msg::Detection2DArray::SharedPtr dets)
  {
    if (!last_) return;                                 // we need the image
    cv::Mat frame = cv_bridge::toCvCopy(last_, "bgr8")->image;

    for (auto &d : dets->detections)
    {
      const auto &b = d.bbox;
      /* YOLO bbox is centre-x, centre-y, width, height (in px) */
      int x = static_cast<int>(b.center.position.x - b.size_x / 2.0);
      int y = static_cast<int>(b.center.position.y - b.size_y / 2.0);
      int w = static_cast<int>(b.size_x);
      int h = static_cast<int>(b.size_y);

      /* clip to image bounds */
      x = std::max(0, x);
      y = std::max(0, y);
      w = std::min(w, frame.cols - x);
      h = std::min(h, frame.rows - y);
      if (w <= 0 || h <= 0) continue;

      cv::Mat crop, resized;
      cv::resize(frame(cv::Rect(x, y, w, h)), resized, {128, 128});

      auto msg = cv_bridge::CvImage(d.header, "bgr8", resized).toImageMsg();
      pub_crop_->publish(*msg);
    }
  }

  sensor_msgs::msg::Image::SharedPtr                         last_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr   sub_img_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_det_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      pub_crop_;
};
// --------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropNode>());
  rclcpp::shutdown();
  return 0;
}
