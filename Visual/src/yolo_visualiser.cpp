#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <algorithm>
#include <functional>

using namespace cv;
using std::vector;

class YoloNode : public rclcpp::Node {
public:
  YoloNode(): Node("yolo_visualizer") {
    // parameters
    std::string model = declare_parameter("weights","models/yolo_bottle.onnx");
    conf_ = declare_parameter("conf",0.35f);

    // load network
    net_ = cv::dnn::readNet(model);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // pub/sub + GUI
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/webcam/image_raw", 10,
      std::bind(&YoloNode::on_image, this, std::placeholders::_1)
    );
    pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      "bottle_boxes", 10
    );
    namedWindow("YOLO Detections", WINDOW_AUTOSIZE);
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 1) ROS→Mat
    Mat img = cv_bridge::toCvCopy(msg,"bgr8")->image;

    // 2) blobFromImage
    Mat blob = cv::dnn::blobFromImage(img, 1/255.0, Size(640,640), {}, true, false);
    net_.setInput(blob);

    // 3) forward
    vector<Mat> outs;
    net_.forward(outs, net_.getUnconnectedOutLayersNames());

    vision_msgs::msg::Detection2DArray arr;
    arr.header = msg->header;

    // 4) parse each output tensor
    for (auto &o : outs) {
      // shape is [1, A*(5+NC), H, W]
      int C = o.size[1];
      int H = o.size[2];
      int W = o.size[3];
      int A = 3;                            // anchors per grid cell (YOLOv5 default)
      int per = C / A;                      // 5 + NC
      int NC  = per - 5;                    // number of classes
      int stride = H*W;                     // number of pixels per channel

      float *data = (float*)o.data;
      // for each anchor, each y, each x
      for (int a = 0; a < A; ++a) {
        int anchor_off = a * per * stride;
        for (int y = 0; y < H; ++y) {
          for (int x = 0; x < W; ++x) {
            int cell = y*W + x;
            int idx0 = anchor_off + cell;

            // objectness
            float obj_conf = data[idx0 + 4*stride];
            if (obj_conf < conf_) continue;

            // class scores
            float *cls_ptr = data + idx0 + 5*stride;
            int   cls      = std::distance(cls_ptr,
                                std::max_element(cls_ptr, cls_ptr+NC));
            float cls_conf = cls_ptr[cls];
            float final_conf = obj_conf * cls_conf;
            if (final_conf < conf_) continue;

            // box deltas
            float bx = data[idx0 + 0*stride];
            float by = data[idx0 + 1*stride];
            float bw = data[idx0 + 2*stride];
            float bh = data[idx0 + 3*stride];

            // convert to pixel coords
            float cx = bx * img.cols;
            float cy = by * img.rows;
            float w  = bw * img.cols;
            float h  = bh * img.rows;
            int   rx = int(cx - w/2);
            int   ry = int(cy - h/2);

            // draw
            rectangle(img, Rect(rx,ry,int(w),int(h)), Scalar(0,255,0), 2);
            char buf[64];
            snprintf(buf,64,"%s:%.0f%%",
                     (cls==1?"bottle":"esky"),
                     final_conf*100);
            putText(img, buf, Point(rx,ry-6),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,0), 1);

            // fill ROS message
            vision_msgs::msg::Detection2D det;
            det.bbox.center.position.x = cx;
            det.bbox.center.position.y = cy;
            det.bbox.size_x = w;
            det.bbox.size_y = h;
            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = cls;
            hyp.hypothesis.score    = final_conf;
            det.results.push_back(hyp);
            arr.detections.push_back(det);
          }
        }
      }
    }

    // 5) publish & show
    pub_->publish(arr);
    imshow("YOLO Detections", img);
    waitKey(1);
  }

  cv::dnn::Net net_;
  float conf_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloNode>());
  rclcpp::shutdown();
  destroyAllWindows();
  return 0;
}
