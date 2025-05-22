// -------- src/BottleDetector.cpp  (accept 8400×5 OR 5×8400) ------------
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
using std::placeholders::_1;

class Detector : public rclcpp::Node {
public:
  Detector() :
    Node("bottle_detector_ort"),
    env_(ORT_LOGGING_LEVEL_WARNING,"lid"),
    mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault))
  {
    auto pkg = ament_index_cpp::get_package_share_directory("visual");
    std::string mod = pkg + "/model/best_flat.onnx";
    Ort::SessionOptions opt; opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    sess_ = std::make_unique<Ort::Session>(env_,mod.c_str(),opt);

    in_  = sess_->GetInputNameAllocated (0,alloc_).get();
    out_ = sess_->GetOutputNameAllocated(0,alloc_).get();
    RCLCPP_INFO(get_logger(),"ORT model loaded   in=\"%s\"   out=\"%s\"",in_.c_str(),out_.c_str());

    sub_ = create_subscription<sensor_msgs::msg::Image>("webcam/image_raw",1,
            std::bind(&Detector::cb,this,_1));
    // pub_ = create_publisher<sensor_msgs::msg::Image>("yolo_detections",10);
    pub_img_ = create_publisher<sensor_msgs::msg::Image>("yolo_detections_image", 10);
    pub_det_ = create_publisher<vision_msgs::msg::Detection2DArray>("yolo_detections", 10);

  }

private:
  /* ═════════════════ callback ════════════════════════════════════════ */
  void cb(const sensor_msgs::msg::Image::SharedPtr m)
  {
    constexpr int   S   = 640;
    constexpr float THR = 0.40f;

    cv::Mat img = cv_bridge::toCvCopy(m,"bgr8")->image;
    float s = float(S)/std::max(img.rows,img.cols);

    cv::Mat rs; cv::resize(img,rs,{int(img.cols*s),int(img.rows*s)});
    cv::Mat pad(S,S,CV_8UC3,cv::Scalar(114,114,114));
    rs.copyTo(pad(cv::Rect(0,0,rs.cols,rs.rows)));
    cv::cvtColor(pad,pad,cv::COLOR_BGR2RGB);

    /* ---- tensor ---------------------------------------------------- */
    std::vector<float> input(3*S*S);
    for(int c=0;c<3;++c)
      for(int y=0;y<S;++y)
        for(int x=0;x<S;++x)
          input[c*S*S + y*S + x] = pad.at<cv::Vec3b>(y,x)[c] / 255.f;

    std::array<int64_t,4> dims{1,3,S,S};
    Ort::Value tin = Ort::Value::CreateTensor<float>(mem_,input.data(),input.size(),dims.data(),4);

    /* ---- run ------------------------------------------------------- */
    const char* names_in []={in_.c_str()};
    const char* names_out[]={out_.c_str()};
    auto out = sess_->Run(Ort::RunOptions{nullptr},names_in,&tin,1,names_out,1);

    auto info  = out[0].GetTensorTypeAndShapeInfo();
    auto shape = info.GetShape();                // expect [1,8400,5]  OR  [1,5,8400]
    if(shape.size()!=3){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Output rank %zu !=3",shape.size());
      return;
    }
    const int N = (shape[1]==5 ? shape[2]           // 1×5×8400  -> N=8400
                               : shape[1]);         // 1×8400×5

    float* d = out[0].GetTensorMutableData<float>();
    auto at = [&](int i,int k)->float{
      return (shape[1]==5) ? d[k*shape[2]+i]   // CHW
                           : d[i*5+k];         // HWC
    };

    std::vector<cv::Rect> boxes; std::vector<float> scores;
    for(int i=0;i<N;++i){
      float sc = at(i,4);
      if(sc < THR) continue;
      float cx = at(i,0), cy = at(i,1),
            w  = at(i,2), h  = at(i,3);
      int x=int((cx-w/2)/s), y=int((cy-h/2)/s);
      boxes.emplace_back(x,y,int(w/s),int(h/s));
      scores.push_back(sc);
    }
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes,scores,THR,0.45f,keep);

    // for(int k:keep) cv::rectangle(img,boxes[k],{0,255,0},2);
    // pub_->publish(*cv_bridge::CvImage(m->header,"bgr8",img).toImageMsg());
    for(int k:keep) cv::rectangle(img, boxes[k], {0,255,0}, 2);
    pub_img_->publish(*cv_bridge::CvImage(m->header,"bgr8", img).toImageMsg());

    // 2) republish detections as Detection2DArray
    vision_msgs::msg::Detection2DArray darr;
    darr.header = m->header;
    darr.detections.reserve(keep.size());
    for (int idx : keep) {
      auto &b = boxes[idx];
      vision_msgs::msg::Detection2D det;
      // fill the bounding‐box
      det.bbox.center.position.x = b.x + b.width*0.5;
      det.bbox.center.position.y = b.y + b.height*0.5;
      det.bbox.size_x = b.width;
      det.bbox.size_y = b.height;
      darr.detections.push_back(std::move(det));
    }
    pub_det_->publish(darr);

  }

  /* members */
  Ort::Env env_; Ort::AllocatorWithDefaultOptions alloc_;
  std::unique_ptr<Ort::Session> sess_; Ort::MemoryInfo mem_;
  std::string in_, out_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_img_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_det_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Detector>());
  rclcpp::shutdown(); return 0;
}
// src/BottleDetector.cpp
