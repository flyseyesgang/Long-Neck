// // // -------- src/BottleDetector.cpp  (accept 8400×5 OR 5×8400) ------------
// // #include <rclcpp/rclcpp.hpp>
// // #include <sensor_msgs/msg/image.hpp>
// // #include <cv_bridge/cv_bridge.h>
// // #include <opencv2/opencv.hpp>
// // #include <onnxruntime_cxx_api.h>
// // #include <vision_msgs/msg/detection2_d_array.hpp>
// // #include <ament_index_cpp/get_package_share_directory.hpp>
// // using std::placeholders::_1;

// // class Detector : public rclcpp::Node {
// // public:
// //   Detector() :
// //     Node("bottle_detector_ort"),
// //     env_(ORT_LOGGING_LEVEL_WARNING,"lid"),
// //     mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault))
// //   {
// //     auto pkg = ament_index_cpp::get_package_share_directory("visual");
// //     std::string mod = pkg + "/model/best_flat.onnx";
// //     Ort::SessionOptions opt; opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
// //     sess_ = std::make_unique<Ort::Session>(env_,mod.c_str(),opt);

// //     in_  = sess_->GetInputNameAllocated (0,alloc_).get();
// //     out_ = sess_->GetOutputNameAllocated(0,alloc_).get();
// //     RCLCPP_INFO(get_logger(),"ORT model loaded   in=\"%s\"   out=\"%s\"",in_.c_str(),out_.c_str());

// //     sub_ = create_subscription<sensor_msgs::msg::Image>("webcam/image_raw",1,
// //             std::bind(&Detector::cb,this,_1));
// //     // pub_ = create_publisher<sensor_msgs::msg::Image>("yolo_detections",10);
// //     pub_img_ = create_publisher<sensor_msgs::msg::Image>("yolo_detections_image", 10);
// //     pub_det_ = create_publisher<vision_msgs::msg::Detection2DArray>("yolo_detections", 10);

// //   }

// // private:
// //   /* ═════════════════ callback ════════════════════════════════════════ */
// //   void cb(const sensor_msgs::msg::Image::SharedPtr m)
// //   {
// //     constexpr int   S   = 640;
// //     constexpr float THR = 0.40f;

// //     cv::Mat img = cv_bridge::toCvCopy(m,"bgr8")->image;
// //     float s = float(S)/std::max(img.rows,img.cols);

// //     cv::Mat rs; cv::resize(img,rs,{int(img.cols*s),int(img.rows*s)});
// //     cv::Mat pad(S,S,CV_8UC3,cv::Scalar(114,114,114));
// //     rs.copyTo(pad(cv::Rect(0,0,rs.cols,rs.rows)));
// //     cv::cvtColor(pad,pad,cv::COLOR_BGR2RGB);

// //     /* ---- tensor ---------------------------------------------------- */
// //     std::vector<float> input(3*S*S);
// //     for(int c=0;c<3;++c)
// //       for(int y=0;y<S;++y)
// //         for(int x=0;x<S;++x)
// //           input[c*S*S + y*S + x] = pad.at<cv::Vec3b>(y,x)[c] / 255.f;

// //     std::array<int64_t,4> dims{1,3,S,S};
// //     Ort::Value tin = Ort::Value::CreateTensor<float>(mem_,input.data(),input.size(),dims.data(),4);

// //     /* ---- run ------------------------------------------------------- */
// //     const char* names_in []={in_.c_str()};
// //     const char* names_out[]={out_.c_str()};
// //     auto out = sess_->Run(Ort::RunOptions{nullptr},names_in,&tin,1,names_out,1);

// //     auto info  = out[0].GetTensorTypeAndShapeInfo();
// //     auto shape = info.GetShape();                // expect [1,8400,5]  OR  [1,5,8400]
// //     if(shape.size()!=3){
// //       RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Output rank %zu !=3",shape.size());
// //       return;
// //     }
// //     const int N = (shape[1]==5 ? shape[2]           // 1×5×8400  -> N=8400
// //                                : shape[1]);         // 1×8400×5

// //     float* d = out[0].GetTensorMutableData<float>();
// //     auto at = [&](int i,int k)->float{
// //       return (shape[1]==5) ? d[k*shape[2]+i]   // CHW
// //                            : d[i*5+k];         // HWC
// //     };

// //     std::vector<cv::Rect> boxes; std::vector<float> scores;
// //     for(int i=0;i<N;++i){
// //       float sc = at(i,4);
// //       if(sc < THR) continue;
// //       float cx = at(i,0), cy = at(i,1),
// //             w  = at(i,2), h  = at(i,3);
// //       int x=int((cx-w/2)/s), y=int((cy-h/2)/s);
// //       boxes.emplace_back(x,y,int(w/s),int(h/s));
// //       scores.push_back(sc);
// //     }
// //     std::vector<int> keep;
// //     cv::dnn::NMSBoxes(boxes,scores,THR,0.45f,keep);

// //     // for(int k:keep) cv::rectangle(img,boxes[k],{0,255,0},2);
// //     // pub_->publish(*cv_bridge::CvImage(m->header,"bgr8",img).toImageMsg());
// //     for(int k:keep) cv::rectangle(img, boxes[k], {0,255,0}, 2);
// //     pub_img_->publish(*cv_bridge::CvImage(m->header,"bgr8", img).toImageMsg());

// //     // 2) republish detections as Detection2DArray
// //     vision_msgs::msg::Detection2DArray darr;
// //     darr.header = m->header;
// //     darr.detections.reserve(keep.size());
// //     for (int idx : keep) {
// //       auto &b = boxes[idx];
// //       vision_msgs::msg::Detection2D det;
// //       // fill the bounding‐box
// //       det.bbox.center.position.x = b.x + b.width*0.5;
// //       det.bbox.center.position.y = b.y + b.height*0.5;
// //       det.bbox.size_x = b.width;
// //       det.bbox.size_y = b.height;
// //       darr.detections.push_back(std::move(det));
// //     }
// //     pub_det_->publish(darr);

// //   }

// //   /* members */
// //   Ort::Env env_; Ort::AllocatorWithDefaultOptions alloc_;
// //   std::unique_ptr<Ort::Session> sess_; Ort::MemoryInfo mem_;
// //   std::string in_, out_;
// //   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
// //   // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_;
// //   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_img_;
// //   rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_det_;
// // };

// // int main(int argc,char** argv){
// //   rclcpp::init(argc,argv);
// //   rclcpp::spin(std::make_shared<Detector>());
// //   rclcpp::shutdown(); return 0;
// // }
// // // src/BottleDetector.cpp

// // src/BottleDetector.cpp
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <onnxruntime_cxx_api.h>
// #include <vision_msgs/msg/detection2_d_array.hpp>
// #include <std_msgs/msg/int32.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <ament_index_cpp/get_package_share_directory.hpp>
// using std::placeholders::_1;

// class Detector : public rclcpp::Node {
// public:
//   Detector() :
//     Node("bottle_detector_ort"),
//     env_(ORT_LOGGING_LEVEL_WARNING,"lid"),
//     mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault))
//   {
//     auto pkg = ament_index_cpp::get_package_share_directory("visual");
//     std::string mod = pkg + "/model/best_flat.onnx";
//     Ort::SessionOptions opt; opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
//     sess_ = std::make_unique<Ort::Session>(env_,mod.c_str(),opt);

//     in_  = sess_->GetInputNameAllocated (0,alloc_).get();
//     out_ = sess_->GetOutputNameAllocated(0,alloc_).get();
//     RCLCPP_INFO(get_logger(),"ORT model loaded   in=\"%s\"   out=\"%s\"",in_.c_str(),out_.c_str());

//     sub_ = create_subscription<sensor_msgs::msg::Image>("webcam/image_raw",1,
//             std::bind(&Detector::cb,this,_1));
//     pub_img_ = create_publisher<sensor_msgs::msg::Image>("yolo_detections_image", 10);
//     pub_det_ = create_publisher<vision_msgs::msg::Detection2DArray>("yolo_detections", 10);
//     pub_count_ = create_publisher<std_msgs::msg::Int32>("bottle_lid_count", 10);
//     pub_pose_array_ = create_publisher<geometry_msgs::msg::PoseArray>("bottle_lid_cartesian", 10);

//     declare_parameter<double>("lid_diameter_mm", 28.0);
//     declare_parameter<std::vector<double>>("camera_intrinsics",
//       std::vector<double>{600.0, 0.0, 320.0,
//                            0.0, 600.0, 240.0,
//                            0.0,   0.0,   1.0});
//     lid_diam_mm_ = get_parameter("lid_diameter_mm").as_double();
//     auto cam = get_parameter("camera_intrinsics").as_double_array();
//     K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(cam.data())).clone();
//   }

// private:
//   void cb(const sensor_msgs::msg::Image::SharedPtr m)
//   {
//     constexpr int   S   = 640;
//     constexpr float THR = 0.40f;

//     cv::Mat img = cv_bridge::toCvCopy(m,"bgr8")->image;
//     float s = float(S)/std::max(img.rows,img.cols);

//     cv::Mat rs; cv::resize(img,rs,{int(img.cols*s),int(img.rows*s)});
//     cv::Mat pad(S,S,CV_8UC3,cv::Scalar(114,114,114));
//     rs.copyTo(pad(cv::Rect(0,0,rs.cols,rs.rows)));
//     cv::cvtColor(pad,pad,cv::COLOR_BGR2RGB);

//     std::vector<float> input(3*S*S);
//     for(int c=0;c<3;++c)
//       for(int y=0;y<S;++y)
//         for(int x=0;x<S;++x)
//           input[c*S*S + y*S + x] = pad.at<cv::Vec3b>(y,x)[c] / 255.f;

//     std::array<int64_t,4> dims{1,3,S,S};
//     Ort::Value tin = Ort::Value::CreateTensor<float>(mem_,input.data(),input.size(),dims.data(),4);

//     const char* names_in []={in_.c_str()};
//     const char* names_out[]={out_.c_str()};
//     auto out = sess_->Run(Ort::RunOptions{nullptr},names_in,&tin,1,names_out,1);

//     auto info  = out[0].GetTensorTypeAndShapeInfo();
//     auto shape = info.GetShape();
//     if(shape.size()!=3){
//       RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Output rank %zu !=3",shape.size());
//       return;
//     }
//     const int N = (shape[1]==5 ? shape[2] : shape[1]);
//     float* d = out[0].GetTensorMutableData<float>();
//     auto at = [&](int i,int k)->float{
//       return (shape[1]==5) ? d[k*shape[2]+i] : d[i*5+k];
//     };

//     std::vector<cv::Rect> boxes; std::vector<float> scores;
//     for(int i=0;i<N;++i){
//       float sc = at(i,4);
//       if(sc < THR) continue;
//       float cx = at(i,0), cy = at(i,1),
//             w  = at(i,2), h  = at(i,3);
//       int x=int((cx-w/2)/s), y=int((cy-h/2)/s);
//       boxes.emplace_back(x,y,int(w/s),int(h/s));
//       scores.push_back(sc);
//     }
//     std::vector<int> keep;
//     cv::dnn::NMSBoxes(boxes,scores,THR,0.45f,keep);

//     for(int k:keep) cv::rectangle(img, boxes[k], {0,255,0}, 2);
//     pub_img_->publish(*cv_bridge::CvImage(m->header,"bgr8", img).toImageMsg());

//     vision_msgs::msg::Detection2DArray darr;
//     darr.header = m->header;
//     darr.detections.reserve(keep.size());

//     geometry_msgs::msg::PoseArray pa;
//     pa.header = m->header;

//     double fx = K_.at<double>(0, 0);
//     double fy = K_.at<double>(1, 1);
//     double cx = K_.at<double>(0, 2);
//     double cy = K_.at<double>(1, 2);
//     double real_diam_m = lid_diam_mm_ * 1e-3;

//     for (int idx : keep) {
//       auto &b = boxes[idx];
//       vision_msgs::msg::Detection2D det;
//       det.bbox.center.position.x = b.x + b.width*0.5;
//       det.bbox.center.position.y = b.y + b.height*0.5;
//       det.bbox.size_x = b.width;
//       det.bbox.size_y = b.height;
//       darr.detections.push_back(det);

//       double u = det.bbox.center.position.x;
//       double v = det.bbox.center.position.y;
//       double pix_diam = (det.bbox.size_x + det.bbox.size_y) * 0.5;
//       double Z = fx * real_diam_m / pix_diam;
//       double X = (u - cx) * Z / fx;
//       double Y = (v - cy) * Z / fy;

//       geometry_msgs::msg::Pose pose;
//       pose.position.x = X;
//       pose.position.y = Y;
//       pose.position.z = Z;
//       pose.orientation.w = 1.0;
//       pa.poses.push_back(pose);
//     }
//     pub_det_->publish(darr);
//     pub_pose_array_->publish(pa);

//     std_msgs::msg::Int32 count_msg;
//     count_msg.data = darr.detections.size();
//     pub_count_->publish(count_msg);
//   }

//   Ort::Env env_; Ort::AllocatorWithDefaultOptions alloc_;
//   std::unique_ptr<Ort::Session> sess_; Ort::MemoryInfo mem_;
//   std::string in_, out_;
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
//   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_img_;
//   rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_det_;
//   rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_count_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_array_;
//   cv::Mat K_;
//   double lid_diam_mm_;
// };

// int main(int argc,char** argv){
//   rclcpp::init(argc,argv);
//   rclcpp::spin(std::make_shared<Detector>());
//   rclcpp::shutdown(); return 0;
// }

// File: src/BottleDetector.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <onnxruntime_cxx_api.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using std::placeholders::_1;

class BottleDetector : public rclcpp::Node {
public:
  BottleDetector()
  : Node("bottle_detector"),
    env_(ORT_LOGGING_LEVEL_WARNING, "lid_detector"),
    mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
  {
    // Load the YOLOv8 model for bottle lids
    std::string pkg_path = ament_index_cpp::get_package_share_directory("visual");
    std::string model_path = pkg_path + "/model/best_flat.onnx";
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);

    // Get model input/output tensor names
    in_name_  = session_->GetInputNameAllocated(0, alloc_).get();
    out_name_ = session_->GetOutputNameAllocated(0, alloc_).get();
    RCLCPP_INFO(get_logger(), "Loaded lid detection model (input='%s', output='%s')",
                in_name_.c_str(), out_name_.c_str());

    // Create subscriber for images and publishers for outputs
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/webcam/image_raw", 10,
      std::bind(&BottleDetector::imageCallback, this, _1));
    image_pub_  = create_publisher<sensor_msgs::msg::Image>("/yolo_detections_image", 10);
    detect_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("/yolo_detections", 10);
    count_pub_  = create_publisher<std_msgs::msg::Int32>("/lid_count", 10);
    matrix_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/bottle_matrix", 10);

    // Declare and get camera parameters
    declare_parameter<double>("lid_diameter_mm", 28.0);
    declare_parameter<std::vector<double>>("camera_intrinsics",
        std::vector<double>{600.0, 0.0, 320.0,
                             0.0, 600.0, 240.0,
                             0.0,   0.0,   1.0});
    lid_diameter_mm_ = get_parameter("lid_diameter_mm").as_double();
    auto intrin = get_parameter("camera_intrinsics").as_double_array();
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(intrin.data())).clone();
    RCLCPP_INFO(get_logger(), "Lid diameter: %.1f mm, Camera fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                lid_diameter_mm_, camera_matrix_.at<double>(0,0),
                camera_matrix_.at<double>(1,1), camera_matrix_.at<double>(0,2),
                camera_matrix_.at<double>(1,2));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (done_) {
      return; // do nothing after all lids found once
    }

    // Prepare image for inference (letterbox to 640x640)
    constexpr int   S   = 640;
    constexpr float SCORE_THRESH = 0.60f;
    constexpr float NMS_THRESH   = 0.45f;
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    float scale = float(S) / std::max(frame.rows, frame.cols);
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(int(frame.cols * scale), int(frame.rows * scale)));
    cv::Mat input_img(S, S, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(input_img(cv::Rect(0, 0, resized.cols, resized.rows)));
    cv::cvtColor(input_img, input_img, cv::COLOR_BGR2RGB);

    // Create input tensor
    std::vector<float> input_data(3 * S * S);
    for (int c = 0; c < 3; ++c) {
      for (int y = 0; y < S; ++y) {
        for (int x = 0; x < S; ++x) {
          input_data[c * S * S + y * S + x] = input_img.at<cv::Vec3b>(y, x)[c] / 255.0f;
        }
      }
    }
    std::array<int64_t, 4> input_shape{1, 3, S, S};
    Ort::Value ort_input = Ort::Value::CreateTensor<float>(
        mem_, input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

    // Run ONNX model inference
    const char* input_names[] = { in_name_.c_str() };
    const char* output_names[] = { out_name_.c_str() };
    auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_names, &ort_input, 1, output_names, 1);

    // Parse model output
    float* data = outputs[0].GetTensorMutableData<float>();
    auto out_info = outputs[0].GetTensorTypeAndShapeInfo();
    auto out_shape = out_info.GetShape();
    if (out_shape.size() != 3) {
      RCLCPP_ERROR(get_logger(), "Unexpected output shape");
      return;
    }
    int count = (out_shape[1] == 5 ? out_shape[2] : out_shape[1]);
    auto get_val = [&](int i, int k)->float {
      return (out_shape[1] == 5) ? data[k * out_shape[2] + i]
                                 : data[i * 5 + k];
    };

    // Collect boxes and scores above threshold
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    boxes.reserve(count);
    scores.reserve(count);
    for (int i = 0; i < count; ++i) {
      float score = get_val(i, 4);
      if (score < SCORE_THRESH) continue;
      float cx = get_val(i, 0);
      float cy = get_val(i, 1);
      float w  = get_val(i, 2);
      float h  = get_val(i, 3);
      int x = static_cast<int>((cx - w/2) / scale);
      int y = static_cast<int>((cy - h/2) / scale);
      int w_orig = static_cast<int>(w / scale);
      int h_orig = static_cast<int>(h / scale);
      boxes.emplace_back(x, y, w_orig, h_orig);
      scores.push_back(score);
    }

    // Apply Non-Max Suppression
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, SCORE_THRESH, NMS_THRESH, keep);

    // Draw detections on the original frame
    for (int idx : keep) {
      cv::rectangle(frame, boxes[idx], cv::Scalar(0, 255, 0), 2);
    }
    // Show debug window with bounding boxes
    cv::imshow("Bottle Lids Detection", frame);
    cv::waitKey(1);

    // Publish image with bounding boxes
    auto out_img_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    image_pub_->publish(*out_img_msg);

    // Publish detection array
    vision_msgs::msg::Detection2DArray det_array;
    det_array.header = msg->header;
    det_array.detections.reserve(keep.size());
    for (int idx : keep) {
      const cv::Rect& b = boxes[idx];
      vision_msgs::msg::Detection2D det;
      det.bbox.center.position.x = b.x + b.width * 0.5;
      det.bbox.center.position.y = b.y + b.height * 0.5;
      det.bbox.size_x = b.width;
      det.bbox.size_y = b.height;
      det_array.detections.push_back(det);
    }
    detect_pub_->publish(det_array);

    // Publish lid count
    std_msgs::msg::Int32 count_msg;
    count_msg.data = det_array.detections.size();
    count_pub_->publish(count_msg);

    // Check if all 12 lids are detected
    size_t num_lids = det_array.detections.size();
    if (num_lids != 12) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Detected %zu lids (waiting for 12)...", num_lids);
      return;
    }

    // Compute 3D coordinates for each lid (in camera frame)
    double fx = camera_matrix_.at<double>(0, 0);
    double fy = camera_matrix_.at<double>(1, 1);
    double cx = camera_matrix_.at<double>(0, 2);
    double cy = camera_matrix_.at<double>(1, 2);
    double real_diameter_m = lid_diameter_mm_ * 1e-3;
    struct Lid { double u, v, X, Y, Z; };
    std::vector<Lid> lids;
    lids.reserve(num_lids);
    for (auto& det : det_array.detections) {
      double u = det.bbox.center.position.x;
      double v = det.bbox.center.position.y;
      double pixel_diameter = (det.bbox.size_x + det.bbox.size_y) * 0.5;
      double Z = fx * real_diameter_m / pixel_diameter;
      double X = (u - cx) * Z / fx;
      double Y = (v - cy) * Z / fy;
      lids.push_back({u, v, X, Y, Z});
    }
    // Sort lids by image position for consistent ordering (top-left to bottom-right)
    std::sort(lids.begin(), lids.end(), [](const Lid& a, const Lid& b) {
      if (fabs(a.v - b.v) > 1e-6) return a.v < b.v;
      return a.u < b.u;
    });

    // Log the positions of all lids
    for (size_t i = 0; i < lids.size(); ++i) {
      RCLCPP_INFO(get_logger(), "Lid %zu -> [%.3f, %.3f, %.3f] (XYZ in m)",
                  i, lids[i].X, lids[i].Y, lids[i].Z);
    }

    // Create and publish 12x4 matrix (id, x, y, z for each lid)
    std_msgs::msg::Float64MultiArray matrix_msg;
    matrix_msg.layout.dim.resize(2);
    matrix_msg.layout.dim[0].label = "lid";
    matrix_msg.layout.dim[0].size = lids.size();
    matrix_msg.layout.dim[0].stride = lids.size() * 4;
    matrix_msg.layout.dim[1].label = "coords";
    matrix_msg.layout.dim[1].size = 4;
    matrix_msg.layout.dim[1].stride = 4;
    matrix_msg.data.resize(lids.size() * 4);
    for (size_t i = 0; i < lids.size(); ++i) {
      matrix_msg.data[i * 4 + 0] = static_cast<double>(i);
      matrix_msg.data[i * 4 + 1] = lids[i].X;
      matrix_msg.data[i * 4 + 2] = lids[i].Y;
      matrix_msg.data[i * 4 + 3] = lids[i].Z;
    }
    matrix_pub_->publish(matrix_msg);

    done_ = true;
    RCLCPP_INFO(get_logger(), "All 12 lids found. /bottle_matrix published.");
  }

  // Member variables
  Ort::Env env_;
  Ort::AllocatorWithDefaultOptions alloc_;
  Ort::MemoryInfo mem_;
  std::unique_ptr<Ort::Session> session_;
  std::string in_name_, out_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detect_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr matrix_pub_;
  cv::Mat camera_matrix_;
  double lid_diameter_mm_;
  bool done_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BottleDetector>());
  rclcpp::shutdown();
  return 0;
}
