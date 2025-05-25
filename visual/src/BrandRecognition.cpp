// // #include <rclcpp/rclcpp.hpp>
// // #include <sensor_msgs/msg/image.hpp>
// // #include <cv_bridge/cv_bridge.h>
// // #include <opencv2/opencv.hpp>
// // #include <onnxruntime_cxx_api.h>
// // #include <ament_index_cpp/get_package_share_directory.hpp>
// // #include <fstream>
// // #include <regex>

// // using std::placeholders::_1;

// // class BrandRecognition : public rclcpp::Node {
// // public:
// //   BrandRecognition()
// //     : Node("brand_recognition"),
// //       env_(ORT_LOGGING_LEVEL_WARNING, "brand")
// //   {
// //     // Initialize
// //     detected_ = false;
// //     corona_index_ = -1;

// //     // Parameters
// //     model_name_  = declare_parameter("model_name", std::string("brand_cls_224.onnx"));
// //     image_topic_ = declare_parameter("image_topic", std::string("/webcam/image_raw"));
// //     input_size_  = declare_parameter("input_size", 224);
// //     input_width_ = input_height_ = input_size_;
// //     RCLCPP_INFO(get_logger(), "Resizing to %dx%d", input_width_, input_height_);

// //     // Load class names from YAML
// //     auto pkg = ament_index_cpp::get_package_share_directory("visual");
// //     std::string names_path = pkg + "/model/brand_names.yaml";
// //     std::ifstream ifs(names_path);
// //     if (!ifs) {
// //       RCLCPP_ERROR(get_logger(), "Failed to open %s", names_path.c_str());
// //       rclcpp::shutdown();
// //       return;
// //     }
// //     std::regex rx(R"(^\s*(\d+)\s*:\s*(\S+))");
// //     std::string line;
// //     while (std::getline(ifs, line)) {
// //       std::smatch m;
// //       if (std::regex_search(line, m, rx) && m.size() == 3) {
// //         int idx = std::stoi(m[1]);
// //         std::string name = m[2];
// //         if ((int)class_names_.size() <= idx) class_names_.resize(idx+1);
// //         class_names_[idx] = name;
// //       }
// //     }
// //     for (size_t i = 0; i < class_names_.size(); ++i) {
// //       if (class_names_[i] == "Corona") {
// //         corona_index_ = (int)i;
// //         break;
// //       }
// //     }
// //     if (corona_index_ < 0) {
// //       RCLCPP_ERROR(get_logger(), "'Corona' not found in names list");
// //       rclcpp::shutdown();
// //       return;
// //     }

// //     // Load ONNX model
// //     std::string model_path = pkg + "/model/" + model_name_;
// //     Ort::SessionOptions opts;
// //     opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
// //     session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);

// //     // Get I/O names
// //     Ort::AllocatorWithDefaultOptions allocator;
// //     auto in_ptr = session_->GetInputNameAllocated(0, allocator);
// //     input_name_ = in_ptr.get();
// //     auto out_ptr = session_->GetOutputNameAllocated(0, allocator);
// //     output_name_ = out_ptr.get();

// //     // Subscribe
// //     image_sub_ = create_subscription<sensor_msgs::msg::Image>(
// //       image_topic_, 10,
// //       std::bind(&BrandRecognition::imageCallback, this, _1)
// //     );
// //     RCLCPP_INFO(get_logger(), "Subscribed to '%s'", image_topic_.c_str());
// //   }

// // private:
// //   void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
// //     if (detected_) return;
// //     cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

// //     // Preprocess
// //     cv::Mat resized;
// //     cv::resize(img, resized, cv::Size(input_width_, input_height_));
// //     cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
// //     resized.convertTo(resized, CV_32F, 1.0f/255.0f);

// //     // HWC -> CHW
// //     std::vector<float> tensor(3 * input_height_ * input_width_);
// //     for (int c = 0; c < 3; ++c)
// //       for (int y = 0; y < input_height_; ++y)
// //         for (int x = 0; x < input_width_; ++x)
// //           tensor[c*input_height_*input_width_ + y*input_width_ + x] = resized.at<cv::Vec3f>(y, x)[c];

// //     // Create input tensor
// //     std::array<int64_t,4> dims{1,3,input_height_,input_width_};
// //     auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
// //     auto input_tensor = Ort::Value::CreateTensor<float>(mem_info, tensor.data(), tensor.size(), dims.data(), dims.size());

// //     // Inference
// //     const char* in_names[]  = { input_name_.c_str() };  // use c_str() on std::string
// //     const char* out_names[] = { output_name_.c_str() };
// //     auto outputs = session_->Run(Ort::RunOptions(), in_names, &input_tensor, 1, out_names, 1);
// //     RCLCPP_INFO(get_logger(),
// //    "Running onnx with input size %d×%d, model expects [%lld×%lld]",
// //    input_height_, input_width_,
// //    session_->GetInputTypeInfo(0)
// //        .GetTensorTypeAndShapeInfo()
// //        .GetShape()[2],
// //    session_->GetInputTypeInfo(0)
// //        .GetTensorTypeAndShapeInfo()
// //        .GetShape()[3]);

// //     // Postprocess
// //     float* scores = outputs[0].GetTensorMutableData<float>();
// //     auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
// //     int C = static_cast<int>(shape[1]);
// //     int best = std::max_element(scores, scores + C) - scores;

// //     if (best != corona_index_) {
// //       RCLCPP_INFO(get_logger(), "BRAND NOT DETECTED");
// //       return;
// //     }

// //     detected_ = true;
// //     RCLCPP_INFO(get_logger(), "BRAND = 'Corona' (score=%.2f)" , scores[best]);
// //   }

// //   // Members
// //   Ort::Env env_;
// //   std::unique_ptr<Ort::Session> session_;
// //   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
// //   std::vector<std::string> class_names_;
// //   int corona_index_;
// //   int input_size_, input_height_, input_width_;
// //   bool detected_;
// //   std::string input_name_, output_name_, model_name_, image_topic_;
// // };

// // int main(int argc, char** argv) {
// //   rclcpp::init(argc, argv);
// //   rclcpp::spin(std::make_shared<BrandRecognition>());
// //   rclcpp::shutdown();
// //   return 0;
// // }



// // src/BrandRecognition.cpp
// // // src/BrandRecognition.cpp
// // #include <rclcpp/rclcpp.hpp>
// // #include <sensor_msgs/msg/image.hpp>
// // #include <cv_bridge/cv_bridge.h>
// // #include <opencv2/opencv.hpp>
// // #include <onnxruntime_cxx_api.h>
// // #include <ament_index_cpp/get_package_share_directory.hpp>
// // #include <fstream>
// // #include <regex>

// // using std::placeholders::_1;

// // class BrandRecognition : public rclcpp::Node {
// // public:
// //   BrandRecognition()
// //     : Node("brand_recognition"),
// //       env_(ORT_LOGGING_LEVEL_WARNING, "brand_recognition")
// //   {
// //     // Parameters
// //     model_name_     = declare_parameter("model_name", std::string("brand_cls_224.onnx"));
// //     image_topic_    = declare_parameter("image_topic", std::string("/webcam/image_raw"));
// //     input_size_     = declare_parameter("input_size", 224);
// //     conf_threshold_ = declare_parameter("conf_threshold", 0.5f);

// //     // Derived dims
// //     input_w_ = input_h_ = input_size_;
// //     RCLCPP_INFO(get_logger(), "Input size: %dx%d, confidence threshold=%.2f",
// //                 input_w_, input_h_, conf_threshold_);

// //     // Load class names
// //     auto pkg_dir = ament_index_cpp::get_package_share_directory("visual");
// //     std::string yaml_path = pkg_dir + "/model/brand_names.yaml";
// //     std::ifstream ifs(yaml_path);
// //     if (!ifs.is_open()) {
// //       RCLCPP_ERROR(get_logger(), "Cannot open names file: %s", yaml_path.c_str());
// //       rclcpp::shutdown(); return;
// //     }
// //     std::string line;
// //     std::regex re(R"(^\s*(\d+)\s*:\s*(\S+))");
// //     while (std::getline(ifs, line)) {
// //       std::smatch m;
// //       if (std::regex_match(line, m, re) && m.size()==3) {
// //         int idx = std::stoi(m[1]);
// //         if ((int)names_.size() <= idx) names_.resize(idx+1);
// //         names_[idx] = m[2];
// //       }
// //     }
// //     RCLCPP_INFO(get_logger(), "Loaded %zu class names", names_.size());

// //     // Load ONNX model
// //     std::string model_path = pkg_dir + "/model/" + model_name_;
// //     Ort::SessionOptions opts;
// //     opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
// //     session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);

// //     // Get I/O names
// //     Ort::AllocatorWithDefaultOptions allocator;
// //     {
// //       auto in_alloc = session_->GetInputNameAllocated(0, allocator);
// //       input_name_ = in_alloc.get();
// //       allocator.Free(in_alloc.release());
// //     }
// //     {
// //       auto out_alloc = session_->GetOutputNameAllocated(0, allocator);
// //       output_name_ = out_alloc.get();
// //       allocator.Free(out_alloc.release());
// //     }
// //     RCLCPP_INFO(get_logger(), "Model I/O names: in='%s', out='%s'",
// //                 input_name_.c_str(), output_name_.c_str());

// //     // Subscribe to images
// //     image_sub_ = create_subscription<sensor_msgs::msg::Image>(
// //       image_topic_, 10,
// //       std::bind(&BrandRecognition::imageCallback, this, _1)
// //     );
// //     RCLCPP_INFO(get_logger(), "Subscribed to '%s'", image_topic_.c_str());
// //   }

// // private:
// //   void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
// //     // Convert ROS image to BGR CV
// //     cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;

// //     // Resize & convert to RGB float
// //     cv::Mat resized, rgb;
// //     cv::resize(bgr, resized, cv::Size(input_w_, input_h_));
// //     cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
// //     rgb.convertTo(rgb, CV_32F, 1.0f/255.0f);

// //     // HWC -> CHW tensor
// //     std::vector<float> tensor(3 * input_h_ * input_w_);
// //     for (int c = 0; c < 3; ++c) {
// //       for (int y = 0; y < input_h_; ++y) {
// //         for (int x = 0; x < input_w_; ++x) {
// //           tensor[c*input_h_*input_w_ + y*input_w_ + x] =
// //             rgb.at<cv::Vec3f>(y, x)[c];
// //         }
// //       }
// //     }

// //     // Build ONNX tensor
// //     std::array<int64_t,4> shape{1, 3, input_h_, input_w_};
// //     Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
// //     Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
// //       mem, tensor.data(), tensor.size(), shape.data(), shape.size());

// //     // Inference
// //     const char* in_n[]  = { input_name_.c_str() };
// //     const char* out_n[] = { output_name_.c_str() };
// //     auto outputs = session_->Run(
// //       Ort::RunOptions(), in_n, &input_tensor, 1, out_n, 1);

// //     // Postprocess
// //     float* scores = outputs[0].GetTensorMutableData<float>();
// //     auto out_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
// //     int C = (int)out_shape[1];
// //     int best = std::max_element(scores, scores+C) - scores;
// //     float conf = scores[best];
// //     std::string name = (best < (int)names_.size()) ? names_[best] : "<unknown>";

// //     if (conf >= conf_threshold_) {
// //       RCLCPP_INFO(get_logger(), "BRAND = '%s' (%.2f)", name.c_str(), conf);
// //     } else {
// //       RCLCPP_INFO(get_logger(), "NO BRAND (top=%s, %.2f)", name.c_str(), conf);
// //     }
// //   }

// //   // ONNX runtime
// //   Ort::Env env_;
// //   std::unique_ptr<Ort::Session> session_;

// //   // ROS subscription
// //   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

// //   // I/O names
// //   std::string input_name_, output_name_;

// //   // parameters
// //   std::string model_name_, image_topic_;
// //   int input_size_, input_w_, input_h_;
// //   float conf_threshold_;

// //   // class names
// //   std::vector<std::string> names_;
// // };

// // int main(int argc, char** argv) {
// //   rclcpp::init(argc, argv);
// //   rclcpp::spin(std::make_shared<BrandRecognition>());
// //   rclcpp::shutdown();
// //   return 0;
// // }

// // src/BrandRecognition.cpp
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <onnxruntime_cxx_api.h>
// #include <ament_index_cpp/get_package_share_directory.hpp>
// #include <fstream>
// #include <regex>

// using std::placeholders::_1;

// class BrandRecognition : public rclcpp::Node {
// public:
//   BrandRecognition()
//     : Node("brand_recognition"),
//       env_(ORT_LOGGING_LEVEL_WARNING, "brand_recognition")
//   {
//     // Initialize detection flag
//     detected_ = false;

//     // Declare parameters
//     model_name_     = declare_parameter("model_name", std::string("brand_cls_224.onnx"));
//     image_topic_    = declare_parameter("image_topic", std::string("/webcam/image_raw"));
//     input_size_     = declare_parameter("input_size", 224);
//     conf_threshold_ = declare_parameter("conf_threshold", 0.5f);

//     input_w_ = input_h_ = input_size_;
//     RCLCPP_INFO(get_logger(), "Input size: %dx%d, conf_threshold=%.2f", input_w_, input_h_, conf_threshold_);

//     // Load class names
//     auto pkg_dir = ament_index_cpp::get_package_share_directory("visual");
//     std::string names_path = pkg_dir + "/model/brand_names.yaml";
//     std::ifstream ifs(names_path);
//     if (!ifs.is_open()) {
//       RCLCPP_ERROR(get_logger(), "Failed to open names file: %s", names_path.c_str());
//       rclcpp::shutdown();
//       return;
//     }
//     std::string line;
//     std::regex re(R"(^\s*(\d+)\s*:\s*(\S+))");
//     while (std::getline(ifs, line)) {
//       std::smatch m;
//       if (std::regex_match(line, m, re) && m.size() == 3) {
//         int idx = std::stoi(m[1]);
//         if ((int)names_.size() <= idx) names_.resize(idx+1);
//         names_[idx] = m[2];
//       }
//     }
//     RCLCPP_INFO(get_logger(), "Loaded %zu class names", names_.size());

//     // Load ONNX model
//     std::string model_path = pkg_dir + "/model/" + model_name_;
//     Ort::SessionOptions opts;
//     opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
//     session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);

//     // Fetch I/O names
//     Ort::AllocatorWithDefaultOptions alloc;
//     auto in_ptr = session_->GetInputNameAllocated(0, alloc);
//     input_name_  = in_ptr.get();
//     alloc.Free(in_ptr.release());
//     auto out_ptr = session_->GetOutputNameAllocated(0, alloc);
//     output_name_ = out_ptr.get();
//     alloc.Free(out_ptr.release());
//     RCLCPP_INFO(get_logger(), "Model I/O: in='%s', out='%s'", input_name_.c_str(), output_name_.c_str());

//     // Subscribe to camera images
//     image_sub_ = create_subscription<sensor_msgs::msg::Image>(
//       image_topic_, 10,
//       std::bind(&BrandRecognition::imageCallback, this, _1)
//     );
//     RCLCPP_INFO(get_logger(), "Subscribed to '%s'", image_topic_.c_str());
//   }

// private:
//   void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//     // If already detected, print empty brand then exit
//     if (detected_) {
//       RCLCPP_INFO(get_logger(), "BRAND = ''");
//       rclcpp::shutdown();
//       return;
//     }

//     // Convert ROS image to OpenCV BGR
//     cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
//     // Resize and convert to RGB float
//     cv::Mat resized, rgb;
//     cv::resize(bgr, resized, cv::Size(input_w_, input_h_));
//     cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
//     rgb.convertTo(rgb, CV_32F, 1.0f/255.0f);

//     // Prepare input tensor CHW
//     std::vector<float> tensor(3 * input_h_ * input_w_);
//     for (int c = 0; c < 3; ++c) {
//       for (int y = 0; y < input_h_; ++y) {
//         for (int x = 0; x < input_w_; ++x) {
//           tensor[c*input_h_*input_w_ + y*input_w_ + x] = rgb.at<cv::Vec3f>(y, x)[c];
//         }
//       }
//     }
//     std::array<int64_t,4> dims{1,3,input_h_,input_w_};
//     Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
//     auto input_tensor = Ort::Value::CreateTensor<float>(mem, tensor.data(), tensor.size(), dims.data(), dims.size());

//     // Run inference
//     const char* in_names[]  = { input_name_.c_str() };
//     const char* out_names[] = { output_name_.c_str() };
//     auto outputs = session_->Run(Ort::RunOptions(), in_names, &input_tensor, 1, out_names, 1);

//     // Postprocess
//     float* scores = outputs[0].GetTensorMutableData<float>();
//     auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
//     int C = static_cast<int>(shape[1]);
//     int best = std::distance(scores, std::max_element(scores, scores + C));
//     float conf = scores[best];
//     std::string name = (best < (int)names_.size() ? names_[best] : std::string("<unknown>"));

//     if (conf >= conf_threshold_) {
//       RCLCPP_INFO(get_logger(), "BRAND = '%s' (%.2f)", name.c_str(), conf);
//       detected_ = true;
//     } else {
//       RCLCPP_INFO(get_logger(), "NO BRAND (top=%s, %.2f)", name.c_str(), conf);
//     }
//   }

//   // Members
//   Ort::Env env_;
//   std::unique_ptr<Ort::Session> session_;
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
//   std::string input_name_, output_name_;
//   std::string model_name_, image_topic_;
//   int input_size_, input_w_, input_h_;
//   float conf_threshold_;
//   std::vector<std::string> names_;
//   bool detected_;
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<BrandRecognition>());
//   rclcpp::shutdown();
//   return 0;
// }

// File: src/BrandRecognition.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <regex>
#include <deque>
#include <vector>
#include <string>

using std::placeholders::_1;

class BrandRecognition : public rclcpp::Node {
public:
  BrandRecognition()
  : Node("brand_recognition"),
    env_(ORT_LOGGING_LEVEL_WARNING, "brand_recognition"),
    mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
  {
    // Declare parameters
    model_name_     = declare_parameter<std::string>("model_name", "brand_cls_224.onnx");
    input_size_     = declare_parameter<int>("input_size", 224);
    conf_threshold_ = declare_parameter<double>("conf_threshold", 0.3);
    input_width_ = input_height_ = input_size_;
    RCLCPP_INFO(get_logger(), "Brand classifier input size: %dx%d, confidence threshold: %.2f",
                input_width_, input_height_, conf_threshold_);

    // Load class names from YAML
    auto pkg_share = ament_index_cpp::get_package_share_directory("visual");
    std::string names_path = pkg_share + "/model/brand_names.yaml";
    std::ifstream ifs(names_path);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Could not open class names file: %s", names_path.c_str());
    } else {
      std::regex line_pattern(R"(^\s*(\d+)\s*:\s*(\S+))");
      std::string line;
      while (std::getline(ifs, line)) {
        std::smatch m;
        if (std::regex_match(line, m, line_pattern) && m.size() == 3) {
          int class_id = std::stoi(m[1]);
          std::string class_name = m[2];
          if ((int)class_names_.size() <= class_id) {
            class_names_.resize(class_id + 1);
          }
          class_names_[class_id] = class_name;
        }
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu class names for brand recognition.", class_names_.size());
    }

    // Load the ONNX classifier model
    std::string model_path = pkg_share + "/model/" + model_name_;
    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options);
    // Get model input/output names
    Ort::AllocatorWithDefaultOptions allocator;
    auto in_name_alloc  = session_->GetInputNameAllocated(0, allocator);
    input_name_  = in_name_alloc.get();
    allocator.Free(in_name_alloc.release());
    auto out_name_alloc = session_->GetOutputNameAllocated(0, allocator);
    output_name_ = out_name_alloc.get();
    allocator.Free(out_name_alloc.release());
    RCLCPP_INFO(get_logger(), "Brand model loaded: %s (Input: '%s', Output: '%s')",
                model_name_.c_str(), input_name_.c_str(), output_name_.c_str());

    // Subscriptions and publishers
    image_sub_    = create_subscription<sensor_msgs::msg::Image>(
                     "/webcam/image_raw", 10,
                     std::bind(&BrandRecognition::imageCallback, this, _1));
    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
                     "/yolo_detections", 10,
                     std::bind(&BrandRecognition::detectionCallback, this, _1));
    label_pub_   = create_publisher<std_msgs::msg::String>("/brand_label", 10);
    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("/brand_overlay", 10);
  }

private:
  // Buffer recent images for sync with detections
void imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    // Maintain buffer for sync
    if (image_buffer_.size() >= 5) {
      image_buffer_.pop_front();
    }
    image_buffer_.push_back(img_msg);

    // --- WHOLE FRAME CLASSIFICATION (for debug/logging) ---
    cv::Mat frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    cv::Mat resized, rgb;
    cv::resize(frame, resized, cv::Size(input_width_, input_height_));
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0/255.0);

    // Prepare input tensor (CHW)
    std::vector<float> input_tensor(3 * input_height_ * input_width_);
    for (int c = 0; c < 3; ++c)
      for (int y = 0; y < input_height_; ++y)
        for (int x = 0; x < input_width_; ++x)
          input_tensor[c*input_height_*input_width_ + y*input_width_ + x] = rgb.at<cv::Vec3f>(y, x)[c];

    std::array<int64_t, 4> tensor_shape{1, 3, input_height_, input_width_};
    Ort::Value input_tensor_val = Ort::Value::CreateTensor<float>(
        mem_, input_tensor.data(), input_tensor.size(), tensor_shape.data(), tensor_shape.size());

    // Run classifier
    const char* in_names[]  = { input_name_.c_str() };
    const char* out_names[] = { output_name_.c_str() };
    auto outputs = session_->Run(Ort::RunOptions{nullptr}, in_names, &input_tensor_val, 1, out_names, 1);
    float* scores = outputs[0].GetTensorMutableData<float>();
    auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_classes = (shape.size() > 1) ? static_cast<int>(shape[1]) : static_cast<int>(shape[0]);
    int best = std::distance(scores, std::max_element(scores, scores + num_classes));
    float conf = scores[best];
    std::string name = (best < (int)class_names_.size() ? class_names_[best] : std::string("<unknown>"));

    RCLCPP_INFO(get_logger(), "FRAME TOP BRAND: %s (%.2f%%)", name.c_str(), conf * 100.0f);
}


  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr det_msg) {
    if (image_buffer_.empty()) return;
    // Find image with matching timestamp
    sensor_msgs::msg::Image::SharedPtr matched_image = nullptr;
    for (auto& img_msg : image_buffer_) {
      if (img_msg->header.stamp == det_msg->header.stamp) {
        matched_image = img_msg;
        break;
      }
    }
    if (!matched_image) {
      // If no exact match, use latest
      matched_image = image_buffer_.back();
      RCLCPP_WARN(get_logger(), "No matching image for detection timestamp, using latest image.");
    }
    // Convert ROS image to OpenCV BGR
    cv::Mat frame = cv_bridge::toCvCopy(matched_image, "bgr8")->image;

    std::string best_label = "";
    float best_conf = -1.0f;

    // Loop through each detected bottle
    for (const auto& det : det_msg->detections) {
      // Compute bounding box in original image coordinates
      double cx = det.bbox.center.position.x;
      double cy = det.bbox.center.position.y;
      double w  = det.bbox.size_x;
      double h  = det.bbox.size_y;
      int x = static_cast<int>(cx - w/2.0);
      int y = static_cast<int>(cy - h/2.0);
      int rect_w = static_cast<int>(w);
      int rect_h = static_cast<int>(h);
      // Clamp ROI to image bounds
      x = std::max(0, x);
      y = std::max(0, y);
      rect_w = std::min(rect_w, frame.cols - x);
      rect_h = std::min(rect_h, frame.rows - y);
      if (rect_w <= 0 || rect_h <= 0) continue;
      // Extract and resize ROI for the classifier
      cv::Mat roi = frame(cv::Rect(x, y, rect_w, rect_h));
      cv::Mat roi_resized;
      cv::resize(roi, roi_resized, cv::Size(input_width_, input_height_));
      cv::Mat roi_rgb;
      cv::cvtColor(roi_resized, roi_rgb, cv::COLOR_BGR2RGB);
      roi_rgb.convertTo(roi_rgb, CV_32F, 1.0/255.0);

      // Prepare input tensor (1 x 3 x H x W)
      std::vector<float> input_tensor(3 * input_height_ * input_width_);
      for (int c = 0; c < 3; ++c) {
        for (int yy = 0; yy < input_height_; ++yy) {
          for (int xx = 0; xx < input_width_; ++xx) {
            input_tensor[c * input_height_ * input_width_ + yy * input_width_ + xx] =
                roi_rgb.at<cv::Vec3f>(yy, xx)[c];
          }
        }
      }
      std::array<int64_t, 4> tensor_shape{1, 3, input_height_, input_width_};
      Ort::Value input_tensor_val = Ort::Value::CreateTensor<float>(
          mem_, input_tensor.data(), input_tensor.size(), tensor_shape.data(), tensor_shape.size());
      // Run classification model
      const char* in_names[]  = { input_name_.c_str() };
      const char* out_names[] = { output_name_.c_str() };
      auto output_tensors = session_->Run(Ort::RunOptions{nullptr}, in_names, &input_tensor_val, 1, out_names, 1);
      // Get output scores
      float* scores = output_tensors[0].GetTensorMutableData<float>();
      auto out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
      int num_classes = (out_shape.size() > 1) ? static_cast<int>(out_shape[1])
                                              : static_cast<int>(out_shape[0]);
      // Find highest score class
      int class_idx = 0;
      float max_score = scores[0];
      for (int j = 1; j < num_classes; ++j) {
        if (scores[j] > max_score) {
          max_score = scores[j];
          class_idx = j;
        }
      }
      std::string class_name;
      if (class_idx < (int)class_names_.size() && !class_names_[class_idx].empty()) {
        class_name = class_names_[class_idx];
      } else {
        class_name = std::string("Class ") + std::to_string(class_idx);
      }
      float confidence = max_score;
      // Track overall best (highest confidence) brand
      if (confidence > best_conf) {
        best_conf = confidence;
        best_label = class_name;
      }
      RCLCPP_INFO(get_logger(), "Candidate: %s, Confidence: %.2f%%", class_name.c_str(), confidence * 100.0);


      

      // Draw bounding box and label on the image
      cv::rectangle(frame, cv::Rect(x, y, rect_w, rect_h), cv::Scalar(0, 255, 0), 2);
      int text_x = x;
      int text_y = std::max(0, y - 5);
      char text[50];
      int conf_percent = static_cast<int>(confidence * 100.0f);
      std::snprintf(text, sizeof(text), "%s (%d%%)", class_name.c_str(), conf_percent);
      cv::putText(frame, text, cv::Point(text_x, text_y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }
    if (best_conf > conf_threshold_) {
        RCLCPP_INFO(get_logger(), "BRAND DETECTED: %s (%.2f%%)", best_label.c_str(), best_conf * 100.0);
    } else {
        RCLCPP_INFO(get_logger(), "NO BRAND DETECTED (top=%s, %.2f%%)", best_label.c_str(), best_conf * 100.0);
    }
    // Determine final label to publish
    std_msgs::msg::String label_msg;
    if (best_conf >= 0.0f) {
      // If at least one detection processed
      if (best_conf >= conf_threshold_) {
        label_msg.data = best_label;
      } else {
        label_msg.data = "Unknown";
      }
    } else {
      label_msg.data = "Unknown";
    }
    label_pub_->publish(label_msg);

    // Publish the annotated image
    auto overlay_msg = cv_bridge::CvImage(det_msg->header, "bgr8", frame).toImageMsg();
    overlay_pub_->publish(*overlay_msg);
  }

  // Members
  Ort::Env env_;
  Ort::MemoryInfo mem_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_, output_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr label_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> image_buffer_;
  std::vector<std::string> class_names_;
  std::string model_name_;
  int input_size_, input_width_, input_height_;
  double conf_threshold_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrandRecognition>());
  rclcpp::shutdown();
  return 0;
}
