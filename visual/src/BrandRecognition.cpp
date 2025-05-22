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
//       env_(ORT_LOGGING_LEVEL_WARNING, "brand")
//   {
//     // Initialize
//     detected_ = false;
//     corona_index_ = -1;

//     // Parameters
//     model_name_  = declare_parameter("model_name", std::string("brand_cls_224.onnx"));
//     image_topic_ = declare_parameter("image_topic", std::string("/webcam/image_raw"));
//     input_size_  = declare_parameter("input_size", 224);
//     input_width_ = input_height_ = input_size_;
//     RCLCPP_INFO(get_logger(), "Resizing to %dx%d", input_width_, input_height_);

//     // Load class names from YAML
//     auto pkg = ament_index_cpp::get_package_share_directory("visual");
//     std::string names_path = pkg + "/model/brand_names.yaml";
//     std::ifstream ifs(names_path);
//     if (!ifs) {
//       RCLCPP_ERROR(get_logger(), "Failed to open %s", names_path.c_str());
//       rclcpp::shutdown();
//       return;
//     }
//     std::regex rx(R"(^\s*(\d+)\s*:\s*(\S+))");
//     std::string line;
//     while (std::getline(ifs, line)) {
//       std::smatch m;
//       if (std::regex_search(line, m, rx) && m.size() == 3) {
//         int idx = std::stoi(m[1]);
//         std::string name = m[2];
//         if ((int)class_names_.size() <= idx) class_names_.resize(idx+1);
//         class_names_[idx] = name;
//       }
//     }
//     for (size_t i = 0; i < class_names_.size(); ++i) {
//       if (class_names_[i] == "Corona") {
//         corona_index_ = (int)i;
//         break;
//       }
//     }
//     if (corona_index_ < 0) {
//       RCLCPP_ERROR(get_logger(), "'Corona' not found in names list");
//       rclcpp::shutdown();
//       return;
//     }

//     // Load ONNX model
//     std::string model_path = pkg + "/model/" + model_name_;
//     Ort::SessionOptions opts;
//     opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
//     session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);

//     // Get I/O names
//     Ort::AllocatorWithDefaultOptions allocator;
//     auto in_ptr = session_->GetInputNameAllocated(0, allocator);
//     input_name_ = in_ptr.get();
//     auto out_ptr = session_->GetOutputNameAllocated(0, allocator);
//     output_name_ = out_ptr.get();

//     // Subscribe
//     image_sub_ = create_subscription<sensor_msgs::msg::Image>(
//       image_topic_, 10,
//       std::bind(&BrandRecognition::imageCallback, this, _1)
//     );
//     RCLCPP_INFO(get_logger(), "Subscribed to '%s'", image_topic_.c_str());
//   }

// private:
//   void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//     if (detected_) return;
//     cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

//     // Preprocess
//     cv::Mat resized;
//     cv::resize(img, resized, cv::Size(input_width_, input_height_));
//     cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
//     resized.convertTo(resized, CV_32F, 1.0f/255.0f);

//     // HWC -> CHW
//     std::vector<float> tensor(3 * input_height_ * input_width_);
//     for (int c = 0; c < 3; ++c)
//       for (int y = 0; y < input_height_; ++y)
//         for (int x = 0; x < input_width_; ++x)
//           tensor[c*input_height_*input_width_ + y*input_width_ + x] = resized.at<cv::Vec3f>(y, x)[c];

//     // Create input tensor
//     std::array<int64_t,4> dims{1,3,input_height_,input_width_};
//     auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
//     auto input_tensor = Ort::Value::CreateTensor<float>(mem_info, tensor.data(), tensor.size(), dims.data(), dims.size());

//     // Inference
//     const char* in_names[]  = { input_name_.c_str() };  // use c_str() on std::string
//     const char* out_names[] = { output_name_.c_str() };
//     auto outputs = session_->Run(Ort::RunOptions(), in_names, &input_tensor, 1, out_names, 1);
//     RCLCPP_INFO(get_logger(),
//    "Running onnx with input size %d×%d, model expects [%lld×%lld]",
//    input_height_, input_width_,
//    session_->GetInputTypeInfo(0)
//        .GetTensorTypeAndShapeInfo()
//        .GetShape()[2],
//    session_->GetInputTypeInfo(0)
//        .GetTensorTypeAndShapeInfo()
//        .GetShape()[3]);

//     // Postprocess
//     float* scores = outputs[0].GetTensorMutableData<float>();
//     auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
//     int C = static_cast<int>(shape[1]);
//     int best = std::max_element(scores, scores + C) - scores;

//     if (best != corona_index_) {
//       RCLCPP_INFO(get_logger(), "BRAND NOT DETECTED");
//       return;
//     }

//     detected_ = true;
//     RCLCPP_INFO(get_logger(), "BRAND = 'Corona' (score=%.2f)" , scores[best]);
//   }

//   // Members
//   Ort::Env env_;
//   std::unique_ptr<Ort::Session> session_;
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
//   std::vector<std::string> class_names_;
//   int corona_index_;
//   int input_size_, input_height_, input_width_;
//   bool detected_;
//   std::string input_name_, output_name_, model_name_, image_topic_;
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<BrandRecognition>());
//   rclcpp::shutdown();
//   return 0;
// }



// src/BrandRecognition.cpp
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
//     // Parameters
//     model_name_     = declare_parameter("model_name", std::string("brand_cls_224.onnx"));
//     image_topic_    = declare_parameter("image_topic", std::string("/webcam/image_raw"));
//     input_size_     = declare_parameter("input_size", 224);
//     conf_threshold_ = declare_parameter("conf_threshold", 0.5f);

//     // Derived dims
//     input_w_ = input_h_ = input_size_;
//     RCLCPP_INFO(get_logger(), "Input size: %dx%d, confidence threshold=%.2f",
//                 input_w_, input_h_, conf_threshold_);

//     // Load class names
//     auto pkg_dir = ament_index_cpp::get_package_share_directory("visual");
//     std::string yaml_path = pkg_dir + "/model/brand_names.yaml";
//     std::ifstream ifs(yaml_path);
//     if (!ifs.is_open()) {
//       RCLCPP_ERROR(get_logger(), "Cannot open names file: %s", yaml_path.c_str());
//       rclcpp::shutdown(); return;
//     }
//     std::string line;
//     std::regex re(R"(^\s*(\d+)\s*:\s*(\S+))");
//     while (std::getline(ifs, line)) {
//       std::smatch m;
//       if (std::regex_match(line, m, re) && m.size()==3) {
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

//     // Get I/O names
//     Ort::AllocatorWithDefaultOptions allocator;
//     {
//       auto in_alloc = session_->GetInputNameAllocated(0, allocator);
//       input_name_ = in_alloc.get();
//       allocator.Free(in_alloc.release());
//     }
//     {
//       auto out_alloc = session_->GetOutputNameAllocated(0, allocator);
//       output_name_ = out_alloc.get();
//       allocator.Free(out_alloc.release());
//     }
//     RCLCPP_INFO(get_logger(), "Model I/O names: in='%s', out='%s'",
//                 input_name_.c_str(), output_name_.c_str());

//     // Subscribe to images
//     image_sub_ = create_subscription<sensor_msgs::msg::Image>(
//       image_topic_, 10,
//       std::bind(&BrandRecognition::imageCallback, this, _1)
//     );
//     RCLCPP_INFO(get_logger(), "Subscribed to '%s'", image_topic_.c_str());
//   }

// private:
//   void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//     // Convert ROS image to BGR CV
//     cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;

//     // Resize & convert to RGB float
//     cv::Mat resized, rgb;
//     cv::resize(bgr, resized, cv::Size(input_w_, input_h_));
//     cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
//     rgb.convertTo(rgb, CV_32F, 1.0f/255.0f);

//     // HWC -> CHW tensor
//     std::vector<float> tensor(3 * input_h_ * input_w_);
//     for (int c = 0; c < 3; ++c) {
//       for (int y = 0; y < input_h_; ++y) {
//         for (int x = 0; x < input_w_; ++x) {
//           tensor[c*input_h_*input_w_ + y*input_w_ + x] =
//             rgb.at<cv::Vec3f>(y, x)[c];
//         }
//       }
//     }

//     // Build ONNX tensor
//     std::array<int64_t,4> shape{1, 3, input_h_, input_w_};
//     Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
//     Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
//       mem, tensor.data(), tensor.size(), shape.data(), shape.size());

//     // Inference
//     const char* in_n[]  = { input_name_.c_str() };
//     const char* out_n[] = { output_name_.c_str() };
//     auto outputs = session_->Run(
//       Ort::RunOptions(), in_n, &input_tensor, 1, out_n, 1);

//     // Postprocess
//     float* scores = outputs[0].GetTensorMutableData<float>();
//     auto out_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
//     int C = (int)out_shape[1];
//     int best = std::max_element(scores, scores+C) - scores;
//     float conf = scores[best];
//     std::string name = (best < (int)names_.size()) ? names_[best] : "<unknown>";

//     if (conf >= conf_threshold_) {
//       RCLCPP_INFO(get_logger(), "BRAND = '%s' (%.2f)", name.c_str(), conf);
//     } else {
//       RCLCPP_INFO(get_logger(), "NO BRAND (top=%s, %.2f)", name.c_str(), conf);
//     }
//   }

//   // ONNX runtime
//   Ort::Env env_;
//   std::unique_ptr<Ort::Session> session_;

//   // ROS subscription
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

//   // I/O names
//   std::string input_name_, output_name_;

//   // parameters
//   std::string model_name_, image_topic_;
//   int input_size_, input_w_, input_h_;
//   float conf_threshold_;

//   // class names
//   std::vector<std::string> names_;
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<BrandRecognition>());
//   rclcpp::shutdown();
//   return 0;
// }

// src/BrandRecognition.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <regex>

using std::placeholders::_1;

class BrandRecognition : public rclcpp::Node {
public:
  BrandRecognition()
    : Node("brand_recognition"),
      env_(ORT_LOGGING_LEVEL_WARNING, "brand_recognition")
  {
    // Initialize detection flag
    detected_ = false;

    // Declare parameters
    model_name_     = declare_parameter("model_name", std::string("brand_cls_224.onnx"));
    image_topic_    = declare_parameter("image_topic", std::string("/webcam/image_raw"));
    input_size_     = declare_parameter("input_size", 224);
    conf_threshold_ = declare_parameter("conf_threshold", 0.5f);

    input_w_ = input_h_ = input_size_;
    RCLCPP_INFO(get_logger(), "Input size: %dx%d, conf_threshold=%.2f", input_w_, input_h_, conf_threshold_);

    // Load class names
    auto pkg_dir = ament_index_cpp::get_package_share_directory("visual");
    std::string names_path = pkg_dir + "/model/brand_names.yaml";
    std::ifstream ifs(names_path);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open names file: %s", names_path.c_str());
      rclcpp::shutdown();
      return;
    }
    std::string line;
    std::regex re(R"(^\s*(\d+)\s*:\s*(\S+))");
    while (std::getline(ifs, line)) {
      std::smatch m;
      if (std::regex_match(line, m, re) && m.size() == 3) {
        int idx = std::stoi(m[1]);
        if ((int)names_.size() <= idx) names_.resize(idx+1);
        names_[idx] = m[2];
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu class names", names_.size());

    // Load ONNX model
    std::string model_path = pkg_dir + "/model/" + model_name_;
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);

    // Fetch I/O names
    Ort::AllocatorWithDefaultOptions alloc;
    auto in_ptr = session_->GetInputNameAllocated(0, alloc);
    input_name_  = in_ptr.get();
    alloc.Free(in_ptr.release());
    auto out_ptr = session_->GetOutputNameAllocated(0, alloc);
    output_name_ = out_ptr.get();
    alloc.Free(out_ptr.release());
    RCLCPP_INFO(get_logger(), "Model I/O: in='%s', out='%s'", input_name_.c_str(), output_name_.c_str());

    // Subscribe to camera images
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&BrandRecognition::imageCallback, this, _1)
    );
    RCLCPP_INFO(get_logger(), "Subscribed to '%s'", image_topic_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // If already detected, print empty brand then exit
    if (detected_) {
      RCLCPP_INFO(get_logger(), "BRAND = ''");
      rclcpp::shutdown();
      return;
    }

    // Convert ROS image to OpenCV BGR
    cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    // Resize and convert to RGB float
    cv::Mat resized, rgb;
    cv::resize(bgr, resized, cv::Size(input_w_, input_h_));
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0f/255.0f);

    // Prepare input tensor CHW
    std::vector<float> tensor(3 * input_h_ * input_w_);
    for (int c = 0; c < 3; ++c) {
      for (int y = 0; y < input_h_; ++y) {
        for (int x = 0; x < input_w_; ++x) {
          tensor[c*input_h_*input_w_ + y*input_w_ + x] = rgb.at<cv::Vec3f>(y, x)[c];
        }
      }
    }
    std::array<int64_t,4> dims{1,3,input_h_,input_w_};
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input_tensor = Ort::Value::CreateTensor<float>(mem, tensor.data(), tensor.size(), dims.data(), dims.size());

    // Run inference
    const char* in_names[]  = { input_name_.c_str() };
    const char* out_names[] = { output_name_.c_str() };
    auto outputs = session_->Run(Ort::RunOptions(), in_names, &input_tensor, 1, out_names, 1);

    // Postprocess
    float* scores = outputs[0].GetTensorMutableData<float>();
    auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    int C = static_cast<int>(shape[1]);
    int best = std::distance(scores, std::max_element(scores, scores + C));
    float conf = scores[best];
    std::string name = (best < (int)names_.size() ? names_[best] : std::string("<unknown>"));

    if (conf >= conf_threshold_) {
      RCLCPP_INFO(get_logger(), "BRAND = '%s' (%.2f)", name.c_str(), conf);
      detected_ = true;
    } else {
      RCLCPP_INFO(get_logger(), "NO BRAND (top=%s, %.2f)", name.c_str(), conf);
    }
  }

  // Members
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::string input_name_, output_name_;
  std::string model_name_, image_topic_;
  int input_size_, input_w_, input_h_;
  float conf_threshold_;
  std::vector<std::string> names_;
  bool detected_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrandRecognition>());
  rclcpp::shutdown();
  return 0;
}