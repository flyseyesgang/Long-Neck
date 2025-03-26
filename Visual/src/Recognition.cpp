#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

using namespace cv;
using namespace std;

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor() : Node("image_processor") {
        this->declare_parameter<std::string>("image_path", "/home/rhys/ros2_ws/src/RS2/Visual/Bottles.png");
        process_image();
    }

    void process_image() {
        std::string image_path;
        this->get_parameter("image_path", image_path);

        Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image");
            return;
        }

        cv::Mat preprocessed_image;
        preprocess_image(image, preprocessed_image);
        std::vector<cv::Vec3f> circles;
        detect_circles(preprocessed_image, circles);

        if (!circles.empty()) {
            adaptive_circle_detection(preprocessed_image, circles);
        } else {
            RCLCPP_INFO(this->get_logger(), "No circles detected.");
        }
    }

private:
    void preprocess_image(const cv::Mat& input, cv::Mat& output) {
        if (input.channels() == 3) {
            cvtColor(input, output, COLOR_BGR2GRAY);
        } else {
            output = input.clone();
        }
        medianBlur(output, output, 5);
        equalizeHist(output, output);
    }

    void detect_circles(const cv::Mat& image, std::vector<cv::Vec3f>& circles) {
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }
        cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2.2);

        double dp = 1.2;  // Adjust the inverse resolution
        double minDist = gray.rows/16;  // Smaller distance if circles are close to each other
        double param1 = 225;  // Canny high threshold
        double param2 = 20;  // Decrease this to detect circles with lower confidence
        int minRadius = 10;  // Expected minimum radius of circles
        int maxRadius = 18;  // Expected maximum radius of circles

        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
    }

    void adaptive_circle_detection(cv::Mat& image, std::vector<cv::Vec3f>& circles) {
        double avg_radius = 0;
        for (const auto& circle : circles) {
            avg_radius += circle[2];  // circle[2] is the radius
        }
        avg_radius /= circles.size();

        for (int i = 0; i < 3; ++i) {
            double new_minDist = avg_radius * 1.5;
            double new_param2 = 100 - i * 10;
            std::vector<cv::Vec3f> new_circles;
            cv::HoughCircles(image, new_circles, cv::HOUGH_GRADIENT, 1, new_minDist, 200, new_param2, 0, 0);
            circles.insert(circles.end(), new_circles.begin(), new_circles.end());
        }

        visualize_circles(image, circles);
    }

    void visualize_circles(cv::Mat& image, const std::vector<cv::Vec3f>& circles) {
        cv::Mat color_image;
        cv::cvtColor(image, color_image, cv::COLOR_GRAY2BGR);
        for (const auto& circle : circles) {
            cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
            int radius = cvRound(circle[2]);
            cv::circle(color_image, center, radius, cv::Scalar(0, 255, 0), 3);
        }
        cv::imshow("Detected Circles", color_image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();


    cv::waitKey(0);  // Wait for user interaction


    return 0;
}

