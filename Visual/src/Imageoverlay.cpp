#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

void checkForMatch(const Mat& refImage, const Mat& targetImage, const string& targetName) {
    Ptr<ORB> detector = ORB::create();
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;


    detector->detectAndCompute(refImage, noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(targetImage, noArray(), keypoints2, descriptors2);


    BFMatcher matcher(NORM_HAMMING);
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    double max_dist = 0; double min_dist = 100;
    for (int i = 0; i < descriptors1.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    vector<DMatch> good_matches;
    for (int i = 0; i < descriptors1.rows; i++) {
        if (matches[i].distance <= max(2*min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }

    double matched_percent = (double)good_matches.size() / keypoints1.size() * 100.0;

    if (matched_percent < 20.0) {
        cout << targetName << ": Corona bottle found." << endl;
    } else {
        cout << targetName << ": No Corona bottles found." << endl;
    }
}

int main() {
    string link = "/home/rhys/ros2_ws/src/RS2/Visual/";
    Mat refImage = imread(link + "coronabottle.jpg", IMREAD_GRAYSCALE);
    vector<string> targetImages = {link + "corona1.jpg", link + "corona2.jpg", link + "Sapporo.jpg"};

    if (refImage.empty()) {
        cout << "Error loading reference image." << endl;
        return -1;
    }

    for (const string& targetName : targetImages) {
        Mat targetImage = imread(targetName, IMREAD_GRAYSCALE);
        if (targetImage.empty()) {
            cout << "Error loading image: " << targetName << endl;
            continue;
        }
        checkForMatch(refImage, targetImage, targetName);
    }

    return 0;
}
