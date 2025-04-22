#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>  // for imread / IMREAD_*
#include <iostream>
#include <vector>
#include <string>
#include <getopt.h>

using namespace cv;
using namespace std;

// ——— print usage ——————————————————————————————————————————————————
static void print_usage(const char* prog) {
  cout << "Usage: " << prog << " --image <path> [OPTIONS]\n"
       << "OPTIONS:\n"
       << "  --image     path to input image (required)\n"
       << "  --dp        Hough dp (double, default=1.2)\n"
       << "  --minDist   Hough minDist (double, default=gray.rows/16)\n"
       << "  --param1    Canny high thresh (int, default=200)\n"
       << "  --param2    Accumulator thresh (int, default=20)\n"
       << "  --minRadius Hough minRadius (int, default=8)\n"
       << "  --maxRadius Hough maxRadius (int, default=40)\n"
       << "  -h,--help   show this help\n";
}

// ——— 1) Mask out everything except the largest quad (the esky top) —————————————————
static Mat mask_esky(const Mat& in) {
  Mat gray, edges;
  cvtColor(in, gray, COLOR_BGR2GRAY);
  GaussianBlur(gray, gray, Size(5,5), 1.5);
  Canny(gray, edges, 50,150);
  vector<vector<Point>> ctrs;
  findContours(edges, ctrs, RETR_LIST, CHAIN_APPROX_SIMPLE);

  double bestA = 0;
  vector<Point> bestQ;
  for (auto &c : ctrs) {
    vector<Point> approx;
    approxPolyDP(c, approx, arcLength(c,true)*0.02, true);
    if (approx.size()==4 && isContourConvex(approx)) {
      double A = fabs(contourArea(approx));
      if (A > bestA) {
        bestA = A;
        bestQ = approx;
      }
    }
  }
  if (bestQ.empty()) return in;

  Mat mask = Mat::zeros(in.size(), CV_8U);
  fillPoly(mask, vector<vector<Point>>{bestQ}, Scalar(255));
  Mat out;
  in.copyTo(out, mask);
  return out;
}

// ——— 2) Hough circle detection ——————————————————————————————————————
bool find_hough(const Mat& top,
                vector<Vec3f>& circles,
                double dp, double minDist,
                int p1, int p2,
                int minR, int maxR)
{
  Mat gray;
  cvtColor(top, gray, COLOR_BGR2GRAY);
  GaussianBlur(gray, gray, Size(9,9), 2.2);
  HoughCircles(gray, circles, HOUGH_GRADIENT,
               dp, minDist, p1, p2, minR, maxR);
  return !circles.empty();
}

// ——— 3) Ellipse fallback ——————————————————————————————————————————
bool find_ellipses(const Mat& top,
                   vector<Vec3f>& circles,
                   int minR, int maxR)
{
  Mat gray, edges;
  cvtColor(top, gray, COLOR_BGR2GRAY);
  Canny(gray, edges, 50, 150);
  vector<vector<Point>> ctrs;
  findContours(edges, ctrs, RETR_LIST, CHAIN_APPROX_NONE);

  for (auto &c : ctrs) {
    if (c.size() < 30) continue;
    RotatedRect e = fitEllipse(c);
    double r = min(e.size.width,e.size.height)/2;
    double ratio = e.size.width / e.size.height;
    if (r>=minR && r<=maxR && ratio>0.8 && ratio<1.2) {
      circles.emplace_back(e.center.x, e.center.y, r);
    }
  }
  return !circles.empty();
}

// ——— 4) HSV–based prune — keeps circles whose ROI has >10% “cap‑like” pixels ——
int prune_by_color(const Mat& top, vector<Vec3f>& circles) {
  Mat hsv; cvtColor(top, hsv, COLOR_BGR2HSV);
  int kept = 0;
  for (auto &c : circles) {
    int x = cvRound(c[0]-c[2]), y = cvRound(c[1]-c[2]),
        w = cvRound(2*c[2]), h = cvRound(2*c[2]);
    Rect roi(x,y,w,h);
    roi &= Rect(0,0,top.cols,top.rows);
    Mat patch = hsv(roi);

    int good=0, tot=0;
    for (int yy=0; yy<patch.rows; yy++) {
      for (int xx=0; xx<patch.cols; xx++) {
        Vec3b v = patch.at<Vec3b>(yy,xx);
        if (v[2]>100 && v[1]<100) good++;
        tot++;
      }
    }
    if (tot>0 && double(good)/tot > 0.1) {
      circles[kept++] = c;
    }
  }
  circles.resize(kept);
  return kept;
}

// ——— main ——————————————————————————————————————————————————————
int main(int argc, char** argv) {
  string image_path;
  double dp = 1.2, minDist = 30;
  int param1 = 200, param2 = 20, minRadius = 8, maxRadius = 40;

  static struct option long_opts[] = {
    {"image",     required_argument,nullptr,'i'},
    {"dp",        required_argument,nullptr,  0},
    {"minDist",   required_argument,nullptr,  0},
    {"param1",    required_argument,nullptr,  0},
    {"param2",    required_argument,nullptr,  0},
    {"minRadius", required_argument,nullptr,  0},
    {"maxRadius", required_argument,nullptr,  0},
    {"help",      no_argument,      nullptr,'h'},
    {nullptr,0,0,0}
  };

  int c, optidx=0;
  while ((c = getopt_long(argc, argv, "i:h", long_opts, &optidx)) != -1) {
    if (c=='i') image_path = optarg;
    else if (c=='h') { print_usage(argv[0]); return 0; }
    else if (c==0) {
      string k = long_opts[optidx].name;
      if      (k=="dp")        dp        = atof(optarg);
      else if (k=="minDist")   minDist   = atof(optarg);
      else if (k=="param1")    param1    = atoi(optarg);
      else if (k=="param2")    param2    = atoi(optarg);
      else if (k=="minRadius") minRadius = atoi(optarg);
      else if (k=="maxRadius") maxRadius = atoi(optarg);
    }
  }

  if (image_path.empty()) {
    cerr<<"Error: no --image provided\n";
    print_usage(argv[0]);
    return 1;
  }

  Mat in = imread(image_path);
  if (in.empty()) {
    cerr<<"Error: cannot load "<<image_path<<"\n";
    return 2;
  }

  // 1) Mask to esky top
  Mat top = mask_esky(in);
  imshow("Masked Esky", top); waitKey(1);

  // 2) Try Hough
  vector<Vec3f> circles;
  bool anyH = find_hough(top, circles, dp, minDist, param1, param2, minRadius, maxRadius);
  if (!anyH || circles.empty()) {
    circles.clear();
    find_ellipses(top, circles, minRadius, maxRadius);
  }

  // 3) Prune by color
  prune_by_color(top, circles);

  // 4) Draw & show
  Mat vis = in.clone();
  for (auto &c : circles) {
    circle(vis, Point(cvRound(c[0]),cvRound(c[1])), cvRound(c[2]), Scalar(0,255,0), 2);
  }
  imshow("Detector", vis); waitKey(1);

  // 5) Output JSON
  double meanD = 0;
  for (auto &c : circles) meanD += 2*c[2];
  if (!circles.empty()) meanD /= circles.size();

  cout << "{\"count\":"<<circles.size()
       <<",\"mean_diameter\":"<<meanD<<"}\n";
  return 0;
}
