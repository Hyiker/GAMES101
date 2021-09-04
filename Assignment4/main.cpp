#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) {
  if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", "
              << y << ")" << '\n';
    control_points.emplace_back(x, y);
  }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) {
  auto &p_0 = points[0];
  auto &p_1 = points[1];
  auto &p_2 = points[2];
  auto &p_3 = points[3];

  for (double t = 0.0; t <= 1.0; t += 0.001) {
    auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

    window.at<cv::Vec3b>(point.y, point.x)[1] =
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
  }
}

// polynomial expansion form of this recursive function:
// \sum_{k = 0}^{n}p_k*C(n, k)*u^k*(1-u)^(n-k)
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points,
                             float t) {
  // TODO: Implement de Casteljau's algorithm
  std::vector<cv::Point2f> cpc;
  for (int i = 0; i < control_points.size() - 1; i++) {
    auto cp = t * control_points[i] + (1.0f - t) * control_points[i + 1];
    cpc.push_back(cp);
  }
  if (cpc.size() == 1)
    return cpc[0];
  else
    return recursive_bezier(cpc, t);
}

void antialiasing_fill(const cv::Point2f &p, cv::Mat &window, int n = 1) {
  int x0 = p.x - n / 2, y0 = p.y - n / 2;
  for (int x = x0; x <= x0 + n; x++) {
    for (int y = y0; y <= y0 + n; y++) {
      float xf = x, yf = y;
      window.at<cv::Vec3b>(y, x)[1] = 255;
    }
  }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) {
  // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de
  // Casteljau's
  for (double t = 0; t <= 1.0; t += 0.001) {
    auto point = recursive_bezier(control_points, t);
    // calculate the distance to decide the color visibility
    antialiasing_fill(point, window, 0);
  }
  // recursive Bezier algorithm.
}

int main() {
  cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
  cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
  cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

  // create control points
  control_points.push_back(cv::Point2f(144, 367));
  control_points.push_back(cv::Point2f(225, 110));
  control_points.push_back(cv::Point2f(329, 549));
  control_points.push_back(cv::Point2f(432, 367));

  cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

  int key = -1;
  while (key != 27) {
    for (auto &point : control_points) {
      cv::circle(window, point, 3, {255, 255, 255}, 3);
    }

    if (control_points.size() == 4) {
      //      naive_bezier(control_points, window);
      bezier(control_points, window);

      cv::imshow("Bezier Curve", window);
      cv::imwrite("my_bezier_curve.png", window);
      key = cv::waitKey(0);

      return 0;
    }

    cv::imshow("Bezier Curve", window);
    key = cv::waitKey(20);
  }
  return 0;
}
