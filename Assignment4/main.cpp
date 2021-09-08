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

float calc_alpha(const cv::Point2f &p1, const cv::Point2f &p2) {
  float distance = cv::norm(p1 - p2);
  return fmax(1 - pow(distance, 2), 0.0);
}

void antialiasing_fill(const cv::Point2f &p, cv::Mat &window) {
  float pfx = floorf(p.x), pfy = floorf(p.y);
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      float alpha = calc_alpha(cv::Point2f(pfx + i, pfy + j), p);
      std::cout << alpha << '\n';
      float g = fmax(255.0f * alpha, window.at<cv::Vec3b>(pfy + j, pfx + i)[1]);
      window.at<cv::Vec3b>(pfy + j, pfx + i)[1] = fmin(g, 255.0f);
    }
  }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) {
  // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de
  // Casteljau's
  for (double t = 0; t <= 1.0; t += 0.001) {
    auto point = recursive_bezier(control_points, t);
    std::cout << point << '\n';
    // calculate the distance to decide the color visibility
    antialiasing_fill(point, window);
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
      // naive_bezier(control_points, window);
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
