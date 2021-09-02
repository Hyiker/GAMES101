//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

template <typename T> T lerp(float x, const T &a, const T &b) {
  return a + x * (b - a);
}

class Texture {
private:
  cv::Mat image_data;

public:
  Texture(const std::string &name) {
    image_data = cv::imread(name);
    cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
    width = image_data.cols;
    height = image_data.rows;
  }

  int width, height;

  Eigen::Vector3f getColor(float u, float v) {
    auto u_img = u * width;
    auto v_img = (1 - v) * height;
    auto color = image_data.at<cv::Vec3b>(v_img, u_img);
    return Eigen::Vector3f(color[0], color[1], color[2]);
  }

  Eigen::Vector3f getColorBilinear(float u, float v) {
    float w = u * width, h = v * height;
    float w0 = floorf(w), w1 = w0 + 1.0f,
          h0 = floorf(h), h1 = h0 + 1.0f;
    float u0 = w0 / width, u1 = std::min(w1 / width, 1.0f),
          v0 = h0 / width, v1 = std::min(h1 / height, 1.0f);
    float s = u - u0, t = v - v0;
    auto v0_lerp = lerp(s, getColor(u0, v0), getColor(u1, v0));
    auto v1_lerp = lerp(s, getColor(u0, v1), getColor(u1, v1));
    auto v_lerp = lerp(t, v0_lerp, v1_lerp);
    return v_lerp;
  }
};
#endif // RASTERIZER_TEXTURE_H
