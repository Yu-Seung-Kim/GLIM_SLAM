#ifndef GLIM_ODOMETRY_ESTIMATION_IMAGE_HPP
#define GLIM_ODOMETRY_ESTIMATION_IMAGE_HPP


#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

namespace glim {

void detect_features(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

void match_features(const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches);

Eigen::Isometry3d estimate_pose(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, const std::vector<cv::DMatch>& matches);

Eigen::Isometry3d process_images(const cv::Mat& last_image, const cv::Mat& current_image, Eigen::Isometry3d& last_pose);

}

#endif

