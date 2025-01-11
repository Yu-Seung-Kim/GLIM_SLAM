#include <glim/odometry/odometry_estimation_image.hpp>



namespace glim {

void detect_features(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}

void match_features(const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches) {
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    matcher->match(descriptors1, descriptors2, matches);
}

Eigen::Isometry3d estimate_pose(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, const std::vector<cv::DMatch>& matches) {
    std::vector<cv::Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    // Essential matrix 계산
    cv::Mat essential_matrix = cv::findEssentialMat(points1, points2, cv::RANSAC);

    // 포즈 복구
    cv::Mat R, t;
    cv::recoverPose(essential_matrix, points1, points2, R, t);

    // Eigen 형식으로 변환
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;

    cv::cv2eigen(R, rotation);
    cv::cv2eigen(t, translation);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = rotation;
    pose.translation() = translation;

    return pose;
}

Eigen::Isometry3d process_images(const cv::Mat& last_image, const cv::Mat& current_image, Eigen::Isometry3d& last_pose) {
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    detect_features(last_image, keypoints1, descriptors1);
    detect_features(current_image, keypoints2, descriptors2);

    std::vector<cv::DMatch> matches;
    match_features(descriptors1, descriptors2, matches);

    if (matches.empty()) {
        throw std::runtime_error("No matches");
    }

    Eigen::Isometry3d relative_pose = estimate_pose(keypoints1, keypoints2, matches);
    return last_pose * relative_pose;
}

}

