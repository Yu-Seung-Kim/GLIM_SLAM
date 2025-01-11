#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/odometry/odometry_estimation_image.hpp>

#include <spdlog/spdlog.h>
#include <glim/util/logging.hpp>

#include <glim/odometry/callbacks.hpp>

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

AsyncOdometryEstimation::AsyncOdometryEstimation(const std::shared_ptr<OdometryEstimationBase>& odometry_estimation, bool enable_imu)
: odometry_estimation(odometry_estimation),
  logger(create_module_logger("odom")) {
  this->enable_imu = enable_imu;
  kill_switch = false;
  end_of_sequence = false;
  internal_frame_queue_size = 0;
  thread = std::thread([this] { run(); });
}

AsyncOdometryEstimation::~AsyncOdometryEstimation() {
  kill_switch = true;
  join();
}

void detect_features(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
void match_features(const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches);
Eigen::Isometry3d estimate_pose(const std::vector<cv::KeyPoint>& keypoints1, 
                                const std::vector<cv::KeyPoint>& keypoints2, 
                                const std::vector<cv::DMatch>& matches);
void AsyncOdometryEstimation::insert_image(const double stamp, const cv::Mat& image) {
  logger->info("insert_image");
  input_image_queue.push_back(std::make_pair(stamp, image));
}

void AsyncOdometryEstimation::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Eigen::Matrix<double, 7, 1> imu_data;
  imu_data << stamp, linear_acc, angular_vel;
  input_imu_queue.push_back(imu_data);
}

void AsyncOdometryEstimation::insert_frame(const PreprocessedFrame::Ptr& frame) {
  input_frame_queue.push_back(frame);
}

void AsyncOdometryEstimation::join() {
  end_of_sequence = true;
  if (thread.joinable()) {
    thread.join();
  }
}

int AsyncOdometryEstimation::workload() const {
  return input_frame_queue.size() + internal_frame_queue_size;
}

void AsyncOdometryEstimation::get_results(std::vector<EstimationFrame::ConstPtr>& estimation_results, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  estimation_results = output_estimation_results.get_all_and_clear();
  marginalized_frames = output_marginalized_frames.get_all_and_clear();
}

void AsyncOdometryEstimation::run() {
  //test
  logger->info("I'm in RUN");
  double last_imu_time = enable_imu ? 0.0 : std::numeric_limits<double>::max();
  std::deque<std::pair<double, cv::Mat>> images;
  std::deque<PreprocessedFrame::Ptr> raw_frames;
  cv::Mat last_image;
  Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();

  int frame_counter = 0;

  while (!kill_switch) {
    auto imu_frames = input_imu_queue.get_all_and_clear();
    auto new_images = input_image_queue.get_all_and_clear();
    auto new_raw_frames = input_frame_queue.get_all_and_clear();
    images.insert(images.end(), new_images.begin(), new_images.end());
    raw_frames.insert(raw_frames.end(), new_raw_frames.begin(), new_raw_frames.end());
    internal_frame_queue_size = raw_frames.size();

    if (images.empty() && imu_frames.empty() && raw_frames.empty()) {
      if (end_of_sequence) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    // IMU
    for (const auto& imu : imu_frames) {
      const double stamp = imu[0];
      const Eigen::Vector3d linear_acc = imu.block<3, 1>(1, 0);
      const Eigen::Vector3d angular_vel = imu.block<3, 1>(4, 0);
      odometry_estimation->insert_imu(stamp, linear_acc, angular_vel);

      last_imu_time = stamp;
    }
    // Camera
    while (!images.empty()) {
      logger->info("image");
      if (!end_of_sequence && images.front().first > last_imu_time) {
        logger->debug("waiting for IMU data (image_time={:.6f}, last_imu_time={:.6f})", images.front().first, last_imu_time);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      }
      const auto [timestamp, image] = images.front();
      images.pop_front();
      if (last_image.empty()){
        last_image = image.clone();
        continue;
      }
      Eigen::Isometry3d current_pose = process_images(last_image, image, last_pose);
      
      EstimationFrame::Ptr visualization_frame = std::make_shared<EstimationFrame>();
      visualization_frame->id = frame_counter;
      visualization_frame->stamp = timestamp;
      visualization_frame->frame_id = FrameID::LIDAR;
      visualization_frame->set_T_world_sensor(FrameID::LIDAR, current_pose);
      visualization_frame->T_world_lidar = current_pose;
      
      std::vector<EstimationFrame::ConstPtr> keyframes = {visualization_frame};
      Callbacks::on_update_frames(keyframes);
      
      last_image = image.clone();
      last_pose = current_pose;
      frame_counter++;
      
    }
    // Lidar
    while (!raw_frames.empty()) {
      if (!end_of_sequence && raw_frames.front()->scan_end_time > last_imu_time) {
        logger->debug("waiting for IMU data (scan_end_time={:.6f}, last_imu_time={:.6f})", raw_frames.front()->scan_end_time, last_imu_time);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      }
      logger->info("raw_image");

      const auto& frame = raw_frames.front();
      std::vector<EstimationFrame::ConstPtr> marginalized;
      auto state = odometry_estimation->insert_frame(frame, marginalized);

      output_estimation_results.push_back(state);
      output_marginalized_frames.insert(marginalized);
      raw_frames.pop_front();
      internal_frame_queue_size = raw_frames.size();
    }
  }

  auto marginalized = odometry_estimation->get_remaining_frames();
  output_marginalized_frames.insert(marginalized);
}

}  // namespace glim
