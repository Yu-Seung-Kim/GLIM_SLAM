**VSLAM 사전 과제**

**이름:** 김유승

**이메일:** inspire156@gmail.com

**날짜:** 2025년 01월 07일 ~ 2025년 01월 11일

## 프로젝트 개요
- **목표:** CPU 전용 SLAM 시스템 구현 및 모노/스테레오 이미지 기반 SLAM 실현
- **환경:**Ubuntu 22.04 LTS
- **GLIM 설치 방법:**GLIM 설치는(https://koide3.github.io/glim/installation.html) Install from souce를 따라 Common dependencies를 설치하고 Install GLIM for ROS2를 따라 ros2_ws를 설치했습니다.

## 실행 방법
1. **터미널 경로 설정**
```cd ros2_ws```
2. **빌드 명령어 실행**
``` colcon build```
3. **GLIM 실행**
``` ros2 run glim_ros glim_rosbag {dataset_path}```

## CPU전용 SLAM 시스템 구현
**수정 내용**
GPU 의존성을 제거하고 CPU 전용으로 SLAM 시스템이 작동하도록 설정했습니다.
'/install/glim/share/glim/config/cofnig.json' 파일에서 아래 항목을 수정했습니다.
```
{
    "config_odometry": "config_odometry_cpu.json",
    "config_sub_mapping": "config_sub_mapping_cpu.json",
    "config_global_mapping": "config_global_mapping_cpu.json"
}
```
'CMakeLists.txt' 파일에서 'BUILD_WITH_CUDA'를 OFF로 설정했습니다.
```
option(BUILD_WITH_CUDA "Enable CUDA support" OFF)
```

## 모노/스테레오 이미지 기반 SLAM 실현s
모노/스테레오 카메라에서 제공하는 시각 데이터만을 이용하여 SLAM 시스템을 구현했습니다.
LiDAR나 IMU추가 센서를 사용하지 않고도 시각적 정보만으로 카메라의 움직임을 추적하였습니다. OpenCV 기반의 특징 검출 및 매칭을 활용하여 프레임 간 움직임을 계산하며, Eigen을 사용하여 카메라 위치를 표현합니다.

---

**코드 수정 내용**

코드는 '/src/glim/src/glim/src/odometry/' 경로에서 async_odometry_estimation.cpp를 수정하고,odometry_estimation_image.cpp를 추가했습니다.

**async_odometry_estimation.cpp**

async_odometry_estimation.cpp 에서images 큐에 존재하는 이미지 데이터를 process_image 함수를 통해 현재 카메라 pose를 추적하고, Callbacks::on_update_frames 콜백 함수로 카메라 pose를 시각화 합니다.

**odometry_estimation_image.cpp**

odometry_estimation_image.cpp는 process_image 함수를 포함하고 있으며 process_image 함수는 이전 프레임, 현재 프레임, 이전 카메라 pose를 입력 받아 현재 카메라 pose를 리턴합니다.
현재 카메라 pose를 추적하기 위해 이전 프레임과 현재 프레임에서 ORB 특징점 검출기를 이용하여 특징점을 검출하고 매칭합니다. 매칭된 특징점을 이용하여 essential_matrix를 ransac으로 계산한뒤 카메라의 회전과 병진 성분을 추출합니다. recoverPose를 통해 두 프레임 간 상대적인 카메라 변환을 복구합니다. 이전 pose와 복구된 상대적 변환을 결합하여 현재 pose를 계산합니다.


