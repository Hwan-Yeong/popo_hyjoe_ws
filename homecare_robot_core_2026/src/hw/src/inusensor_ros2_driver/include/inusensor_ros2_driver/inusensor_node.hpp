#ifndef INUSENSOR_NODE_HPP
#define INUSENSOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

// TF2 관련 헤더
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "inusensor_ros2_driver/inusensor_manager.hpp"
#include "CalibrationData.h"

#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>
#include <chrono>
#include <atomic>
#include <thread>
#include <map>

namespace inusensor_ros2_driver
{

class InuSensorNode : public rclcpp::Node
{
public:
    explicit InuSensorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~InuSensorNode();

private:
    // Initialization methods
    void initializeParameters();
    void initializePublishers();
    void setupSensor();
    void startSensor();

    // ==================== SDK 콜백 함수들 (Register 모드용) ====================
    void onDepthFrameReceived(std::shared_ptr<InuDev::CDepthStream> stream, 
                             std::shared_ptr<const InuDev::CImageFrame> frame, 
                             InuDev::CInuError retCode);
    void onRGBFrameReceived(std::shared_ptr<InuDev::CImageStream> stream, 
                           std::shared_ptr<const InuDev::CImageFrame> frame, 
                           InuDev::CInuError retCode);

    // ==================== 타이머 기반 동기화 모드 ====================
    void initializeTimerMode();
    void timerCallback();
    bool getAndPublishSynchronizedFrames();

    // 타이머 모드와 콜백 모드 전환
    void enableTimerMode();
    void disableTimerMode();
    void enableCallbackMode();
    void disableCallbackMode();

    void setDepthProfile(std::string depth_profie);

    // IMU callback methods
    void onIMUAccFrame(std::shared_ptr<InuDev::CImuStream> stream, 
                      std::shared_ptr<const InuDev::CImuFrame> frame, 
                      InuDev::CInuError retCode);
    void onIMUGyroFrame(std::shared_ptr<InuDev::CImuStream> stream, 
                       std::shared_ptr<const InuDev::CImuFrame> frame, 
                       InuDev::CInuError retCode);

    // 최적화된 이미지 처리 메소드 (zero-copy 지향)
    bool processAndPublishDepthImage(std::shared_ptr<const InuDev::CImageFrame> frame);
    bool processAndPublishRGBImage(std::shared_ptr<const InuDev::CImageFrame> frame);

    // 타임스탬프를 받는 이미지 처리 메소드 (타이머 모드용)
    bool processAndPublishDepthImageWithTimestamp(std::shared_ptr<const InuDev::CImageFrame> frame, 
                                                 const rclcpp::Time& timestamp);
    bool processAndPublishRGBImageWithTimestamp(std::shared_ptr<const InuDev::CImageFrame> frame, 
                                               const rclcpp::Time& timestamp);
    
    // 동기화된 프레임 처리 (타이머 모드용)
    bool processAndPublishSynchronizedFrames(std::shared_ptr<const InuDev::CImageFrame> depthFrame,
                                           std::shared_ptr<const InuDev::CImageFrame> rgbFrame);
    
    
    // Publisher helper methods
    void publishIMUData(const sensor_msgs::msg::Imu& imuMsg);
    void publishCombinedIMUData();

    // Camera info creation
    sensor_msgs::msg::CameraInfo createCameraInfoFromYaml(const std::string& cameraName, const std::string& profile);


    // Cleanup
    void cleanup();

    // Sensor manager
    std::unique_ptr<InuSensorManager> sensorManager_;

    // ROS2 publishers
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher depthImagePub_;
    image_transport::Publisher rgbImagePub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depthInfoPub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbInfoPub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;

    // ==================== 타이머 기반 동기화 관련 ====================
    rclcpp::TimerBase::SharedPtr syncTimer_;
    std::mutex timerMutex_;

    // ==================== Watchdog 타이머 ====================
    rclcpp::TimerBase::SharedPtr watchdogTimer_;
    double watchdogTimeoutS_;
    void watchdogCallback();

    // TF broadcasters
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster_;

    // Camera info managers
    std::shared_ptr<camera_info_manager::CameraInfoManager> depthInfoManager_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> rgbInfoManager_;

    // ==================== 성능 최적화 변수 ====================
    // 카메라 정보
    sensor_msgs::msg::CameraInfo depthCameraInfo_;
    sensor_msgs::msg::CameraInfo rgbCameraInfo_;
    bool cameraInfoInitialized_;

    // 스레드 안전성  카운터
    std::atomic<uint64_t> depthFrameCount_;
    std::atomic<uint64_t> rgbFrameCount_;
    std::atomic<uint64_t> imuFrameCount_;
    std::atomic<uint64_t> syncFrameCount_;

    // 성능 모니터링
    std::chrono::steady_clock::time_point lastDepthTime_;
    std::chrono::steady_clock::time_point lastRGBTime_;
    std::mutex performanceMutex_;

    // Parameters
    std::string frameId_;
    std::string depthFrameId_;
    std::string rgbFrameId_;
    std::string imuFrameId_;
    std::string baseFrameId_;
    bool publishDepth_;
    bool publishRGB_;
    bool publishIMU_;
    bool rgh_depth_register_;
    int imuSyncTimeoutMs_;
    bool debugging_;
    bool useTimerMode_;           // 타이머 모드 활성화 여부
    double timerFrequency_;       // 타이머 주파수 (Hz)
    bool callbackModeActive_;     // 콜백 모드 활성화 상태
    bool timerModeActive_;        // 타이머 모드 활성화 상태
    int rgb_resolution_mode_;     // RGB 화질 0 : FHD , 1 : VGA
    int depth_resolution_mode_ ;  // Depth 화질 0 : 1080 x 720, 1: 848 x 480, 2: 544 x 360
    
    int rgb_res_num_ = 3;
    int depth_res_num_ = 3;

    std::string imageQosMode_;
    int imageQosDepth_;

    std::string rgb_res_profile_str;
    std::string depth_res_profile_str;

    enum RGB_Profile{
        HD,
        VGA,
        HVGA,
    };

    // enum 값을 문자열로 매핑하는 맵 생성
    std::map<RGB_Profile, std::string> RGB_ProfileToString = {
        {RGB_Profile::HD, "HD"},
        {RGB_Profile::VGA, "VGA"},
        {RGB_Profile::HVGA, "HVGA"},
    };

    std::string RGB_ProfileToStringConverter(RGB_Profile profile) {
        auto it = RGB_ProfileToString.find(profile);
        if (it != RGB_ProfileToString.end()) {
            return it->second;
        }
        return "HD"; // 찾을 수 없을 경우 기본값 반환
    };

    enum Depth_Profile{
        Full,
        V_Binning,
        Binning,
    };

    // enum 값을 문자열로 매핑하는 맵 생성
    std::map<Depth_Profile, std::string> Depth_ProfileToString = {
        {Depth_Profile::Full, "Full"},
        {Depth_Profile::V_Binning, "V_Binning"},
        {Depth_Profile::Binning, "Binning"},
    };

    std::string Depth_ProfileToStringConverter(Depth_Profile profile) {
        auto it = Depth_ProfileToString.find(profile);
        if (it != Depth_ProfileToString.end()) {
            return it->second;
        }
        return "Full"; // 찾을 수 없을 경우 기본값 반환
    };
    
    // Calibration data
    InuDev::CCalibrationData calibrationData_;
    bool calibrationLoaded_;
    
    // ==================== Recovery ====================
    int maxRecoveryAttempts_;
    int maxConsecutiveTimeouts_;
    std::atomic<bool> isRecovering_{false};
    std::atomic<bool> shutdownRequested_{false};
    std::atomic<int> consecutiveTimeoutCount_{0};
    std::thread recoveryThread_;
    std::chrono::steady_clock::time_point lastSuccessfulFrameTime_;

    bool isRecoveryTriggerError(InuDev::CInuError retCode);
    void triggerRecovery();
    void performRecovery();
    bool reinitializeStreams();
    void restartStreamingMode();
    void publishStatus(const std::string& status);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;

    // IMU data synchronization structure
    struct IMUData {
        bool hasAccData = false;
        bool hasGyroData = false;
        InuDev::CPoint3D accelerometer;
        InuDev::CPoint3D gyroscope;
        rclcpp::Time timestamp;
        std::mutex mutex;
        
        void reset() {
            std::lock_guard<std::mutex> lock(mutex);
            hasAccData = false;
            hasGyroData = false;
        }
        
        bool isComplete() const {
            return hasAccData && hasGyroData;
        }
    };
    
    IMUData currentImuData_;
};

} // namespace inusensor_ros2_driver

#endif // INUSENSOR_NODE_HPP