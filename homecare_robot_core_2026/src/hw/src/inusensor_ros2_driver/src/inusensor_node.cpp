#include "inusensor_ros2_driver/inusensor_node.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cstring>
#include <std_msgs/msg/string.hpp>
//#define DEBUG_LOG

namespace inusensor_ros2_driver
{

InuSensorNode::InuSensorNode(const rclcpp::NodeOptions& options)
    : Node("inusensor_node", options)
    , calibrationLoaded_(false)
    , depthFrameCount_(0)
    , rgbFrameCount_(0)
    , imuFrameCount_(0)
    , imuSyncTimeoutMs_(50)  // 50ms timeout for IMU sync
    , cameraInfoInitialized_(false)
    , callbackModeActive_(false)
    , timerModeActive_(false)
{
    try {
        RCLCPP_INFO(this->get_logger(), "Starting InuSensor ROS2 Driver Node");

        // Initialize parameters
        RCLCPP_INFO(this->get_logger(), "Initializing parameters...");
        initializeParameters();

        // Initialize publishers
        RCLCPP_INFO(this->get_logger(), "Initializing publishers...");
        initializePublishers();

        // Setup sensor manager (with error handling)
        RCLCPP_INFO(this->get_logger(), "Setting up sensor...");
        setupSensor();

        // Start Sensor
        RCLCPP_INFO(this->get_logger(), "Start sensor...");
        startSensor();

        // Initialize appropriate mode based on parameters
        if (useTimerMode_) {
            RCLCPP_INFO(this->get_logger(), "Initializing timer mode...");
            initializeTimerMode();
            enableTimerMode();
        } else {
            RCLCPP_INFO(this->get_logger(), "Using callback mode...");
            enableCallbackMode();
        }

        RCLCPP_INFO(this->get_logger(), "InuSensor Node initialized successfully");
        
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during node initialization: %s", e.what());
        throw;
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception during node initialization");
        throw;
    }
}

InuSensorNode::~InuSensorNode()
{
    cleanup();
}

void InuSensorNode::initializeParameters()
{
    // Declare and get parameters
    this->declare_parameter("frame_id", "inusensor");
    this->declare_parameter("depth_frame_id", "inusensor_depth");
    this->declare_parameter("rgb_frame_id", "inusensor_rgb");
    this->declare_parameter("imu_frame_id", "inusensor_imu");
    this->declare_parameter("base_frame_id", "inusensor_base");  // TF 
    
    this->declare_parameter("publish_depth", true);
    this->declare_parameter("publish_rgb", true);
    this->declare_parameter("publish_imu", true);
    this->declare_parameter("rgh_depth_register", false);              
    this->declare_parameter("imu_sync_timeout_ms", 50);  // IMU sync timeout
    this->declare_parameter("use_timer_mode", false);        // 타이머 모드 활성화 여부
    this->declare_parameter("timer_frequency", 15.0);        // 타이머 주파수 (Hz)

    this->declare_parameter("rgb_resolution_mode", 0);  // 0 : 1280 x 720, 1: 640 x 480 , 2: HVGA
    this->declare_parameter("depth_resolution_mode", 0);  // 0 : 1080 x 720, 1: 848 x 480, 2: 544 x 360
    this->declare_parameter("image_qos_mode", "reliable");
    this->declare_parameter("image_qos_depth", 1);


    frameId_ = this->get_parameter("frame_id").as_string();
    depthFrameId_ = this->get_parameter("depth_frame_id").as_string();
    rgbFrameId_ = this->get_parameter("rgb_frame_id").as_string();
    imuFrameId_ = this->get_parameter("imu_frame_id").as_string();
    baseFrameId_ = this->get_parameter("base_frame_id").as_string();  // TF

    publishDepth_ = this->get_parameter("publish_depth").as_bool();
    publishRGB_ = this->get_parameter("publish_rgb").as_bool();
    publishIMU_ = this->get_parameter("publish_imu").as_bool();
    rgh_depth_register_ = this->get_parameter("rgh_depth_register").as_bool();         
    imuSyncTimeoutMs_ = this->get_parameter("imu_sync_timeout_ms").as_int();
    useTimerMode_ = this->get_parameter("use_timer_mode").as_bool();
    timerFrequency_ = this->get_parameter("timer_frequency").as_double();

    rgb_resolution_mode_ = this->get_parameter("rgb_resolution_mode").as_int();
    depth_resolution_mode_ = this->get_parameter("depth_resolution_mode").as_int();
    imageQosMode_ = this->get_parameter("image_qos_mode").as_string();
    imageQosDepth_ = this->get_parameter("image_qos_depth").as_int();

    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frameId_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publish depth: %s", publishDepth_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Publish RGB: %s", publishRGB_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Publish IMU: %s", publishIMU_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  IMU sync timeout: %d ms", imuSyncTimeoutMs_);
    RCLCPP_INFO(this->get_logger(), "  Use Timer Mode : %s", useTimerMode_ ? "true" : "false" );
    RCLCPP_INFO(this->get_logger(), "  Timer Frequency: %lf hz", timerFrequency_);

    if(rgb_resolution_mode_ >= rgb_res_num_ || rgb_resolution_mode_< 0 )
    { 
        RCLCPP_WARN(this->get_logger(), "  Wrong RGB resolution Setiing %d, change default", rgb_resolution_mode_);
        rgb_resolution_mode_ = 0;
    }
    if(depth_resolution_mode_ >= depth_res_num_ || depth_resolution_mode_< 0)
    {
        RCLCPP_WARN(this->get_logger(), "  Wrong Depth resolution Setiing %d, change default", depth_resolution_mode_);
        depth_resolution_mode_ = 0;
    }

    RGB_Profile rgb_profile = static_cast<RGB_Profile>(rgb_resolution_mode_);
    rgb_res_profile_str = RGB_ProfileToStringConverter(rgb_profile);

    ///specially depth는 HW에서 프로파일 지원이 됨. 따라서 바로 준비함.
    Depth_Profile depth_profile = static_cast<Depth_Profile>(depth_resolution_mode_);
    depth_res_profile_str = Depth_ProfileToStringConverter(depth_profile);

    RCLCPP_INFO(this->get_logger(), "  RGB Resolution  (0 : 1280 x 720, 1: 640 x 480, 2: HVGA) : %d", rgb_resolution_mode_);
    RCLCPP_INFO(this->get_logger(), "  Depth Resolution (0 : 1080 x 720, 1: 848 x 480, 2: 544 x 360) : %d", depth_resolution_mode_);
    RCLCPP_INFO(this->get_logger(), "  Image QoS Mode: %s", imageQosMode_.c_str());

    this->declare_parameter("max_recovery_attempts", 10);
    this->declare_parameter("max_consecutive_timeouts", 3);
    maxRecoveryAttempts_ = this->get_parameter("max_recovery_attempts").as_int();
    maxConsecutiveTimeouts_ = this->get_parameter("max_consecutive_timeouts").as_int();
    RCLCPP_INFO(this->get_logger(), "  Max recovery attempts: %d", maxRecoveryAttempts_);
    RCLCPP_INFO(this->get_logger(), "  Max consecutive timeouts: %d", maxConsecutiveTimeouts_);

    this->declare_parameter("watchdog_timeout_s", 10.0);
    watchdogTimeoutS_ = this->get_parameter("watchdog_timeout_s").as_double();
    RCLCPP_INFO(this->get_logger(), "  Watchdog timeout: %.1f s", watchdogTimeoutS_);
}

void InuSensorNode::initializePublishers()
{
    // ---- 공통 QoS 설정 (이미 쓰고 있는 imageQosMode_ 활용) ----
    auto image_qos = rclcpp::QoS(imageQosDepth_);
    if (imageQosMode_ == "best_effort") {
        image_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    } else {
        image_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    }

    if (publishDepth_) {
        // ✅ ImageTransport 객체 대신 create_publisher 사용
        depthImagePub_ = image_transport::create_publisher(
            this,
            "depth/image_raw",
            image_qos.get_rmw_qos_profile()
        );

        RCLCPP_INFO(this->get_logger(), "Depth publisher created (image_transport)");
        RCLCPP_INFO(this->get_logger(), "  → /compressed (auto by image_transport)");

        // CameraInfo는 기존대로 rclcpp publisher 사용
        auto info_qos = rclcpp::QoS(1);
        if (imageQosMode_ == "best_effort") {
            info_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        } else {
            info_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        }
        depthInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "depth/camera_info", info_qos);
    }

    if (publishRGB_) {
        rgbImagePub_ = image_transport::create_publisher(
            this,
            "rgb/image_raw",
            image_qos.get_rmw_qos_profile()
        );

        RCLCPP_INFO(this->get_logger(), "RGB publisher created (image_transport)");
        RCLCPP_INFO(this->get_logger(), "  → /compressed (auto by image_transport)");

        auto info_qos = rclcpp::QoS(1);
        if (imageQosMode_ == "best_effort") {
            info_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        } else {
            info_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        }
        rgbInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "rgb/camera_info", info_qos);
    }

    if (publishIMU_) {
        imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        RCLCPP_INFO(this->get_logger(), "IMU publisher created");
    }

    statusPub_ = this->create_publisher<std_msgs::msg::String>(
        "/inusensor/status", rclcpp::QoS(1).reliable());
    RCLCPP_INFO(this->get_logger(), "Status publisher created (/inusensor/status)");
}

// ==================== 새로운 타이머 모드 초기화 ====================
void InuSensorNode::initializeTimerMode()
{
    if (!useTimerMode_) return;
    
    // 타이머 주파수 검증
    if (timerFrequency_ <= 0.0 || timerFrequency_ > 100.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid timer frequency %.1f Hz, using 30.0 Hz", timerFrequency_);
        timerFrequency_ = 30.0;
    }
    
    // 타이머 생성 (비활성 상태로 시작)
    auto timer_period = std::chrono::duration<double>(1.0 / timerFrequency_);
    syncTimer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&InuSensorNode::timerCallback, this)
    );
    
    // 초기에는 타이머를 비활성화
    syncTimer_->cancel();
    
    RCLCPP_INFO(this->get_logger(), "Timer mode initialized with frequency: %.1f Hz", timerFrequency_);
}

void InuSensorNode::setupSensor()
{
    try {
        RCLCPP_INFO(this->get_logger(), "Creating sensor manager...");
        sensorManager_ = std::make_unique<InuSensorManager>();
        
        RCLCPP_INFO(this->get_logger(), "Initializing sensor...");
        
        ////special position for depth fuck
        setDepthProfile(depth_res_profile_str);

        // Initialize sensor following the original API sequence
        if (!sensorManager_->initializeSensor()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor - sensor may not be connected");
            // Don't throw exception, just disable sensor functionality
            sensorManager_.reset();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sensor initialized successfully");

        // Initialize streams based on what we want to publish
        if (rgh_depth_register_) {
            sensorManager_->rgb_to_depthRegistration_ = rgh_depth_register_;

            RCLCPP_INFO(this->get_logger(), "Initializing registration stream...");
            if (!sensorManager_->initializeRGBRegister()) {
                RCLCPP_WARN(this->get_logger(), "Failed to initialize registration stream ");
            }
        }


        // Initialize streams based on what we want to publish
        if (publishDepth_) {
            RCLCPP_INFO(this->get_logger(), "Initializing depth stream...");
            if (!sensorManager_->initializeDepth()) {
                RCLCPP_WARN(this->get_logger(), "Failed to initialize depth stream - disabling depth publishing");
                publishDepth_ = false;
            }
        }

        if (publishRGB_) {
            RCLCPP_INFO(this->get_logger(), "Initializing RGB stream...");
            if (!sensorManager_->initializeRGB()) {
                RCLCPP_WARN(this->get_logger(), "Failed to initialize RGB stream - disabling RGB publishing");
                publishRGB_ = false;
            }
        }

        if (publishIMU_) {
            RCLCPP_INFO(this->get_logger(), "Initializing IMU streams...");
            if (!sensorManager_->initializeIMU()) {
                RCLCPP_WARN(this->get_logger(), "Failed to initialize IMU streams - disabling IMU publishing");
                publishIMU_ = false;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during sensor setup: %s", e.what());
        sensorManager_.reset();
        RCLCPP_WARN(this->get_logger(), "Continuing without sensor - node will run in simulation mode");
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception during sensor setup");
        sensorManager_.reset();
        RCLCPP_WARN(this->get_logger(), "Continuing without sensor - node will run in simulation mode");
    }
}




void InuSensorNode::startSensor()
{
    try {

        // Initialize streams based on what we want to publish
        if (rgh_depth_register_) {

            RCLCPP_INFO(this->get_logger(), "Start registration stream...");
            if (!sensorManager_->startRGBRegister()) {
                RCLCPP_WARN(this->get_logger(), "Failed to start registration stream ");
            } else {
                RCLCPP_INFO(this->get_logger(), "registration stream started successfully");
            }
        }


        // Initialize streams based on what we want to publish
        if (publishDepth_) {
            RCLCPP_INFO(this->get_logger(), "Start depth stream...");
            if (!sensorManager_->startDepth()) {
                RCLCPP_WARN(this->get_logger(), "Failed to start depth stream - disabling depth publishing");
                publishDepth_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Depth stream started successfully");
            }
        }

        if (publishRGB_) {
            RCLCPP_INFO(this->get_logger(), "Start RGB stream...");
            if (!sensorManager_->startRGB()) {
                RCLCPP_WARN(this->get_logger(), "Failed to start RGB stream - disabling RGB publishing");
                publishRGB_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "RGB stream started successfully");
            }
        }

        if (publishIMU_) {
            RCLCPP_INFO(this->get_logger(), "Start IMU streams...");
            if (!sensorManager_->startIMU()) {
                RCLCPP_WARN(this->get_logger(), "Failed to start IMU streams - disabling IMU publishing");
                publishIMU_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "IMU streams started successfully");
            }
        }

        // Register SDK callbacks for all streams (필수: SDK 동작을 위해)
        if (publishDepth_ && sensorManager_) {
            RCLCPP_INFO(this->get_logger(), "Registering depth SDK callback...");
            sensorManager_->registerDepthCallback(
                std::bind(&InuSensorNode::onDepthFrameReceived, this, 
                         std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }

        if (publishRGB_ && sensorManager_) {
            RCLCPP_INFO(this->get_logger(), "Registering RGB SDK callback...");
            sensorManager_->registerRGBCallback(
                std::bind(&InuSensorNode::onRGBFrameReceived, this, 
                         std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }

        if (publishIMU_ && sensorManager_) {
            RCLCPP_INFO(this->get_logger(), "Registering IMU SDK callbacks...");
            sensorManager_->registerIMUAccCallback(
                std::bind(&InuSensorNode::onIMUAccFrame, this, 
                         std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            sensorManager_->registerIMUGyroCallback(
                std::bind(&InuSensorNode::onIMUGyroFrame, this, 
                         std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }

        if (sensorManager_) {
            RCLCPP_INFO(this->get_logger(), "Sensor setup completed successfully");
            publishStatus("ACTIVE");

            // Watchdog 타이머 초기화 — 스트림 시작 이후 시각 기준점 설정
            auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(performanceMutex_);
                lastDepthTime_ = now;
                lastRGBTime_   = now;
            }
            watchdogTimer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&InuSensorNode::watchdogCallback, this));
            RCLCPP_INFO(this->get_logger(),
                        "Watchdog timer started (timeout: %.1f s, check: 5 s)", watchdogTimeoutS_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Sensor setup completed with warnings - running in simulation mode");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during sensor setup: %s", e.what());
        sensorManager_.reset();
        RCLCPP_WARN(this->get_logger(), "Continuing without sensor - node will run in simulation mode");
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception during sensor setup");
        sensorManager_.reset();
        RCLCPP_WARN(this->get_logger(), "Continuing without sensor - node will run in simulation mode");
    }
}

// ==================== 모드 전환 함수들 ====================
void InuSensorNode::setDepthProfile(std::string depth_profile)
{
        if (depth_profile == "V_Binning" || depth_profile == "v_binning" || depth_profile == "VerticalBinning") {
            RCLCPP_INFO(this->get_logger(), "Depth Profile setting colplete : VerticalBinning");
            sensorManager_->setDepthResolutionVerticalBinning();

        } else if (depth_profile == "Binning" || depth_profile == "binning") {
            RCLCPP_INFO(this->get_logger(), "Depth Profile setting colplete : Binning");
            sensorManager_->setDepthResolutionBinning();
        }
        else if (depth_profile == "Full" || depth_profile == "full")
        {
            RCLCPP_INFO(this->get_logger(), "Depth Profile setting colplete : Full");
            sensorManager_->setDepthResolutionFull();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Depth Profile setting colplete : Default(FUll)");
            sensorManager_->setDepthResolutionDefault();
        }
}

void InuSensorNode::enableTimerMode()
{
    if (timerModeActive_) return;
    
    RCLCPP_INFO(this->get_logger(), "Enabling timer mode...");
    
    // 콜백 모드 비활성화
    disableCallbackMode();
    
    // 타이머 모드 활성화
    if (syncTimer_) {
        syncTimer_->reset();  // 타이머 다시 시작
        timerModeActive_ = true;
        RCLCPP_INFO(this->get_logger(), "Timer mode enabled (%.1f Hz)", timerFrequency_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to enable timer mode: timer not initialized");
    }
}

void InuSensorNode::disableTimerMode()
{
    if (!timerModeActive_) return;
    
    RCLCPP_INFO(this->get_logger(), "Disabling timer mode...");
    
    // 타이머 중지
    if (syncTimer_) {
        syncTimer_->cancel();
    }
    timerModeActive_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Timer mode disabled");
}

void InuSensorNode::enableCallbackMode()
{
    if (callbackModeActive_) return;
    
    RCLCPP_INFO(this->get_logger(), "Enabling callback mode...");
    
    // 타이머 모드 비활성화
    disableTimerMode();
    
    // SDK 콜백 등록
    if (publishDepth_ && sensorManager_) {
        RCLCPP_INFO(this->get_logger(), "Registering depth SDK callback...");
        sensorManager_->registerDepthCallback(
            std::bind(&InuSensorNode::onDepthFrameReceived, this, 
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    if (publishRGB_ && sensorManager_) {
        RCLCPP_INFO(this->get_logger(), "Registering RGB SDK callback...");
        sensorManager_->registerRGBCallback(
            std::bind(&InuSensorNode::onRGBFrameReceived, this, 
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    if (publishIMU_ && sensorManager_) {
        RCLCPP_INFO(this->get_logger(), "Registering IMU SDK callbacks...");
        sensorManager_->registerIMUAccCallback(
            std::bind(&InuSensorNode::onIMUAccFrame, this, 
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        sensorManager_->registerIMUGyroCallback(
            std::bind(&InuSensorNode::onIMUGyroFrame, this, 
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }
    
    callbackModeActive_ = true;
    RCLCPP_INFO(this->get_logger(), "Callback mode enabled");
}

void InuSensorNode::disableCallbackMode()
{
    if (!callbackModeActive_) return;
    
    RCLCPP_INFO(this->get_logger(), "Disabling callback mode...");
    
    // SDK 콜백 해제는 Manager에서 처리됨 (새 콜백으로 덮어쓰거나 nullptr로 설정)
    // 여기서는 단순히 플래그만 설정
    callbackModeActive_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Callback mode disabled");
}

// ==================== 타이머 콜백 함수 ====================
void InuSensorNode::timerCallback()
{
    if (isRecovering_) return;
    std::lock_guard<std::mutex> lock(timerMutex_);

    if (!timerModeActive_) return;
    
    try {
        #ifdef DEBUG_LOG
        auto timer_start = std::chrono::high_resolution_clock::now();
        #endif
        
        if (getAndPublishSynchronizedFrames()) {
            syncFrameCount_++;
            
            #ifdef DEBUG_LOG
            if (syncFrameCount_ % 100 == 0) {
                auto timer_end = std::chrono::high_resolution_clock::now();
                auto timer_time = std::chrono::duration_cast<std::chrono::microseconds>(timer_end - timer_start);
                RCLCPP_INFO(this->get_logger(), 
                    "🔄 Timer sync #%lu - Processing time: %ld μs (%.2f ms)", 
                    static_cast<unsigned long>(syncFrameCount_),
                    timer_time.count(), 
                    timer_time.count() / 1000.0);
            }
            #endif
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Exception in timer callback: %s", e.what());
    }
}

bool InuSensorNode::getAndPublishSynchronizedFrames()
{
    if (!sensorManager_) return false;
    
    // SDK GetFrame 방식으로 프레임 획득 (API 예제 참고)
    std::shared_ptr<const InuDev::CImageFrame> depthFrame;
    std::shared_ptr<const InuDev::CImageFrame> rgbFrame;
    
    bool depthSuccess = false;
    bool rgbSuccess = false;
    
    // Depth 프레임 획득
    if (publishDepth_) {
        auto depthStream = sensorManager_->getDepthStream();
        if (depthStream) {
            InuDev::CInuError retCode = depthStream->GetFrame(depthFrame);
            if (retCode == InuDev::eOK && depthFrame && depthFrame->Valid) {
                lastSuccessfulFrameTime_ = std::chrono::steady_clock::now();
                consecutiveTimeoutCount_ = 0;
                // Watchdog 시각 갱신
                {
                    std::lock_guard<std::mutex> lock(performanceMutex_);
                    lastDepthTime_ = lastSuccessfulFrameTime_;
                }
                depthSuccess = true;
            } else {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Failed to get depth frame: %s", std::string(retCode).c_str());
                if (isRecoveryTriggerError(retCode)) {
                    triggerRecovery();
                    return false;
                }
                if (static_cast<int>(retCode) == InuDev::eTimeoutError) {
                    if (++consecutiveTimeoutCount_ >= maxConsecutiveTimeouts_) {
                        RCLCPP_WARN(this->get_logger(),
                                    "[Recovery] %d 회 연속 타임아웃 감지. Recovery 시작...",
                                    maxConsecutiveTimeouts_);
                        triggerRecovery();
                        return false;
                    }
                }
            }
        }
    }

    // RGB 프레임 획득
    if (publishRGB_) {
        std::shared_ptr<InuDev::CImageStream> rgbStream;

        // RGB Registration 모드에 따라 스트림 선택
        if (rgh_depth_register_) {
            rgbStream = sensorManager_->getRGBRegStream();
        } else {
            rgbStream = sensorManager_->getRGBStream();
        }

        if (rgbStream) {
            InuDev::CInuError retCode = rgbStream->GetFrame(rgbFrame);
            if (retCode == InuDev::eOK && rgbFrame && rgbFrame->Valid) {
                // Watchdog 시각 갱신
                {
                    std::lock_guard<std::mutex> lock(performanceMutex_);
                    lastRGBTime_ = std::chrono::steady_clock::now();
                }
                rgbSuccess = true;
            } else {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Failed to get RGB frame: %s", std::string(retCode).c_str());
                if (isRecoveryTriggerError(retCode)) {
                    triggerRecovery();
                    return false;
                }
            }
        }
    }
    
    // 동기화된 프레임 처리 및 발행
    if ((publishDepth_ && depthSuccess) || (publishRGB_ && rgbSuccess)) {
        return processAndPublishSynchronizedFrames(
            depthSuccess ? depthFrame : nullptr,
            rgbSuccess ? rgbFrame : nullptr
        );
    }
    
    return false;
}

// ==================== 동기화된 프레임 처리 ====================
bool InuSensorNode::processAndPublishSynchronizedFrames(std::shared_ptr<const InuDev::CImageFrame> depthFrame,
                                                       std::shared_ptr<const InuDev::CImageFrame> rgbFrame)
{
    try {
        // 공통 타임스탬프 생성 (동기화)
        rclcpp::Time commonTimestamp = this->get_clock()->now();
        
        bool depthPublished = false;
        bool rgbPublished = false;
        
        // Depth 이미지 처리 및 발행
        if (depthFrame && publishDepth_) {
            depthPublished = processAndPublishDepthImageWithTimestamp(depthFrame, commonTimestamp);
        }
        
        // RGB 이미지 처리 및 발행
        if (rgbFrame && publishRGB_) {
            rgbPublished = processAndPublishRGBImageWithTimestamp(rgbFrame, commonTimestamp);
        }
        
        return depthPublished || rgbPublished;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Exception in synchronized frame processing: %s", e.what());
        return false;
    }
}

// ==================== 타임스탬프를 받는 이미지 처리 함수들 ====================
bool InuSensorNode::processAndPublishDepthImageWithTimestamp(std::shared_ptr<const InuDev::CImageFrame> frame, 
                                                            const rclcpp::Time& timestamp)
{
    try {
        // 이미지 정보 획득
        int height = frame->Height();
        int width = frame->Width();
        
        // ROS2 메시지 직접 생성
        sensor_msgs::msg::Image depthMsg;
        
        depthMsg.header.stamp = timestamp;
        depthMsg.header.frame_id = depthFrameId_;
        depthMsg.height = static_cast<uint32_t>(height);
        depthMsg.width = static_cast<uint32_t>(width);
        depthMsg.encoding = "16UC1";  // 원본 16-bit 그대로 사용
        depthMsg.is_bigendian = false;
        depthMsg.step = static_cast<uint32_t>(width * sizeof(uint16_t));
        
        // 데이터 공간 할당 및 직접 복사
        size_t data_size = depthMsg.step * depthMsg.height;
        depthMsg.data.resize(data_size);
        
        // SDK 데이터를 직접 memcpy (변환/처리 없이)
        const void* sdk_data = frame->GetData();
        std::memcpy(depthMsg.data.data(), sdk_data, data_size);
        
        sensor_msgs::msg::CameraInfo depthCameraInfo_ = createCameraInfoFromYaml("depth", depth_res_profile_str);
        depthCameraInfo_.header.stamp = timestamp;
        depthCameraInfo_.header.frame_id = depthFrameId_;
        
        // 즉시 발행
        depthInfoPub_->publish(depthCameraInfo_);
        depthImagePub_.publish(depthMsg);
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Depth 처리 예외: %s", e.what());
        return false;
    }
}

bool InuSensorNode::processAndPublishRGBImageWithTimestamp(
    std::shared_ptr<const InuDev::CImageFrame> frame,
    const rclcpp::Time& timestamp)
{
    try {
        // 1) 원본 사이즈 (HD 입력)
        const int src_h = frame->Height();
        const int src_w = frame->Width();

        // 2) 현재 노드/파라미터로부터 RGB 프로필 문자열을 가져온다고 가정 (예: "HD","VGA","HVGA")
        //    ex) this->rgb_profile_str_ 가 멤버로 존재한다고 가정
        const std::string profile = rgb_res_profile_str; // 필요 시 파라미터에서 읽어오세요.

        // 3) 타깃 해상도/크롭/스케일 계산 (선택지 3)
        int dst_w = src_w, dst_h = src_h;
        int crop_x = 0, crop_y = 0, crop_w = src_w, crop_h = src_h;
        double s = 1.0;

        if (profile == "HD") {
            // 1280x720 → 그대로
            dst_w = 1280; dst_h = 720;
            crop_w = src_w; crop_h = src_h; crop_x = 0; crop_y = 0;
            s = 1.0;
        } else if (profile == "VGA") {
            // 1280x720 → (센터 크롭 960x720) → 등비 2/3 → 640x480
            dst_w = 640; dst_h = 480;
            crop_w = 960; crop_h = 720;
            crop_x = (src_w - crop_w) / 2;  // 160
            crop_y = 0;
            s = static_cast<double>(dst_w) / static_cast<double>(crop_w); // 640/960 = 2/3
        } else if (profile == "HVGA") {
            // 1280x720 → 등비 1/2 → 640x360 (크롭 없음)
            dst_w = 640; dst_h = 360;
            crop_w = src_w; crop_h = src_h; crop_x = 0; crop_y = 0;
            s = static_cast<double>(dst_w) / static_cast<double>(crop_w); // 640/1280 = 0.5
        } else {
            // 안전장치: 기본 HD
            dst_w = src_w; dst_h = src_h;
            crop_w = src_w; crop_h = src_h; crop_x = 0; crop_y = 0;
            s = 1.0;
        }

        // 4) SDK 메모리를 OpenCV Mat로 래핑 (소스가 BGRA라고 가정: 채널 4)
        const void* sdk_data = frame->GetData();
        const InuDev::byte* src_ptr = static_cast<const InuDev::byte*>(sdk_data);

        // CV_8UC4: BGRA/ RGBA 8-bit × 4채널
        cv::Mat src_4ch(src_h, src_w, CV_8UC4, const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(src_ptr)));

        // 5) 센터 크롭 (ROI)
        cv::Rect roi(crop_x, crop_y, crop_w, crop_h);
        cv::Mat cropped_4ch = src_4ch(roi);

        // 6) 등비 스케일 리사이즈 (보간: 영역/선형 적절히 선택)
        cv::Mat resized_4ch;
        cv::resize(cropped_4ch, resized_4ch, cv::Size(dst_w, dst_h),
                   0, 0, (s < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR));

        // 7) 채널 변환: BGRA → BGR  (SDK가 RGBA면 COLOR_RGBA2BGR 사용)
        cv::Mat bgr; 
        cv::cvtColor(resized_4ch, bgr, cv::COLOR_BGRA2BGR);

        // 8) ROS Image 메시지 구성
        sensor_msgs::msg::Image rgbMsg;
        rgbMsg.header.stamp = timestamp;
        rgbMsg.header.frame_id = rgbFrameId_;
        rgbMsg.height = static_cast<uint32_t>(bgr.rows);
        rgbMsg.width  = static_cast<uint32_t>(bgr.cols);
        rgbMsg.encoding = "bgr8";          // B,G,R 순
        rgbMsg.is_bigendian = false;
        rgbMsg.step = static_cast<uint32_t>(bgr.cols * 3);
        rgbMsg.data.assign(bgr.data, bgr.data + (bgr.rows * bgr.step));

        // 9) CameraInfo 생성 (프로필 반영)
        sensor_msgs::msg::CameraInfo cameraInfo;
        if (!rgh_depth_register_) {
            cameraInfo = createCameraInfoFromYaml("rgb", profile);   // ← 프로필 문자열 반영
        } else {
            // RGB->Depth 정합 모드라면 기존 로직 유지(필요 시 별도 정책 반영)
            cameraInfo = createCameraInfoFromYaml("depth", "HD");
        }
        cameraInfo.header.stamp = timestamp;
        cameraInfo.header.frame_id = rgbFrameId_;

        // 10) Publish
        rgbInfoPub_->publish(cameraInfo);
        rgbImagePub_.publish(rgbMsg);  //

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "RGB 처리 예외: %s", e.what());
        return false;
    }
}


// ==================== SDK 콜백 함수들 (콜백 모드용) ====================

void InuSensorNode::onDepthFrameReceived(std::shared_ptr<InuDev::CDepthStream> stream,
                                        std::shared_ptr<const InuDev::CImageFrame> frame,
                                        InuDev::CInuError retCode)
{
    if (isRecovering_) return;
    // 타이머 모드가 활성화된 경우 콜백 무시
    if (timerModeActive_) return;

    if (retCode != InuDev::eOK) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Error receiving depth frame from SDK: %s", std::string(retCode).c_str());
        if (isRecoveryTriggerError(retCode)) {
            triggerRecovery();
        }
        return;
    }

    if (!frame->Valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Invalid depth frame %lu received from SDK", frame->FrameIndex);
        return;
    }

    // Watchdog 시각 갱신
    {
        std::lock_guard<std::mutex> lock(performanceMutex_);
        lastDepthTime_ = std::chrono::steady_clock::now();
    }

    try {
        #ifdef DEBUG_LOG
        // 성능 모니터링
        auto callback_start = std::chrono::high_resolution_clock::now();

        if (processAndPublishDepthImage(frame)) {
            depthFrameCount_++;
        }
        
        // 시간 측정 및 출력  
        auto callback_end = std::chrono::high_resolution_clock::now();
        auto callback_time = std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start);

        if (depthFrameCount_ % 100 == 0) {
            RCLCPP_WARN(this->get_logger(), "🔍 Depth 처리시간: %ld μs (%.2f ms)", 
                    callback_time.count(), callback_time.count() / 1000.0);
        }
        #else
        processAndPublishDepthImage(frame);
        #endif
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Exception in depth frame processing: %s", e.what());
    }
}

void InuSensorNode::onRGBFrameReceived(std::shared_ptr<InuDev::CImageStream> stream,
                                      std::shared_ptr<const InuDev::CImageFrame> frame,
                                      InuDev::CInuError retCode)
{
    if (isRecovering_) return;
    // 타이머 모드가 활성화된 경우 콜백 무시
    if (timerModeActive_) return;

    if (retCode != InuDev::eOK) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Error receiving RGB frame from SDK: %s", std::string(retCode).c_str());
        if (isRecoveryTriggerError(retCode)) {
            triggerRecovery();
        }
        return;
    }

    if (!frame->Valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Invalid RGB frame %lu received from SDK", frame->FrameIndex);
        return;
    }

    // Watchdog 시각 갱신
    {
        std::lock_guard<std::mutex> lock(performanceMutex_);
        lastRGBTime_ = std::chrono::steady_clock::now();
    }

    try {
        #ifdef DEBUG_LOG
        // 성능 모니터링
        auto callback_start = std::chrono::high_resolution_clock::now();

        if (processAndPublishRGBImage(frame)) {
            rgbFrameCount_++;
        }
        
        auto callback_end = std::chrono::high_resolution_clock::now();
        auto callback_time = std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start);

        if (rgbFrameCount_ % 100 == 0) {
            RCLCPP_WARN(this->get_logger(), 
                "🔍 RGB 콜백 #%lu - 처리시간: %ld μs (%.2f ms)", 
                static_cast<unsigned long>(rgbFrameCount_),
                callback_time.count(), 
                callback_time.count() / 1000.0);
        }
        #else
        processAndPublishRGBImage(frame);
        #endif
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Exception in RGB frame processing: %s", e.what());
    }
}

// ==================== 이미지 처리 함수들 (콜백 모드용) ====================
bool InuSensorNode::processAndPublishDepthImage(std::shared_ptr<const InuDev::CImageFrame> frame)
{
    rclcpp::Time timestamp = this->get_clock()->now();
    return processAndPublishDepthImageWithTimestamp(frame, timestamp);
}

bool InuSensorNode::processAndPublishRGBImage(std::shared_ptr<const InuDev::CImageFrame> frame)
{
    rclcpp::Time timestamp = this->get_clock()->now();
    return processAndPublishRGBImageWithTimestamp(frame, timestamp);
}

// ==================== Camera Info 생성 함수 ====================
sensor_msgs::msg::CameraInfo InuSensorNode::createCameraInfoFromYaml(
    const std::string& cameraName,
    const std::string& profile)
{
    sensor_msgs::msg::CameraInfo cameraInfo;

        if (cameraName == "depth") {
        // 문자열 -> Depth_Profile
        Depth_Profile depth_profile = Depth_Profile::Full;
        if (profile == "V_Binning" || profile == "v_binning" || profile == "VerticalBinning") {
            depth_profile = Depth_Profile::V_Binning;
        } else if (profile == "Binning" || profile == "binning") {
            depth_profile = Depth_Profile::Binning;
        }

        // 공통: rectified 가정 → 왜곡 0
        cameraInfo.distortion_model = "plumb_bob";
        cameraInfo.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        // 스테레오 Tx (baseline) 설정: 기존 값 유지(예: 48.018mm)
        constexpr double baseline_mm = 48.018;
        constexpr double baseline_m  = baseline_mm / 1000.0;

        // 각 모드별 YAML 값 적용
        double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
        int width = 0, height = 0;

        switch (depth_profile) {
            case Depth_Profile::Full:
            default: {
                // YAML: full.sensor_0
                // fc: [540.0, 510.0], out_size: [720, 1080]
                fx = 540.0; fy = 510.0;
                height = 720; width = 1080;
                // 중심점: out_size 중앙
                cx = width * 0.5;
                cy = height * 0.5;
                break;
            }
            case Depth_Profile::V_Binning: {
                // YAML: vertical_binning.sensor_0
                // fc: [540.0, 510.0], out_size: [480, 848]
                fx = 540.0; fy = 510.0;
                height = 480; width = 848;
                // 중심점: 중앙 (shift 항 없음)
                cx = width * 0.5;
                cy = height * 0.5;
                break;
            }
            case Depth_Profile::Binning: {
                // YAML: binning.sensor_0
                // fc: [270.0, 255.0], out_size: [360, 544]
                // rescale.sensor_0: hor_shift=-0.25, ver_shift=-0.25
                fx = 270.0; fy = 255.0;
                height = 360; width = 544;

                // 중심점: out_size 중앙 + (subpixel) 시프트 반영
                // - shift 단위를 '픽셀'로 가정 → cx, cy에 직접 더함
                const double hor_shift = -0.25;
                const double ver_shift = -0.25;
                cx = width * 0.5 + hor_shift;
                cy = height * 0.5 + ver_shift;
                break;
            }
        }

        // 해상도
        cameraInfo.width  = width;
        cameraInfo.height = height;

        // K
        cameraInfo.k[0] = fx;   cameraInfo.k[1] = 0.0; cameraInfo.k[2] = cx;
        cameraInfo.k[3] = 0.0;  cameraInfo.k[4] = fy;  cameraInfo.k[5] = cy;
        cameraInfo.k[6] = 0.0;  cameraInfo.k[7] = 0.0; cameraInfo.k[8] = 1.0;

        // P (Left 카메라): P = [K | [Tx= -fx*B; 0; 0]]
        cameraInfo.p[0] = fx;  cameraInfo.p[1] = 0.0; cameraInfo.p[2] = cx;                 cameraInfo.p[3] = -fx * baseline_m;
        cameraInfo.p[4] = 0.0; cameraInfo.p[5] = fy;  cameraInfo.p[6] = cy;                 cameraInfo.p[7] = 0.0;
        cameraInfo.p[8] = 0.0; cameraInfo.p[9] = 0.0; cameraInfo.p[10] = 1.0;               cameraInfo.p[11] = 0.0;

        // R (rectified 가정 → I)
        cameraInfo.r[0] = 1.0; cameraInfo.r[1] = 0.0; cameraInfo.r[2] = 0.0;
        cameraInfo.r[3] = 0.0; cameraInfo.r[4] = 1.0; cameraInfo.r[5] = 0.0;
        cameraInfo.r[6] = 0.0; cameraInfo.r[7] = 0.0; cameraInfo.r[8] = 1.0;

        // ROI/빈닝 기본값
        cameraInfo.binning_x = 0;
        cameraInfo.binning_y = 0;
        cameraInfo.roi.x_offset = 0;
        cameraInfo.roi.y_offset = 0;
        cameraInfo.roi.height = 0;
        cameraInfo.roi.width  = 0;
        cameraInfo.roi.do_rectify = false;

        return cameraInfo;
    }

    else if (cameraName == "rgb") {
        // 문자열 -> RGB_Profile 변환
        RGB_Profile rgb_profile = RGB_Profile::HD; // 기본값
        if (profile == "VGA") {
            rgb_profile = RGB_Profile::VGA;
        } else if (profile == "HVGA") {
            rgb_profile = RGB_Profile::HVGA;
        }
        // 원본 센서 Intrinsics (1280x720 기준)
        const double base_fx = 554.73663886177394;
        const double base_fy = 554.73663886177394;
        const double base_cx = 640.45615247006094;
        const double base_cy = 401.82175562754185;

        double fx, fy, cx, cy;
        int width, height;

        switch (rgb_profile) {
            case RGB_Profile::HD: { // 1280x720
                width = 1280; height = 720;
                fx = base_fx; fy = base_fy;
                cx = base_cx; cy = base_cy;
                break;
            }

            case RGB_Profile::VGA: { // 640x480 (센터 크롭 + 등비 스케일)
                width = 640; height = 480;

                // 센터 크롭 후 등비 스케일
                double crop_x = 160.0; // 좌우 크롭
                double s = 640.0 / 960.0; // 스케일 비율 2/3

                fx = base_fx * s;
                fy = base_fy * s;
                cx = (base_cx - crop_x) * s;
                cy = base_cy * s;
                break;
            }

            case RGB_Profile::HVGA: { // 640x360 (16:9 유지, 등비 축소)
                width = 640; height = 360;
                double s = 640.0 / 1280.0;

                fx = base_fx * s;
                fy = base_fy * s;
                cx = base_cx * s;
                cy = base_cy * s;
                break;
            }

            default: { // HD 기본값
                width = 1280; height = 720;
                fx = base_fx; fy = base_fy;
                cx = base_cx; cy = base_cy;
                break;
            }
        }

        // 해상도 적용
        cameraInfo.width  = width;
        cameraInfo.height = height;

        // Intrinsics K
        cameraInfo.k[0] = fx; cameraInfo.k[1] = 0.0; cameraInfo.k[2] = cx;
        cameraInfo.k[3] = 0.0; cameraInfo.k[4] = fy; cameraInfo.k[5] = cy;
        cameraInfo.k[6] = 0.0; cameraInfo.k[7] = 0.0; cameraInfo.k[8] = 1.0;

        // 왜곡 계수 그대로 사용
        cameraInfo.distortion_model = "plumb_bob";
        cameraInfo.d = {-0.044270290843990721,
                        0.057341275662470423,
                        0.00039232846148056172,
                        -0.00046911074253110151,
                        -0.033068106021877618};

        // Projection matrix P = K
        cameraInfo.p[0] = fx; cameraInfo.p[1] = 0.0; cameraInfo.p[2] = cx; cameraInfo.p[3] = 0.0;
        cameraInfo.p[4] = 0.0; cameraInfo.p[5] = fy; cameraInfo.p[6] = cy; cameraInfo.p[7] = 0.0;
        cameraInfo.p[8] = 0.0; cameraInfo.p[9] = 0.0; cameraInfo.p[10] = 1.0; cameraInfo.p[11] = 0.0;

        // Rectification matrix
        cameraInfo.r[0] = 1.0; cameraInfo.r[1] = 0.0; cameraInfo.r[2] = 0.0;
        cameraInfo.r[3] = 0.0; cameraInfo.r[4] = 1.0; cameraInfo.r[5] = 0.0;
        cameraInfo.r[6] = 0.0; cameraInfo.r[7] = 0.0; cameraInfo.r[8] = 1.0;

    } else {
        // 기본값 (640x480)
        cameraInfo.width  = 640;
        cameraInfo.height = 480;
        cameraInfo.k[0] = 400.0; cameraInfo.k[1] = 0.0; cameraInfo.k[2] = 320.0;
        cameraInfo.k[3] = 0.0; cameraInfo.k[4] = 400.0; cameraInfo.k[5] = 240.0;
        cameraInfo.k[6] = 0.0; cameraInfo.k[7] = 0.0; cameraInfo.k[8] = 1.0;

        cameraInfo.distortion_model = "plumb_bob";
        cameraInfo.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        cameraInfo.p[0] = cameraInfo.k[0]; cameraInfo.p[1] = 0.0; cameraInfo.p[2] = cameraInfo.k[2]; cameraInfo.p[3] = 0.0;
        cameraInfo.p[4] = 0.0; cameraInfo.p[5] = cameraInfo.k[4]; cameraInfo.p[6] = cameraInfo.k[5]; cameraInfo.p[7] = 0.0;
        cameraInfo.p[8] = 0.0; cameraInfo.p[9] = 0.0; cameraInfo.p[10] = 1.0; cameraInfo.p[11] = 0.0;

        cameraInfo.r[0] = 1.0; cameraInfo.r[1] = 0.0; cameraInfo.r[2] = 0.0;
        cameraInfo.r[3] = 0.0; cameraInfo.r[4] = 1.0; cameraInfo.r[5] = 0.0;
        cameraInfo.r[6] = 0.0; cameraInfo.r[7] = 0.0; cameraInfo.r[8] = 1.0;
    }

    return cameraInfo;
}


// ==================== 기타 헬퍼 함수들 ====================

void InuSensorNode::publishIMUData(const sensor_msgs::msg::Imu& imuMsg)
{
    try
    {
        if (imuPub_) {
            imuPub_->publish(imuMsg);
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during IMU publish: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception during IMU publish");
    }
}

void InuSensorNode::publishCombinedIMUData()
{
    //std::lock_guard<std::mutex> lock(currentImuData_.mutex);
    
    if (!currentImuData_.isComplete()) {
        return;
    }
    
    // Create IMU message with synchronized data
    sensor_msgs::msg::Imu imuMsg;
    imuMsg.header.stamp = currentImuData_.timestamp;
    imuMsg.header.frame_id = imuFrameId_;
    
    // Set Cam <-> IMU TF
    // real : imu           real : imu
    // acc x : acc z        gyro x : gyro -z
    // acc y : acc -x       gyro y : gyro -x
    // acc z : acc y        gyro z : gyro -y
    //
    //
    // Set linear acceleration
    /* 
    imuMsg.linear_acceleration.x = currentImuData_.accelerometer[2];
    imuMsg.linear_acceleration.y = currentImuData_.accelerometer[0]; // -
    imuMsg.linear_acceleration.z = +currentImuData_.accelerometer[1]; // +
    
    // Set angular velocity
    imuMsg.angular_velocity.x = -currentImuData_.gyroscope[2];
    imuMsg.angular_velocity.y = -currentImuData_.gyroscope[0]; // +
    imuMsg.angular_velocity.z = currentImuData_.gyroscope[1]; // -
    */
    
    //Not Set Cam <-> IMU TF
    // Set linear acceleration
    imuMsg.linear_acceleration.x = currentImuData_.accelerometer[0];
    imuMsg.linear_acceleration.y = currentImuData_.accelerometer[1];
    imuMsg.linear_acceleration.z = currentImuData_.accelerometer[2];
    
    // Set angular velocity
    imuMsg.angular_velocity.x = currentImuData_.gyroscope[0];
    imuMsg.angular_velocity.y = currentImuData_.gyroscope[1];
    imuMsg.angular_velocity.z = currentImuData_.gyroscope[2];
    

    // Set covariance matrices (could be loaded from calibration YAML)
    // Linear acceleration covariance
    for (int i = 0; i < 9; ++i) {
        imuMsg.linear_acceleration_covariance[i] = 0.0;
        imuMsg.angular_velocity_covariance[i] = 0.0;
    }
    imuMsg.linear_acceleration_covariance[0] = 0.001; // x variance
    imuMsg.linear_acceleration_covariance[4] = 0.001; // y variance  
    imuMsg.linear_acceleration_covariance[8] = 0.001; // z variance
    
    // Angular velocity covariance
    imuMsg.angular_velocity_covariance[0] = 0.0001; // x variance
    imuMsg.angular_velocity_covariance[4] = 0.0001; // y variance
    imuMsg.angular_velocity_covariance[8] = 0.0001; // z variance
    
    // No orientation data available from this sensor
    imuMsg.orientation_covariance[0] = -1.0; // Mark as unavailable
    
    // Publish synchronized IMU data
    publishIMUData(imuMsg);
    
    // Reset for next sync
    currentImuData_.reset();
    
    //imuFrameCount_++;
    
    // Debug info
    #ifdef DEBUG_MODE
    if (imuFrameCount_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Published %lu synchronized IMU frames", imuFrameCount_);
    }
    #endif
}

// IMU callbacks remain the same for synchronization
void InuSensorNode::onIMUAccFrame(std::shared_ptr<InuDev::CImuStream> stream,
                                 std::shared_ptr<const InuDev::CImuFrame> frame,
                                 InuDev::CInuError retCode)
{
    if (isRecovering_) return;
    if (retCode != InuDev::eOK) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Error receiving IMU ACC frame: %s", std::string(retCode).c_str());
        return;
    }

    if (!frame->Valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Invalid IMU ACC frame %lu", frame->FrameIndex);
        return;
    }

    // Extract accelerometer data
    InuDev::CPoint3D acc;
    bool ret = frame->GetSensorData(InuDev::EImuType::eAccelerometer, acc);
    if (!ret) {
        return;
    }

    // Store accelerometer data for synchronization
    {
        //std::lock_guard<std::mutex> lock(currentImuData_.mutex);
        
        // If we don't have gyro data or it's too old, update timestamp
        auto currentTime = this->get_clock()->now();
        if (!currentImuData_.hasGyroData || 
            (currentTime - currentImuData_.timestamp).nanoseconds() > (imuSyncTimeoutMs_ * 1e6)) {
            currentImuData_.timestamp = currentTime;
        }
        
        currentImuData_.accelerometer = acc;
        currentImuData_.hasAccData = true;
        
        // Try to publish synchronized data
        if (currentImuData_.isComplete()) {
            // Check if data is not too old
            auto timeDiff = (currentTime - currentImuData_.timestamp).nanoseconds() / 1e6; // Convert to ms
            if (timeDiff <= imuSyncTimeoutMs_) {
                publishCombinedIMUData();
            } else {
                // Data too old, reset and start fresh
                currentImuData_.reset();
                currentImuData_.timestamp = currentTime;
                currentImuData_.accelerometer = acc;
                currentImuData_.hasAccData = true;
            }
        }
    }
}

void InuSensorNode::onIMUGyroFrame(std::shared_ptr<InuDev::CImuStream> stream,
                                  std::shared_ptr<const InuDev::CImuFrame> frame,
                                  InuDev::CInuError retCode)
{
    if (isRecovering_) return;
    if (retCode != InuDev::eOK) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Error receiving IMU GYRO frame: %s", std::string(retCode).c_str());
        return;
    }

    if (!frame->Valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Invalid IMU GYRO frame %lu", frame->FrameIndex);
        return;
    }

    // Extract gyroscope data
    InuDev::CPoint3D gyro;
    bool ret = frame->GetSensorData(InuDev::EImuType::eGyroscope, gyro);
    if (!ret) {
        return;
    }

    // Store gyroscope data for synchronization
    {
        //std::lock_guard<std::mutex> lock(currentImuData_.mutex);
        
        // If we don't have acc data or it's too old, update timestamp
        auto currentTime = this->get_clock()->now();
        if (!currentImuData_.hasAccData || 
            (currentTime - currentImuData_.timestamp).nanoseconds() > (imuSyncTimeoutMs_ * 1e6)) {
            currentImuData_.timestamp = currentTime;
        }
        
        currentImuData_.gyroscope = gyro;
        currentImuData_.hasGyroData = true;
        
        // Try to publish synchronized data
        if (currentImuData_.isComplete()) {
            // Check if data is not too old
            auto timeDiff = (currentTime - currentImuData_.timestamp).nanoseconds() / 1e6; // Convert to ms
            if (timeDiff <= imuSyncTimeoutMs_) {
                publishCombinedIMUData();
            } else {
                // Data too old, reset and start fresh
                currentImuData_.reset();
                currentImuData_.timestamp = currentTime;
                currentImuData_.gyroscope = gyro;
                currentImuData_.hasGyroData = true;
            }
        }
    }
}


void InuSensorNode::cleanup()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up InuSensor Node");

    // Recovery 스레드 종료 신호 후 대기
    shutdownRequested_ = true;
    if (recoveryThread_.joinable()) {
        recoveryThread_.join();
    }

    // Watchdog 타이머 취소
    if (watchdogTimer_) {
        watchdogTimer_->cancel();
        watchdogTimer_.reset();
    }

    // 타이머 모드 비활성화
    disableTimerMode();

    // 콜백 모드 비활성화
    disableCallbackMode();
    
    // TF broadcaster 정리
    staticTfBroadcaster_.reset();
    
    // Sensor manager 정리
    if (sensorManager_) {
        sensorManager_->shutdown();
        sensorManager_.reset();
    }
    
    RCLCPP_INFO(this->get_logger(), "InuSensor Node cleanup completed");
}

// ==================== Recovery 함수들 ====================

void InuSensorNode::watchdogCallback()
{
    if (isRecovering_) return;
    if (!sensorManager_) return;

    auto now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point depthTime, rgbTime;

    {
        std::lock_guard<std::mutex> lock(performanceMutex_);
        depthTime = lastDepthTime_;
        rgbTime   = lastRGBTime_;
    }

    auto timeoutDur = std::chrono::duration<double>(watchdogTimeoutS_);

    if (publishDepth_ && (now - depthTime) > timeoutDur) {
        double elapsed = std::chrono::duration<double>(now - depthTime).count();
        RCLCPP_WARN(this->get_logger(),
                    "[Watchdog] Depth 스트림 침묵 %.1f초 감지 (임계값: %.1f초). Recovery 시작.",
                    elapsed, watchdogTimeoutS_);
        triggerRecovery();
        return;  // Depth에서 트리거됐으면 RGB 체크 불필요
    }

    if (publishRGB_ && (now - rgbTime) > timeoutDur) {
        double elapsed = std::chrono::duration<double>(now - rgbTime).count();
        RCLCPP_WARN(this->get_logger(),
                    "[Watchdog] RGB 스트림 침묵 %.1f초 감지 (임계값: %.1f초). Recovery 시작.",
                    elapsed, watchdogTimeoutS_);
        triggerRecovery();
    }
}

bool InuSensorNode::isRecoveryTriggerError(InuDev::CInuError retCode)
{
    // eServiceProcessFailure=11, eSensorDetectionFailure=12 (InuError.h 기준)
    // SDK 버전에 따라 enum 상수명이 없을 수 있으므로 정수값으로 직접 비교
    int code = static_cast<int>(retCode);
    return (code == 11 || code == 12);
}

void InuSensorNode::publishStatus(const std::string& status)
{
    if (!statusPub_) return;
    auto msg = std_msgs::msg::String();
    msg.data = status;
    statusPub_->publish(msg);
}

void InuSensorNode::triggerRecovery()
{
    bool expected = false;
    if (!isRecovering_.compare_exchange_strong(expected, true)) {
        return;  // 이미 Recovery 중
    }

    consecutiveTimeoutCount_ = 0;
    RCLCPP_WARN(this->get_logger(), "[Recovery] IPC 단절 감지. Recovery 절차 시작...");
    publishStatus("RECOVERING");

    // 타이머 중단 (Recovery 중 데이터 수신 방지)
    if (useTimerMode_ && syncTimer_) {
        syncTimer_->cancel();
    }

    // 이전 Recovery 스레드 정리
    if (recoveryThread_.joinable()) {
        recoveryThread_.join();
    }
    recoveryThread_ = std::thread(&InuSensorNode::performRecovery, this);
}

void InuSensorNode::performRecovery()
{
    const int maxAttempts = maxRecoveryAttempts_;
    int delayMs = 2000;

    for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
        if (shutdownRequested_) {
            RCLCPP_INFO(this->get_logger(), "[Recovery] 종료 요청으로 Recovery 중단.");
            break;
        }

        RCLCPP_WARN(this->get_logger(), "[Recovery] 시도 %d/%d ...", attempt, maxAttempts);

        // 1단계: 기존 센서 완전 정리
        if (sensorManager_) {
            sensorManager_->shutdown();
            sensorManager_.reset();
        }

        // 2단계: 센서 객체 재생성
        try {
            sensorManager_ = std::make_unique<InuSensorManager>();
            setDepthProfile(depth_res_profile_str);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "[Recovery] 센서 객체 생성 실패: %s", e.what());
            sensorManager_.reset();
        }

        // 3단계: Init 시도
        if (!sensorManager_ || !sensorManager_->initializeSensor()) {
            RCLCPP_WARN(this->get_logger(), "[Recovery] Init 실패. %dms 후 재시도...", delayMs);
            // 인터럽트 가능한 sleep (100ms 단위)
            for (int i = 0; i < delayMs / 100 && !shutdownRequested_; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            delayMs = std::min(delayMs * 2, 30000);
            continue;
        }

        // 4단계: 스트림 재초기화
        if (!reinitializeStreams()) {
            RCLCPP_WARN(this->get_logger(), "[Recovery] 스트림 재초기화 실패. %dms 후 재시도...", delayMs);
            for (int i = 0; i < delayMs / 100 && !shutdownRequested_; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            delayMs = std::min(delayMs * 2, 30000);
            continue;
        }

        // 5단계: 콜백/타이머 재등록
        restartStreamingMode();

        RCLCPP_INFO(this->get_logger(), "[Recovery] 성공! (%d회 시도)", attempt);
        publishStatus("ACTIVE");
        isRecovering_ = false;
        return;
    }

    RCLCPP_ERROR(this->get_logger(),
                 "[Recovery] %d회 시도 후 복구 실패. 에러 상태 유지.", maxAttempts);
    publishStatus("ERROR");
    isRecovering_ = false;
}

bool InuSensorNode::reinitializeStreams()
{
    if (!sensorManager_) return false;

    if (publishDepth_ && !sensorManager_->initializeDepth()) {
        RCLCPP_WARN(this->get_logger(), "[Recovery] Depth 스트림 초기화 실패");
        return false;
    }
    if (publishRGB_ && !sensorManager_->initializeRGB()) {
        RCLCPP_WARN(this->get_logger(), "[Recovery] RGB 스트림 초기화 실패");
        return false;
    }
    if (publishIMU_ && !sensorManager_->initializeIMU()) {
        RCLCPP_WARN(this->get_logger(), "[Recovery] IMU 스트림 초기화 실패");
        return false;
    }
    if (publishDepth_ && !sensorManager_->startDepth()) {
        RCLCPP_WARN(this->get_logger(), "[Recovery] Depth 스트림 시작 실패");
        return false;
    }
    if (publishRGB_ && !sensorManager_->startRGB()) {
        RCLCPP_WARN(this->get_logger(), "[Recovery] RGB 스트림 시작 실패");
        return false;
    }
    if (publishIMU_ && !sensorManager_->startIMU()) {
        RCLCPP_WARN(this->get_logger(), "[Recovery] IMU 스트림 시작 실패");
        return false;
    }
    return true;
}

void InuSensorNode::restartStreamingMode()
{
    // SDK 콜백 재등록 (sensorManager_ 교체 후 항상 필요)
    if (publishDepth_ && sensorManager_) {
        sensorManager_->registerDepthCallback(
            std::bind(&InuSensorNode::onDepthFrameReceived, this,
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }
    if (publishRGB_ && sensorManager_) {
        sensorManager_->registerRGBCallback(
            std::bind(&InuSensorNode::onRGBFrameReceived, this,
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }
    if (publishIMU_ && sensorManager_) {
        sensorManager_->registerIMUAccCallback(
            std::bind(&InuSensorNode::onIMUAccFrame, this,
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        sensorManager_->registerIMUGyroCallback(
            std::bind(&InuSensorNode::onIMUGyroFrame, this,
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    if (useTimerMode_) {
        timerModeActive_ = true;
        if (syncTimer_) syncTimer_->reset();
        RCLCPP_INFO(this->get_logger(), "[Recovery] Timer mode 재시작 (%.1f Hz)", timerFrequency_);
    } else {
        callbackModeActive_ = true;
        RCLCPP_INFO(this->get_logger(), "[Recovery] Callback mode 재시작");
    }

    // Watchdog 시각 리셋 (Recovery 완료 직후 오탐 방지)
    {
        std::lock_guard<std::mutex> lock(performanceMutex_);
        lastDepthTime_ = std::chrono::steady_clock::now();
        lastRGBTime_   = std::chrono::steady_clock::now();
    }
    RCLCPP_INFO(this->get_logger(), "[Watchdog] 스트림 시각 리셋 완료.");
}

} // namespace inusensor_ros2_driver