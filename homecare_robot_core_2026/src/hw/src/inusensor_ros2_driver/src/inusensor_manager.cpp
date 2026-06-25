#include "inusensor_ros2_driver/inusensor_manager.hpp"
#include <iostream>

namespace inusensor_ros2_driver
{

InuSensorManager::InuSensorManager()
    : depthChannelId_(3)  // M4.51S Depth Stream channel
    , rgbChannelId_(8)    // M4.51S RGB Stream channel
    //, regBaseChannelId_(4) // M4.51S Depth Reg Stream channel -> not work
    , sensorInitialized_(false)
    , depthInitialized_(false)
    , rgbInitialized_(false)
    , rgbregInitialized_(false)
    , imuInitialized_(false)
    , sensorStarted_(false)
{
    std::cout << "InuSensorManager constructor started" << std::endl;
    
    try {
        std::cout << "Creating CInuSensor object..." << std::endl;
        // Create CInuSensor object (following original API)
        inuSensor_ = InuDev::CInuSensor::Create();
        
        if (inuSensor_) {
            std::cout << "CInuSensor object created successfully" << std::endl;
        } else {
            std::cout << "Failed to create CInuSensor object" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "Exception in InuSensorManager constructor: " << e.what() << std::endl;
        inuSensor_.reset();
    } catch (...) {
        std::cout << "Unknown exception in InuSensorManager constructor" << std::endl;
        inuSensor_.reset();
    }
    
    std::cout << "InuSensorManager constructor completed" << std::endl;
}

InuSensorManager::~InuSensorManager()
{
    shutdown();
}

bool InuSensorManager::initializeSensor()
{
    std::cout << "InuSensorManager::initializeSensor() called" << std::endl;
    
    if (sensorInitialized_) {
        std::cout << "Sensor already initialized" << std::endl;
        return true;
    }

    try {
        std::cout << "Creating InuDev::CInuSensor..." << std::endl;
        // Create CInuSensor object (following original API)
        if (!inuSensor_) {
            std::cout << "InuSensor object is null, this should not happen" << std::endl;
            return false;
        }
        
        std::cout << "Calling inuSensor_->Init()..." << std::endl;
        // Initialize the sensor (same as original API)
        InuDev::CInuError retCode = inuSensor_->Init();
        if (retCode != InuDev::eOK) {
            std::cout << "Failed to connect to Inuitive Sensor. Error: " << std::hex 
                      << int(retCode) << " - " << std::string(retCode) << std::endl;
            return false;
        }
        std::cout << "Connected to Sensor" << std::endl;

        std::cout << "Calling inuSensor_->Start()..." << std::endl;
        // Start acquiring frames

        if(!cparamsInitialized_){
            std::cout << "Depth Channel setting : Default" << std::endl;
            setDepthResolutionDefault();
        }

        retCode = inuSensor_->Start(oChannelsSize, ioParams);
        //retCode = inuSensor_->Start();

        if (retCode != InuDev::eOK) {
            std::cout << "Failed to start Inuitive Sensor." << std::endl;
            return false;
        }
        std::cout << "Sensor is started" << std::endl;

        sensorInitialized_ = true;
        sensorStarted_ = true;
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "Exception in initializeSensor: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cout << "Unknown exception in initializeSensor" << std::endl;
        return false;
    }
}

// Depth resolution setting
void InuSensorManager::setDepthResolutionDefault()
{
    std::cout << "Depth Resolution Set: Default" << std::endl;
    CParams.SensorRes = InuDev::ESensorResolution::eDefaultResolution;
    ioParams[depthChannelId_] = CParams;
    cparamsInitialized_ = true;
}

void InuSensorManager::setDepthResolutionFull()
{
    std::cout << "Depth Resolution Set: Full" << std::endl;
    CParams.SensorRes = InuDev::ESensorResolution::eFull;
    ioParams[depthChannelId_] = CParams;
    cparamsInitialized_ = true;
}

void InuSensorManager::setDepthResolutionVerticalBinning()
{
    std::cout << "Depth Resolution Set: Vertical Binning" << std::endl;
    CParams.SensorRes = InuDev::ESensorResolution::eVerticalBinning;
    ioParams[depthChannelId_] = CParams;
    cparamsInitialized_ = true;
}

void InuSensorManager::setDepthResolutionBinning()
{
    std::cout << "Depth Resolution Set: Binning" << std::endl;
    CParams.SensorRes = InuDev::ESensorResolution::eBinning;
    ioParams[depthChannelId_] = CParams;
    cparamsInitialized_ = true;
}

// Depth post-processing configuration methods
void InuSensorManager::setDepthPostProcessingFlags(InuDev::CDepthProperties::EPostProcessing flags)
{
    depthPostProcessingFlags_ = flags;
    std::cout << "Depth post-processing flags set to: " << static_cast<int>(flags) << std::endl;
}

InuDev::CDepthProperties::EPostProcessing InuSensorManager::getDepthPostProcessingFlags() const
{
    return depthPostProcessingFlags_;
}

void InuSensorManager::enableAllDepthPostProcessing()
{
    depthPostProcessingFlags_ = static_cast<InuDev::CDepthProperties::EPostProcessing>(
        InuDev::CDepthProperties::EPostProcessing::eBlob |
        InuDev::CDepthProperties::EPostProcessing::eOutlierRemove 
    );
    std::cout << "All depth post-processing enabled" << std::endl;
}

bool InuSensorManager::initializeDepth()
{
    if (!sensorInitialized_) {
        std::cout << "Sensor must be initialized before depth stream" << std::endl;
        return false;
    }

    if (depthInitialized_) {
        return true;
    }

    // Generate depth stream object
    depthStream_ = inuSensor_->CreateDepthStream(depthChannelId_);
    if (depthStream_ == nullptr) {
        std::cout << "Unexpected error, failed to get Depth Stream" << std::endl;
        return false;
    }

    enableAllDepthPostProcessing();

    // Configure Depth parameters
    InuDev::CInuError retCode = depthStream_->Init(InuDev::CDepthStream::EOutputFormat::eDefault, depthPostProcessingFlags_);
    if (retCode != InuDev::eOK) {
        std::cout << "Depth initiation error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "Depth Stream is initialized" << std::endl;

    depthInitialized_ = true;
    return true;
}

bool InuSensorManager::initializeRGB()
{
    if (!sensorInitialized_) {
        std::cout << "Sensor must be initialized before RGB stream" << std::endl;
        return false;
    }

    if (rgbInitialized_) {
        return true;
    }

    // Generate RGB stream object
    rgbStream_ = inuSensor_->CreateImageStream(rgbChannelId_);
    if (rgbStream_ == nullptr) {
        std::cout << "Unexpected error, failed to get RGB Stream" << std::endl;
        return false;
    }

    // Configure RGB parameters (Default Mode = RGBA)
    InuDev::CInuError retCode = rgbStream_->Init(InuDev::CImageStream::EOutputFormat::eDefault);
    if (retCode != InuDev::eOK) {
        std::cout << "RGB initiation error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "RGB Stream is initialized" << std::endl;

    rgbInitialized_ = true;
    return true;
}

bool InuSensorManager::initializeRGBRegister()
{
    if (!sensorInitialized_) {
        std::cout << "Sensor must be initialized before RGB stream" << std::endl;
        return false;
    }

    if (rgbInitialized_) {
        return true;
    }

    // Generate RGB stream object
    rgbRegStream_ = inuSensor_->CreateImageRegisteredStream(rgbChannelId_ , depthChannelId_);
    if (rgbRegStream_ == nullptr) {
        std::cout << "Unexpected error, failed to get Frame Register Stream" << std::endl;
        return false;
    }


    // Configure RGB parameters (Default Mode = RGBA)
    InuDev::CInuError retCode = rgbRegStream_->Init(InuDev::CImageStream::EOutputFormat::eDefault, InuDev::CImageStream::eNone, depthPostProcessingFlags_);

    if (retCode != InuDev::eOK) {
        std::cout << "RGB REG initiation error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "RGB REG Stream is initialized" << std::endl;

    rgbregInitialized_ = true;
    return true;
}

bool InuSensorManager::initializeIMU()
{
    if (!sensorInitialized_) {
        std::cout << "Sensor must be initialized before IMU streams" << std::endl;
        return false;
    }

    if (imuInitialized_) {
        return true;
    }

    // Generate IMU stream objects
    imuAccStream_ = inuSensor_->CreateImuStream();
    imuGyroStream_ = inuSensor_->CreateImuStream();

    if (imuAccStream_ == nullptr || imuGyroStream_ == nullptr) {
        std::cout << "Unexpected error, failed to get IMU Streams" << std::endl;
        return false;
    }

    // Initialize IMU streams
    InuDev::CInuError retCode = imuAccStream_->Init();
    if (retCode != InuDev::eOK) {
        std::cout << "IMU ACC initiation error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }

    retCode = imuGyroStream_->Init();
    if (retCode != InuDev::eOK) {
        std::cout << "IMU GYRO initiation error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "IMU Streams are initialized" << std::endl;

    imuInitialized_ = true;
    return true;
}

bool InuSensorManager::startDepth()
{
    if (!depthInitialized_) {
        std::cout << "Depth stream must be initialized before starting" << std::endl;
        return false;
    }

    InuDev::CInuError retCode = depthStream_->Start();
    if (retCode != InuDev::eOK) {
        std::cout << "Depth Start error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "Depth frames acquisition started" << std::endl;
    return true;
}

bool InuSensorManager::startRGBRegister()
{
    if (!rgbregInitialized_) {
        std::cout << "Frame Register stream must be initialized before starting" << std::endl;
        return false;
    }

    InuDev::CInuError retCode = rgbRegStream_->Start();
    if (retCode != InuDev::eOK) {
        std::cout << "Frame Register Start error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "Frame Register frames acquisition started" << std::endl;
    return true;
}

bool InuSensorManager::startRGB()
{
    if (!rgbInitialized_) {
        std::cout << "RGB stream must be initialized before starting" << std::endl;
        return false;
    }

    InuDev::CInuError retCode = rgbStream_->Start();
    if (retCode != InuDev::eOK) {
        std::cout << "RGB Start error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "RGB frames acquisition started" << std::endl;
    return true;
}

bool InuSensorManager::startIMU()
{
    if (!imuInitialized_) {
        std::cout << "IMU streams must be initialized before starting" << std::endl;
        return false;
    }

    InuDev::CInuError retCode = imuAccStream_->Start();
    if (retCode != InuDev::eOK) {
        std::cout << "IMU ACC Start error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }

    retCode = imuGyroStream_->Start();
    if (retCode != InuDev::eOK) {
        std::cout << "IMU GYRO Start error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "IMU frames acquisition started" << std::endl;
    return true;
}

bool InuSensorManager::stopDepth()
{
    if (!depthStream_) return true;

    InuDev::CInuError retCode = depthStream_->Stop();
    if (retCode != InuDev::eOK) {
        std::cout << "Depth Stop error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "Depth frames acquisition stopped" << std::endl;
    return true;
}

bool InuSensorManager::stopRGB()
{
    if (!rgbStream_) return true;

    InuDev::CInuError retCode = rgbStream_->Stop();
    if (retCode != InuDev::eOK) {
        std::cout << "RGB Stop error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "RGB frames acquisition stopped" << std::endl;
    return true;
}

bool InuSensorManager::stopRGBRegister()
{
    if (!rgbRegStream_) return true;

    InuDev::CInuError retCode = rgbRegStream_->Stop();
    if (retCode != InuDev::eOK) {
        std::cout << "Frame Register Stop error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "Frame Register frames acquisition stopped" << std::endl;
    return true;
}

bool InuSensorManager::stopIMU()
{
    bool success = true;
    
    if (imuAccStream_) {
        InuDev::CInuError retCode = imuAccStream_->Stop();
        if (retCode != InuDev::eOK) {
            std::cout << "IMU ACC Stop error: " << std::hex << int(retCode) 
                      << " - " << std::string(retCode) << std::endl;
            success = false;
        }
    }

    if (imuGyroStream_) {
        InuDev::CInuError retCode = imuGyroStream_->Stop();
        if (retCode != InuDev::eOK) {
            std::cout << "IMU GYRO Stop error: " << std::hex << int(retCode) 
                      << " - " << std::string(retCode) << std::endl;
            success = false;
        }
    }

    if (success) {
        std::cout << "IMU frames acquisition stopped" << std::endl;
    }
    return success;
}

bool InuSensorManager::registerDepthCallback(const DepthCallback& callback)
{
    if (!depthStream_) {
        std::cout << "Depth stream not initialized" << std::endl;
        return false;
    }

    depthCallback_ = callback;
    
    InuDev::CInuError retCode = depthStream_->Register(
        std::bind(&InuSensorManager::depthFrameCallback, this, 
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    if (retCode != InuDev::eOK) {
        std::cout << "Depth register error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    return true;
}

bool InuSensorManager::registerRGBCallback(const RGBCallback& callback)
{
    if(!rgb_to_depthRegistration_){
        if (!rgbStream_) {
            std::cout << "RGB stream not initialized" << std::endl;
            return false;
        }
    
        rgbCallback_ = callback;
        
        InuDev::CInuError retCode = rgbStream_->Register(
            std::bind(&InuSensorManager::rgbFrameCallback, this, 
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        if (retCode != InuDev::eOK) {
            std::cout << "RGB register error: " << std::hex << int(retCode) 
                    << " - " << std::string(retCode) << std::endl;
            return false;
        }
    }
    ///////////////RGB to Depth Register
    else{
        if (!rgbRegStream_) {
            std::cout << "RGB REG stream not initialized" << std::endl;
            return false;
        }
    
        rgbCallback_ = callback;
        
        InuDev::CInuError retCode = rgbRegStream_->Register(
            std::bind(&InuSensorManager::rgbFrameCallback, this, 
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        if (retCode != InuDev::eOK) {
            std::cout << "Frame Register register error: " << std::hex << int(retCode) 
                    << " - " << std::string(retCode) << std::endl;
            return false;
        }
    }
    return true;
}

bool InuSensorManager::registerIMUAccCallback(const IMUCallback& callback)
{
    if (!imuAccStream_) {
        std::cout << "IMU ACC stream not initialized" << std::endl;
        return false;
    }

    imuAccCallback_ = callback;
    
    InuDev::CInuError retCode = imuAccStream_->Register(
        std::bind(&InuSensorManager::imuAccFrameCallback, this, 
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    if (retCode != InuDev::eOK) {
        std::cout << "IMU ACC register error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    return true;
}

bool InuSensorManager::registerIMUGyroCallback(const IMUCallback& callback)
{
    if (!imuGyroStream_) {
        std::cout << "IMU GYRO stream not initialized" << std::endl;
        return false;
    }

    imuGyroCallback_ = callback;
    
    InuDev::CInuError retCode = imuGyroStream_->Register(
        std::bind(&InuSensorManager::imuGyroFrameCallback, this, 
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    if (retCode != InuDev::eOK) {
        std::cout << "IMU GYRO register error: " << std::hex << int(retCode) 
                  << " - " << std::string(retCode) << std::endl;
        return false;
    }
    return true;
}

bool InuSensorManager::getCalibrationData(InuDev::CCalibrationData& calibData, uint32_t channelId)
{
    if (!sensorInitialized_) {
        std::cout << "Sensor not initialized" << std::endl;
        return false;
    }

    InuDev::CInuError retCode = inuSensor_->GetCalibrationData(calibData, channelId);
    if (retCode != InuDev::eOK) {
        std::cout << "Failed to get calibration data. Error: " << std::hex 
                  << int(retCode) << " - " << std::string(retCode) << std::endl;
        return false;
    }
    return true;
}

void InuSensorManager::printOpticalData(uint32_t channelId)
{
    InuDev::CCalibrationData calibData;
    if (!getCalibrationData(calibData, channelId)) {
        return;
    }

    std::cout << "--------------------------------------------" << std::endl;
    
    // Print available sensors
    for (const auto& sensorPair : calibData.Sensors) {
        std::cout << "Sensor " << sensorPair.first << " type: " << sensorPair.second.Description << std::endl;
    }
    std::cout << "--------------------------------------------" << std::endl;
    
    // Print details for sensor 0 and 1 if they exist
    auto sensor0It = calibData.Sensors.find(0);
    auto sensor1It = calibData.Sensors.find(1);
    
    if (sensor0It != calibData.Sensors.end()) {
        const auto& sensor0 = sensor0It->second;
        std::cout << "Real Opt center X:" << sensor0.RealCamera.Intrinsic.OpticalCenter[0] << std::endl;
        std::cout << "Real Opt center Y:" << sensor0.RealCamera.Intrinsic.OpticalCenter[1] << std::endl;
        std::cout << "Real Camera Extrinsic Valid = " << sensor0.RealCamera.Extrinsic.Valid << std::endl;
        std::cout << "Real Camera Intrinsic Valid = " << sensor0.RealCamera.Intrinsic.Valid << std::endl;
        std::cout << "Real Camera X resolution:" << sensor0.RealCamera.Resolution[0] << std::endl;
        std::cout << "Real Camera Y resolution:" << sensor0.RealCamera.Resolution[1] << std::endl;
    }
    
    if (sensor1It != calibData.Sensors.end()) {
        const auto& sensor1 = sensor1It->second;
        std::cout << "Rectified Opt center X:" << sensor1.VirtualCamera.Intrinsic.OpticalCenter[0] << std::endl;
        std::cout << "Rectified Opt center Y:" << sensor1.VirtualCamera.Intrinsic.OpticalCenter[1] << std::endl;
    }
}

bool InuSensorManager::isStreamActive(std::shared_ptr<InuDev::CDepthStream> stream) const
{
    if (!stream) return false;
    
    try {
        // 스트림이 활성 상태인지 확인하는 방법
        // 실제 SDK에 상태 확인 함수가 있다면 해당 함수 사용
        return true; // 기본적으로 스트림이 존재하면 활성으로 간주
    } catch (...) {
        return false;
    }
}

bool InuSensorManager::isStreamActive(std::shared_ptr<InuDev::CImageStream> stream) const
{
    if (!stream) return false;
    
    try {
        return true; // 기본적으로 스트림이 존재하면 활성으로 간주
    } catch (...) {
        return false;
    }
}

bool InuSensorManager::isStreamActive(std::shared_ptr<InuDev::CImuStream> stream) const
{
    if (!stream) return false;
    
    try {
        return true; // 기본적으로 스트림이 존재하면 활성으로 간주
    } catch (...) {
        return false;
    }
}

bool InuSensorManager::waitForStreamShutdown(const std::string& streamName, int maxWaitMs)
{
    std::cout << "Waiting for " << streamName << " to shutdown..." << std::endl;
    
    int waitedMs = 0;
    const int checkIntervalMs = 10;
    
    while (waitedMs < maxWaitMs) {
        // 여기서 실제 스트림 상태를 확인할 수 있다면 더 정확함
        // 지금은 단순히 시간 기반 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(checkIntervalMs));
        waitedMs += checkIntervalMs;
        
        // 중간 진행 상황 표시
        if (waitedMs % 100 == 0) {
            std::cout << "  " << streamName << " shutdown progress: " 
                      << (waitedMs * 100 / maxWaitMs) << "%" << std::endl;
        }
    }
    
    std::cout << streamName << " shutdown wait completed" << std::endl;
    return true;
}

void InuSensorManager::forceTerminateAllStreams()
{
    std::cout << "Forcing termination of all remaining streams..." << std::endl;
    
    // Force terminate in reverse order of initialization
    if (imuGyroStream_) {
        try {
            imuGyroStream_->Terminate();
        } catch (...) {
            std::cout << "Exception during forced IMU gyro termination" << std::endl;
        }
        imuGyroStream_.reset();
    }
    
    if (imuAccStream_) {
        try {
            imuAccStream_->Terminate();
        } catch (...) {
            std::cout << "Exception during forced IMU acc termination" << std::endl;
        }
        imuAccStream_.reset();
    }
    
    if (rgbRegStream_) {
        try {
            rgbRegStream_->Terminate();
        } catch (...) {
            std::cout << "Exception during forced RGB registered termination" << std::endl;
        }
        rgbRegStream_.reset();
    }
    
    if (rgbStream_) {
        try {
            rgbStream_->Terminate();
        } catch (...) {
            std::cout << "Exception during forced RGB termination" << std::endl;
        }
        rgbStream_.reset();
    }
    
    if (depthStream_) {
        try {
            depthStream_->Terminate();
        } catch (...) {
            std::cout << "Exception during forced depth termination" << std::endl;
        }
        depthStream_.reset();
    }
    
    std::cout << "Forced termination completed" << std::endl;
}

// 더욱 강화된 shutdown 함수 (기존 shutdown 함수 개선 버전)
void InuSensorManager::shutdown()
{
    std::cout << "=== Starting sensor shutdown sequence ===" << std::endl;
    
    try {
        // Phase 1: Graceful stream stopping
        std::cout << "Phase 1: Graceful stream shutdown..." << std::endl;
        
        std::vector<std::pair<std::string, bool>> shutdownResults;
        
        // Stop all streams with result tracking
        if (depthStream_ && depthInitialized_) {
            bool result = stopDepth();
            shutdownResults.push_back({"Depth", result});
            if (result) waitForStreamShutdown("Depth", 500);
        }

        if (rgbStream_ && rgbInitialized_) {
            bool result = stopRGB();
            shutdownResults.push_back({"RGB", result});
            if (result) waitForStreamShutdown("RGB", 500);
        }
        
        if (rgbRegStream_ && rgbregInitialized_) {
            bool result = stopRGBRegister();
            shutdownResults.push_back({"RGB Registered", result});
            if (result) waitForStreamShutdown("RGB Registered", 500);
        }

        if ((imuAccStream_ || imuGyroStream_) && imuInitialized_) {
            bool result = stopIMU();
            shutdownResults.push_back({"IMU", result});
            if (result) waitForStreamShutdown("IMU", 500);
        }

        // Report shutdown results
        std::cout << "Stream shutdown results:" << std::endl;
        bool allStopped = true;
        for (const auto& result : shutdownResults) {
            std::cout << "  " << result.first << ": " << (result.second ? "SUCCESS" : "FAILED") << std::endl;
            if (!result.second) allStopped = false;
        }

        // Phase 2: Clear callbacks and terminate streams
        std::cout << "Phase 2: Clearing callbacks and terminating streams..." << std::endl;
        
        // Clear all callbacks first
        depthCallback_ = nullptr;
        rgbCallback_ = nullptr;
        imuAccCallback_ = nullptr;
        imuGyroCallback_ = nullptr;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Terminate streams in reverse order
        if (!allStopped) {
            std::cout << "Some streams failed to stop gracefully, proceeding with termination..." << std::endl;
        }

        // Terminate IMU streams first
        if (imuGyroStream_) {
            InuDev::CInuError retCode = imuGyroStream_->Terminate();
            std::cout << "IMU Gyro terminate: " << (retCode == InuDev::eOK ? "OK" : std::string(retCode)) << std::endl;
            imuGyroStream_.reset();
        }
        
        if (imuAccStream_) {
            InuDev::CInuError retCode = imuAccStream_->Terminate();
            std::cout << "IMU Acc terminate: " << (retCode == InuDev::eOK ? "OK" : std::string(retCode)) << std::endl;
            imuAccStream_.reset();
        }

        // Terminate RGB streams
        if (rgbRegStream_) {
            InuDev::CInuError retCode = rgbRegStream_->Terminate();
            std::cout << "RGB Registered terminate: " << (retCode == InuDev::eOK ? "OK" : std::string(retCode)) << std::endl;
            rgbRegStream_.reset();
        }
        
        if (rgbStream_) {
            InuDev::CInuError retCode = rgbStream_->Terminate();
            std::cout << "RGB terminate: " << (retCode == InuDev::eOK ? "OK" : std::string(retCode)) << std::endl;
            rgbStream_.reset();
        }

        // Terminate depth stream last
        if (depthStream_) {
            InuDev::CInuError retCode = depthStream_->Terminate();
            std::cout << "Depth terminate: " << (retCode == InuDev::eOK ? "OK" : std::string(retCode)) << std::endl;
            depthStream_.reset();
        }

        // Phase 3: Main sensor shutdown
        std::cout << "Phase 3: Main sensor shutdown..." << std::endl;
        
        if (inuSensor_ && sensorStarted_) {
            std::cout << "All streams terminated. Stopping main sensor..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Extra safety wait
            
            InuDev::CInuError retCode = inuSensor_->Stop();
            if (retCode == InuDev::eOK) {
                std::cout << "Main sensor stopped successfully" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                
                retCode = inuSensor_->Terminate();
                std::cout << "Main sensor terminate: " << (retCode == InuDev::eOK ? "OK" : std::string(retCode)) << std::endl;
            } else {
                std::cout << "Failed to stop main sensor: " << std::string(retCode) << std::endl;
                std::cout << "Attempting forced termination..." << std::endl;
                inuSensor_->Terminate(); // Try to terminate anyway
            }
        }

        // Phase 4: Final cleanup
        std::cout << "Phase 4: Final cleanup..." << std::endl;
        
        // Reset all flags
        sensorInitialized_ = false;
        depthInitialized_ = false;
        rgbInitialized_ = false;
        rgbregInitialized_ = false;
        imuInitialized_ = false;
        sensorStarted_ = false;
        
        // Reset main sensor object
        inuSensor_.reset();
        
        // Final stabilization wait
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::cout << "===  sensor shutdown completed successfully ===" << std::endl;

    } catch (const std::exception& e) {
        std::cout << "Exception during shutdown: " << e.what() << std::endl;
        std::cout << "Attempting emergency cleanup..." << std::endl;
        forceTerminateAllStreams();
        
        // Emergency reset
        sensorInitialized_ = false;
        depthInitialized_ = false;
        rgbInitialized_ = false;
        rgbregInitialized_ = false;
        imuInitialized_ = false;
        sensorStarted_ = false;
        inuSensor_.reset();
        
    } catch (...) {
        std::cout << "Unknown exception during shutdown" << std::endl;
        std::cout << "Attempting emergency cleanup..." << std::endl;
        forceTerminateAllStreams();
        
        // Emergency reset
        sensorInitialized_ = false;
        depthInitialized_ = false;
        rgbInitialized_ = false;
        rgbregInitialized_ = false;
        imuInitialized_ = false;
        sensorStarted_ = false;
        inuSensor_.reset();
    }
}

// Internal callback implementations
void InuSensorManager::depthFrameCallback(std::shared_ptr<InuDev::CDepthStream> stream, 
                                         std::shared_ptr<const InuDev::CImageFrame> frame, 
                                         InuDev::CInuError retCode)
{
    if (depthCallback_) {
        depthCallback_(stream, frame, retCode);
    }
}

void InuSensorManager::rgbFrameCallback(std::shared_ptr<InuDev::CImageStream> stream, 
                                       std::shared_ptr<const InuDev::CImageFrame> frame, 
                                       InuDev::CInuError retCode)
{
    if (rgbCallback_) {
        rgbCallback_(stream, frame, retCode);
    }
}

void InuSensorManager::imuAccFrameCallback(std::shared_ptr<InuDev::CImuStream> stream, 
                                          std::shared_ptr<const InuDev::CImuFrame> frame, 
                                          InuDev::CInuError retCode)
{
    if (imuAccCallback_) {
        imuAccCallback_(stream, frame, retCode);
    }
}

void InuSensorManager::imuGyroFrameCallback(std::shared_ptr<InuDev::CImuStream> stream, 
                                           std::shared_ptr<const InuDev::CImuFrame> frame, 
                                           InuDev::CInuError retCode)
{
    if (imuGyroCallback_) {
        imuGyroCallback_(stream, frame, retCode);
    }
}

} // namespace inusensor_ros2_driver