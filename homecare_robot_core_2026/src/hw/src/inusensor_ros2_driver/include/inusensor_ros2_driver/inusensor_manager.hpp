#ifndef INUSENSOR_MANAGER_HPP
#define INUSENSOR_MANAGER_HPP

#include "InuSensor.h"
#include "InuError.h"
#include "DepthStream.h"
#include "ImageStream.h"
#include "ImuStream.h"
#include "CalibrationData.h"
#include "DepthProperties.h"
#include "ImageRegisteredStream.h"
#include "HwInformation.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>
#include <functional>
#include <thread>

namespace inusensor_ros2_driver
{

class InuSensorManager
{
public:
    // Callback function types
    using DepthCallback = std::function<void(std::shared_ptr<InuDev::CDepthStream>, 
                                           std::shared_ptr<const InuDev::CImageFrame>, 
                                           InuDev::CInuError)>;
    using RGBCallback = std::function<void(std::shared_ptr<InuDev::CImageStream>, 
                                         std::shared_ptr<const InuDev::CImageFrame>, 
                                         InuDev::CInuError)>;
    using IMUCallback = std::function<void(std::shared_ptr<InuDev::CImuStream>, 
                                         std::shared_ptr<const InuDev::CImuFrame>, 
                                         InuDev::CInuError)>;

    InuSensorManager();
    ~InuSensorManager();

    // Initialization methods (following original API sequence)
    bool initializeSensor();
    bool initializeDepth();
    bool initializeRGB();
    bool initializeRGBRegister();
    bool initializeIMU();

    // Start/Stop methods
    bool startDepth();
    bool startRGB();
    bool startRGBRegister();
    bool startIMU();
    
    bool stopDepth();
    bool stopRGB();
    bool stopRGBRegister();
    bool stopIMU();

    // Register callbacks for data streams
    bool registerDepthCallback(const DepthCallback& callback);
    bool registerRGBCallback(const RGBCallback& callback);
    bool registerIMUAccCallback(const IMUCallback& callback);
    bool registerIMUGyroCallback(const IMUCallback& callback);

    // Depth post-processing configuration methods
    void setDepthPostProcessingFlags(InuDev::CDepthProperties::EPostProcessing flags);
    InuDev::CDepthProperties::EPostProcessing getDepthPostProcessingFlags() const;

    // Convenience methods for common post-processing combinations
    void enableAllDepthPostProcessing();

    // Calibration data access
    bool getCalibrationData(InuDev::CCalibrationData& calibData, uint32_t channelId = 0);
    void printOpticalData(uint32_t channelId = 0);

    void setDepthResolutionBinning();
    void setDepthResolutionVerticalBinning();
    void setDepthResolutionFull();
    void setDepthResolutionDefault();

    // Cleanup
    void shutdown();

    // Getters for stream objects (for direct access if needed)
    std::shared_ptr<InuDev::CDepthStream> getDepthStream() { return depthStream_; }
    std::shared_ptr<InuDev::CImageStream> getRGBStream() { return rgbStream_; }
    std::shared_ptr<InuDev::CImageStream> getRGBRegStream() { return rgbRegStream_; }
    std::shared_ptr<InuDev::CImuStream> getIMUAccStream() { return imuAccStream_; }
    std::shared_ptr<InuDev::CImuStream> getIMUGyroStream() { return imuGyroStream_; }
    
    bool rgb_to_depthRegistration_ = false;

    // Shutdown helper functions
    bool isStreamActive(std::shared_ptr<InuDev::CDepthStream> stream) const;
    bool isStreamActive(std::shared_ptr<InuDev::CImageStream> stream) const;
    bool isStreamActive(std::shared_ptr<InuDev::CImuStream> stream) const;
    bool waitForStreamShutdown(const std::string& streamName, int maxWaitMs = 1000);
    void forceTerminateAllStreams();

private:
    // Sensor and stream objects
    std::shared_ptr<InuDev::CInuSensor> inuSensor_;
    std::shared_ptr<InuDev::CDepthStream> depthStream_;
    std::shared_ptr<InuDev::CImageStream> rgbStream_;
    std::shared_ptr<InuDev::CImageRegisteredStream> rgbRegStream_;
    std::shared_ptr<InuDev::CImuStream> imuAccStream_;
    std::shared_ptr<InuDev::CImuStream> imuGyroStream_;

    // Channel IDs (based on M4.51S specifications)
    uint32_t depthChannelId_;
    uint32_t rgbChannelId_;
    uint32_t regBaseChannelId_;
    
    // Status flags
    
    bool sensorInitialized_;
    bool depthInitialized_;
    bool rgbInitialized_;
    bool rgbregInitialized_;
    bool imuInitialized_;
    bool sensorStarted_;

    // Depth post-processing configuration
    InuDev::CDepthProperties::EPostProcessing depthPostProcessingFlags_;

    // Depth resolution setting configuration
    bool cparamsInitialized_ = false;
    InuDev::CChannelControlParams CParams;
    std::map<uint32_t, InuDev::CChannelSize> oChannelsSize;
    std::map<uint32_t, InuDev::CChannelControlParams> ioParams;

    // Internal callback wrappers
    void depthFrameCallback(std::shared_ptr<InuDev::CDepthStream> stream, 
                           std::shared_ptr<const InuDev::CImageFrame> frame, 
                           InuDev::CInuError retCode);
    void rgbFrameCallback(std::shared_ptr<InuDev::CImageStream> stream, 
                         std::shared_ptr<const InuDev::CImageFrame> frame, 
                         InuDev::CInuError retCode);
    void imuAccFrameCallback(std::shared_ptr<InuDev::CImuStream> stream, 
                            std::shared_ptr<const InuDev::CImuFrame> frame, 
                            InuDev::CInuError retCode);
    void imuGyroFrameCallback(std::shared_ptr<InuDev::CImuStream> stream, 
                             std::shared_ptr<const InuDev::CImuFrame> frame, 
                             InuDev::CInuError retCode);

    // External callbacks
    DepthCallback depthCallback_;
    RGBCallback rgbCallback_;
    IMUCallback imuAccCallback_;
    IMUCallback imuGyroCallback_;
    
};

} // namespace inusensor_ros2_driver

#endif // INUSENSOR_MANAGER_HPP