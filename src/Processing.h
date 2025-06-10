#ifndef PROCESSING_H
#define PROCESSING_H

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <stdint.h>
#include <mutex>
#include <string>
#include "LhForwardFrame.h"
#include "LhForwardImage.h"
#include "common/CircleBuffer.h"
#include "common/WhileThread.h"
#include "common/sonarCommon.h"
#include <opencv2/opencv.hpp>  // Add OpenCV for imdecode
#include <bx_msgs/RosBindings.hpp>
#include <bx_msgs/SurveyorInfo.h>
#include <std_msgs/UInt8MultiArray.h>
#include <opencv2/highgui.hpp>
#include <mutex>


using namespace LhForwardSDK;

class Processing : public WhileThread {
public:
    /**
     * @brief Constructor for Processing
     * @param circleBuffer Circular buffer pointer reference
     * @param nh ROS node handle for publishing images
     */
    explicit Processing(CircleBuffer *&circleBuffer);
    ~Processing();

    std::string device_name;
    std::string l_avoidCollisionMsgs;
std::mutex guiMutex;

private:
    uint16_t PACK_SIZE = LhForwardFrame::PACK_SIZE;
    void threadFunc() override;

    CircleBuffer *&m_circleBuffer;
    LhForwardFrame *m_lhForwardFrame;
    LhForwardImage *m_lhForwardImage;
    struct packageInfo curFrameInfo;
    uint8_t *m_frameDataBuf;
    double *m_intensity;
    uint8_t *m_panSectorMemory;

    ros::Publisher image_pub_;  // ROS image publisher
    DECLARE_ROS_PUBLISHER(pub_image_cartesian, std_msgs::UInt8MultiArray)
    DECLARE_ROS_PUBLISHER(pub_image_polar, Msg_SurveyorInfo)
    DECLARE_ROS_PUBLISHER(pub_image_cartesian_compressed, Msg_CompressedImage)
    DECLARE_ROS_PUBLISHER(pub_image_cartesian_fd_compressed, Msg_CompressedImage)

};

#endif  // PROCESSING_H