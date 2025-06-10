#ifndef SONAR_NODE_H
#define SONAR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>
#include "LhForwardCMD.h"
#include "LhForwardFrame.h"
#include "Processing.h"
#include "common/CircleBuffer.h"
#include "net/UdpReceiver.h"
#include "bx_msgs/SetInt8.h"

using namespace LhForwardSDK;

class SonarNode {
public:
    SonarNode(const std::string &manual_ip = "");
    ~SonarNode();
    void run();

private:
    std::string selectHostIP();
    void getHostSocket();
    bool tryConnect(const std::string &sonar_ip, const std::string &host_ip);
    void connectTimerActive();
    void setPower();
    void setGamma(int direction);
    bool callback_gammaRequest(bx_msgs::SetInt8::Request &req, bx_msgs::SetInt8::Response &res);
    void setRange(int direction);
    bool callback_rangeRequest(bx_msgs::SetInt8::Request &req, bx_msgs::SetInt8::Response &res);
    void setFreq(int direction);
    bool callback_freqRequest(bx_msgs::SetInt8::Request &req, bx_msgs::SetInt8::Response &res);
    void setBrightness(int direction);
    bool callback_brightnessRequest(bx_msgs::SetInt8::Request &req, bx_msgs::SetInt8::Response &res);

    CircleBuffer *circle_buffer_;
    Processing *processing_;
    UdpReceiver *udp_receiver_;
    std::vector<int> udp_sockets_;
    std::string manual_ip_;
    LhForwardCMD *cmd_;
    struct packageInfo curFrameInfo;
    // ros::ServiceServer gamma_service_;
    ros::ServiceServer range_service_;
    DECLARE_ROS_SERVICE_SERVER(srv_set_gamma, bx_msgs::SetInt8)
    DECLARE_ROS_SERVICE_SERVER(srv_set_range, bx_msgs::SetInt8)
    DECLARE_ROS_SERVICE_SERVER(srv_set_freq, bx_msgs::SetInt8)
    DECLARE_ROS_SERVICE_SERVER(srv_set_brightness, bx_msgs::SetInt8)

    // DECLARE_ROS_SERVICE(srv_set_range, bx_msgs::SetInt8)
    bool power_status_;
    float gamma_;
    float range_;
    uint8_t brightness_;
    uint8_t mode_;
    static const uint16_t UDP_DEV_ASK_PORT = 9'001;
    static const uint16_t UDP_REC_SDK_PORT = 5'009;
    static const uint16_t TCP_CMD_PORT = 5'007;
    uint8_t count = 0;
};

#endif // SONAR_NODE_H