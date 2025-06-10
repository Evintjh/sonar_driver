#include "SonarNode.h"
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <bx_msgs/RosBindings.hpp>

SonarNode::SonarNode(const std::string &manual_ip)
    : circle_buffer_(new CircleBuffer(LhForwardFrame::PACK_SIZE, PACK_MAX_NUM)),
      processing_(new Processing(circle_buffer_)),
      udp_receiver_(new UdpReceiver(circle_buffer_)),
      manual_ip_(manual_ip),
      cmd_(nullptr),
      power_status_(false),
      gamma_(1.0),
      range_(1.0),
      brightness_(18),
      mode_(1) {
    processing_->start();
    udp_receiver_->start();

    if (manual_ip_.empty()) {
        getHostSocket();
        if (udp_sockets_.empty()) {
            ROS_WARN("No valid broadcast sockets created for discovery");
        }
    }
    INIT_ROS_SERVICE_SERVER(srv_set_gamma, "set_gamma", &SonarNode::callback_gammaRequest);
    INIT_ROS_SERVICE_SERVER(srv_set_range, "set_range", &SonarNode::callback_rangeRequest);
    INIT_ROS_SERVICE_SERVER(srv_set_freq, "set_frequency", &SonarNode::callback_freqRequest);
    INIT_ROS_SERVICE_SERVER(srv_set_brightness, "set_brightness", &SonarNode::callback_brightnessRequest);
    std::cout << "gamma: " << gamma_ << std::endl;
    std::cout << "range: " << range_ << std::endl;
    std::cout << "brightness: " << brightness_ << std::endl;
    std::cout << "mode: 1200kHz" << std::endl;

    ROS_INFO("'set_gamma' and 'set_range' and 'set_frequency' and 'set_brightness' services are ready.");
}

SonarNode::~SonarNode() {
    processing_->stop();
    udp_receiver_->stop();

    if (cmd_ != nullptr) {
        cmd_->sendCMD(LhForwardCMD::startStop, (uint8_t)0);
        delete cmd_;
    }

    delete processing_;
    delete udp_receiver_;
    delete circle_buffer_;

    for (int sock : udp_sockets_) {
        if (sock >= 0) {
            shutdown(sock, SHUT_RDWR);
            close(sock);
        }
    }
}

void SonarNode::run() {
    if (!manual_ip_.empty()) {
        ROS_INFO("Using manual IP: %s", manual_ip_.c_str());
        // std::string host_ip = selectHostIP();
        std::string host_ip = "192.168.1.1";
        if (tryConnect(manual_ip_, host_ip)) {
            ROS_INFO("Successfully connected to sonar at %s", manual_ip_.c_str());
        } else {
            ROS_ERROR("Failed to connect to sonar at %s", manual_ip_.c_str());
        }
    } else {
        std::thread discovery_thread(&SonarNode::connectTimerActive, this);
        ROS_INFO("Starting discovery");
        discovery_thread.detach();
    }
}

std::string SonarNode::selectHostIP() {
    struct ifaddrs *ifaddr, *ifa;
    if (getifaddrs(&ifaddr) == -1) {
        throw std::runtime_error("Failed to get network interfaces");
    }

    std::string host_ip;
    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr, ip, INET_ADDRSTRLEN);
        std::string ip_str(ip);
        if (ip_str != "127.0.0.1" && ip_str.find("172.17.") != 0) {
            host_ip = ip_str;
            break;
        }
    }
    freeifaddrs(ifaddr);

    if (host_ip.empty()) {
        throw std::runtime_error("No suitable host IP found");
    }
    return host_ip;
}

void SonarNode::getHostSocket() {
    struct ifaddrs *ifaddr, *ifa;
    if (getifaddrs(&ifaddr) == -1) {
        throw std::runtime_error("Failed to get network interfaces");
    }

    std::vector<std::string> host_ips;
    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr, ip, INET_ADDRSTRLEN);
        host_ips.push_back(ip);
    }
    freeifaddrs(ifaddr);

    for (int sock : udp_sockets_) {
        if (sock >= 0) {
            shutdown(sock, SHUT_RDWR);
            close(sock);
        }
    }
    udp_sockets_.clear();

    for (const auto &ip : host_ips) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            ROS_WARN("Failed to create socket for IP: %s", ip.c_str());
            continue;
        }

        int buffer_size = 10 * 1'024 * 1'024;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
        struct timeval timeout = {0, 50'000};
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        int broadcast = 1;
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
        int reuse = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(0);
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            ROS_WARN("Failed to bind socket to IP: %s", ip.c_str());
            close(sock);
            continue;
        }

        udp_sockets_.push_back(sock);
        ROS_INFO("Bound socket to IP: %s", ip.c_str());
    }
}

bool SonarNode::tryConnect(const std::string &sonar_ip, const std::string &host_ip) {
    uint8_t ip1, ip2, ip3, ip4;
    ip1 = ip2 = ip3 = ip4 = 0;

    std::vector<std::string> ip_parts;
    std::string ip_part;
    for (char c : host_ip) {
        if (c == '.') {
            ip_parts.push_back(ip_part);
            ip_part.clear();
        } else {
            ip_part += c;
        }
    }
    ip_parts.push_back(ip_part);

    if (ip_parts.size() == 4) {
        ip1 = std::stoi(ip_parts[0]);
        ip2 = std::stoi(ip_parts[1]);
        ip3 = std::stoi(ip_parts[2]);
        ip4 = std::stoi(ip_parts[3]);
    } else {
        ROS_ERROR("Invalid host IP format: %s", host_ip.c_str());
        return false;
    }

    cmd_ = new LhForwardCMD(sonar_ip, TCP_CMD_PORT);
    int ret = cmd_->connectDevice(host_ip);
    if (ret != LhForwardCMD::ok) {
        ROS_ERROR("Failed to connect to sonar device at %s", sonar_ip.c_str());
        delete cmd_;
        cmd_ = nullptr;
        return false;
    }

    ret = cmd_->sendCMD(LhForwardCMD::dataSource, (uint8_t)0);  // Request raw polar data (packType == 0)
    if (ret != LhForwardCMD::ok) {
        ROS_ERROR("Failed to set data source to raw polar data");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1'000));

    uint8_t ip[4] = {ip1, ip2, ip3, ip4};
    ret = cmd_->sendCMD(LhForwardCMD::udpData, ip, UDP_REC_SDK_PORT);
    if (ret != LhForwardCMD::ok) {
        ROS_ERROR("Failed to set UDP data destination");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1'000));

    ret = cmd_->sendCMD(LhForwardCMD::startStop, (uint8_t)1);
    uint8_t temp = power_status_ ? 0 : 255;
    LhForwardCMD::LfCMDRet ret_power = cmd_->sendCMD(LhForwardCMD::power, temp);
    if (ret != LhForwardCMD::ok) {
        ROS_ERROR("Failed to start sonar");
        delete cmd_;
        cmd_ = nullptr;
        return false;
    }

    return true;
}

void SonarNode::connectTimerActive() {
    if (udp_sockets_.empty()) {
        ROS_ERROR("No broadcast sockets available for discovery");
        return;
    }

    struct sockaddr_in sonar_addr;
    memset(&sonar_addr, 0, sizeof(sonar_addr));
    sonar_addr.sin_family = AF_INET;
    sonar_addr.sin_port = htons(UDP_DEV_ASK_PORT);
    sonar_addr.sin_addr.s_addr = INADDR_BROADCAST;

    char search_msg[] = "SVSearching";

    while (ros::ok()) {
        for (int sock : udp_sockets_) {
            if (sendto(sock, search_msg, sizeof(search_msg), 0, (struct sockaddr *)&sonar_addr, sizeof(sonar_addr)) < 0) {
                ROS_WARN("Failed to send broadcast on socket %d", sock);
                continue;
            }

            char buffer[1'024] = {0};
            struct sockaddr_in recv_addr;
            socklen_t addr_len = sizeof(recv_addr);
            ssize_t recv_len = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&recv_addr, &addr_len);
            if (recv_len > 0) {
                std::string reply(buffer, recv_len);
                if (reply.find("SV") != std::string::npos || reply[0] == (char)0xa0) {
                    char ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &recv_addr.sin_addr, ip, INET_ADDRSTRLEN);
                    ROS_INFO("Sonar found at IP: %s", ip);

                    std::string host_ip = selectHostIP();
                    ROS_INFO("Using host IP: %s", host_ip.c_str());

                    if (tryConnect(ip, host_ip)) {
                        ROS_INFO("Successfully connected to sonar at %s", ip);
                    } else {
                        ROS_ERROR("Failed to connect to sonar at %s", ip);
                    }
                    return;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void SonarNode::setPower() {
    if (cmd_ == nullptr) return;

    uint8_t temp = power_status_ ? 0 : 255;
    LhForwardCMD::LfCMDRet ret = cmd_->sendCMD(LhForwardCMD::power, temp);
    if (ret != LhForwardCMD::ok) {
        ROS_ERROR("Failed to set sonar power: %d", ret);
        return;
    }

    power_status_ = !power_status_;
    ROS_INFO("Sonar power set to: %s", power_status_ ? "ON" : "OFF");
}

void SonarNode::setGamma(int direction) {
    if (cmd_ == nullptr) {
        ROS_ERROR("Sonar not connected. Cannot set gamma.");
        return;
    }

    if (direction == 1) gamma_ += 0.1;
    else if (direction == 0) gamma_ -= 0.1;

    gamma_ = std::min(std::max(0.5f, gamma_), 1.5f);

    LhForwardCMD::LfCMDRet ret = cmd_->sendCMD(LhForwardCMD::gamma, gamma_);
    if (ret != LhForwardCMD::ok) {
        ROS_ERROR("Failed to set gamma to %.2f: %d", gamma_, ret);
    } else {
        ROS_INFO("Gamma set to %.2f", gamma_);
    }
}

bool SonarNode::callback_gammaRequest(bx_msgs::SetInt8::Request &req, bx_msgs::SetInt8::Response &res) {
    if (req.data == 1) {
        setGamma(1);
        res.success = true;
        // ROS_INFO("Gamma increased to %.2f", gamma_);
    } 
    else if (req.data ==0){
        setGamma(0);
        res.success = true;
        // ROS_INFO("Gamma decreased to %.2f", gamma_);
    }
    else ROS_ERROR("No such mode. Either 1: Increase gamma by 0.1, 0: Decrease range by 0.1");

    return true;
}

void SonarNode::setRange(int direction) {
    if (cmd_ == nullptr) {
        ROS_ERROR("Sonar not connected. Cannot set range.");
        return;
    }

    if (direction == 1) range_ += 1.0;
    else if (direction == 0) range_ -= 1.0;
    
    range_ = std::min(std::max(1.0f, range_), 150.0f);

    LhForwardCMD::LfCMDRet ret = cmd_->sendCMD(LhForwardCMD::maxDist, range_);
    if (ret != LhForwardCMD::ok) {
        ROS_ERROR("Failed to set Range to %.2f: %d", range_, ret);
    } else {
        ROS_INFO("Range set to %.2f", range_);
    }
}

bool SonarNode::callback_rangeRequest(bx_msgs::SetInt8::Request &req, bx_msgs::SetInt8::Response &res) {
    if (req.data == 1) {
        setRange(1);
        res.success = true;
        // ROS_INFO("Range increased to %.2f", range_);
    } 
    else if (req.data ==0){
        setRange(0);
        res.success = true;
        // ROS_INFO("Range decreased to %.2f", range_);
    }
    else ROS_ERROR("No such mode. Either 1: Increase range by 1m, 0: Decrease range by 1m");

    return true;
}

void SonarNode::setFreq(int mode) {
    if (cmd_ == nullptr) {
        ROS_ERROR("Sonar not connected. Cannot set gamma.");
        return;
    }

    if (mode == 0) mode_ = 0;
    else if (mode == 1) mode_ = 1;
    
    LhForwardCMD::LfCMDRet ret = cmd_->sendCMD(LhForwardCMD::workMode, mode_);
    if (ret != LhForwardCMD::ok) {
        ROS_ERROR("Failed to set mode to %.2f: %d", mode_, ret);
    } else {
        std::cout << mode_ << std::endl;
    }
}

bool SonarNode::callback_freqRequest(bx_msgs::SetInt8::Request &req, bx_msgs::SetInt8::Response &res) {
    if (req.data == 0) {
        setFreq(0);
        res.success = true;
        ROS_INFO("Mode set to 0: 750kHz");
    } 
    
    else if (req.data ==1) {
        setFreq(1);
        res.success = true;
        ROS_INFO("Mode set to 1: 1.2MHz");
    }
    else ROS_ERROR("No such mode. Either 0: 750kHz, 1: 1200kHz");

    return true;
}

void SonarNode::setBrightness(int direction) {
    if (cmd_ == nullptr) {
        ROS_ERROR("Sonar not connected. Cannot set range.");
        return;
    }

    if (direction == 1) {
        brightness_ += 1;
    } else {
        brightness_ -= 1;
    }

    brightness_ = std::min(std::max(uint8_t(1), brightness_), uint8_t(60));

    LhForwardCMD::LfCMDRet ret = cmd_->sendCMD(LhForwardCMD::brightness, static_cast<uint8_t>(brightness_));
    if (ret != LhForwardCMD::ok) {
        ROS_ERROR("Failed to set Brightness to %d: %d", brightness_, ret);
    } else {
        ROS_INFO("Brightness set to %d", brightness_);
    }
}

bool SonarNode::callback_brightnessRequest(bx_msgs::SetInt8::Request &req, bx_msgs::SetInt8::Response &res) {
    if (req.data == 1) {
        setBrightness(1);
        res.success = true;
        // ROS_INFO("Brightness increased to %.2f", brightness_);
    } 
    else if (req.data == 0) {
        setBrightness(0);
        res.success = true;
        // ROS_INFO("Brightness decreased to %.2f", brightness_);
    }
    else ROS_ERROR("No such mode. Either 0: Increase brightness by 1, 1: Decrease brightness by 1");

    return true;
}