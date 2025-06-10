#include "SonarNode.h"
#include <bx_msgs/RateLimiter.hpp>
#include <bx_msgs/RosBindings.hpp>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    INIT_ROS_NODE("sonar_node", 0, "sonar_node/alive");

    std::string manual_ip;
    GET_ROS_PARAM("~ip_addr", manual_ip, "192.168.1.26");
    LOG_INFO("Connecting to Sonar @ %s\n", manual_ip.c_str());

    try {
        SonarNode node(manual_ip);
        node.run();
        RateLimiter rate(30);
        while (IS_ROS_NODE_OK()) {
            ROS_SPIN_ONCE();
            rate.sleep();
        }
    } catch (const std::exception &e) {
        ROS_ERROR("Error: %s", e.what());
        return 1;
    }

    return 0;
}