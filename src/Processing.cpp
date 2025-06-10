#include "Processing.h"
#include <chrono>
#include <cstring>
#include <thread>
#include <iostream>

Processing::Processing(CircleBuffer *&circleBuffer)
        : m_circleBuffer(circleBuffer),
          m_lhForwardFrame(new LhForwardFrame),
          m_lhForwardImage(new LhForwardImage(IMAGE_WIDTH, IMAGE_HEIGHT)),
          m_frameDataBuf(new uint8_t[PACK_SIZE * PACK_MAX_NUM]),
          m_intensity(new double[PACK_SIZE * PACK_MAX_NUM]),
          m_panSectorMemory(new uint8_t[(uint64_t)PACK_SIZE * PACK_MAX_NUM]){
            INIT_ROS_PUBLISHER(pub_image_cartesian, std_msgs::UInt8MultiArray, "/ikan/fls/data_cartesian", 1);
            INIT_ROS_PUBLISHER(pub_image_polar, Msg_SurveyorInfo, "/ikan/fls/data", 1);
            INIT_ROS_PUBLISHER(pub_image_cartesian_compressed, Msg_CompressedImage, "/ikan/fls/sonar_img_cartesian/compressed", 1);
            INIT_ROS_PUBLISHER(pub_image_cartesian_fd_compressed, Msg_CompressedImage, "/ikan/sonar/image/compressed", 1);

           }

Processing::~Processing() {
    delete m_lhForwardFrame;
    delete m_lhForwardImage;
    delete[] m_frameDataBuf;
    delete[] m_intensity;
    delete[] m_panSectorMemory;
}


void Processing::threadFunc() {
    int count     = 0;
    uint8_t *data = new uint8_t[PACK_SIZE];

    while (m_runFlag) {
        if (m_circleBuffer->isAnyData()) {
            memset(data, 0, PACK_SIZE);
            memcpy(data, m_circleBuffer->readDataAddr(), PACK_SIZE);
            if (m_lhForwardFrame->writeOnePackData(data) == LhForwardFrame::oneFrameOK) {
                m_lhForwardFrame->getOneFrame(&curFrameInfo, m_frameDataBuf);
                char l_devName[16] = {0};
                memcpy(l_devName, curFrameInfo.deviceName, 15);
                device_name = l_devName;

                if (curFrameInfo.packType == 0) {  

                    ////////////////////////////////* Publish Polar image as uint8[] *////////////////////////////////
                    // Assume numPerRow is the number of angle steps and dataHeight is the number of range bins
                    int num_ranges = curFrameInfo.dataHeight;
                    int num_angles = curFrameInfo.numPerRow;

                    // Create a cv::Mat directly from the raw polar data in m_frameDataBuf
                    cv::Mat polar_image(num_ranges, num_angles, CV_8UC1, m_frameDataBuf);

                    // Define compression parameters (shared for both polar and Cartesian images)
                    std::vector<int> params;
                    params.push_back(cv::IMWRITE_JPEG_QUALITY);
                    params.push_back(90);  


                    // Populate SurveyorInfo message
                    Msg_SurveyorInfo msg;

                    // Convert ros::Time::now() to Unix time in milliseconds
                    ros::Time current_time = ros::Time::now();
                    msg.unix_time_ms = static_cast<uint64_t>(current_time.toSec() * 1000.0);

                    // Set number of range bins (height of polar image)
                    msg.num_range_bins = num_ranges;

                    msg.sonar_type = 10;  

                    // Set maximum range (already available in curFrameInfo.maxDist)
                    msg.max_range = curFrameInfo.maxDist;

                    // Set FOV bounds
                    float hfov = 160.0f;  
                    msg.fov_min = -hfov / 2.0f;  // Lower/left bound (-80 degrees)
                    msg.fov_max = hfov / 2.0f;   // Upper/right bound (+80 degrees)
                    msg.temperature = curFrameInfo.temp2;  
                    msg.gain = curFrameInfo.brightness;         
                    msg.gamma = curFrameInfo.gamma;         
                    msg.beam_ranges.clear();

                    // Compress polar image and assign to msg.data
                    std::vector<uchar> compressed_data;
                    cv::imencode(".jpg", polar_image, compressed_data, params);
                    msg.data.assign(compressed_data.begin(), compressed_data.end());

                    PUBLISH_ROS(pub_image_polar, msg);

                    ///////////////////////////////////////* Convert Polar to Cart with cv::warpPolar *///////////////////////////////
                    // Transpose the beam mask so that radius becomes X, angle becomes Y
                    cv::Mat polar_image_opencv;
                    cv::transpose(polar_image, polar_image_opencv);


                    // Adjust the polar image to represent a full 360-degree range
                    int radius = polar_image_opencv.cols;   // Number of range bins (radius)
                    int angles = polar_image_opencv.rows;   // Number of angular steps (originally 512 for 160 degrees)

                    // Calculate the number of rows needed for a full 360-degree range
                    // 160 degrees maps to 512 rows, so 360 degrees maps to (360 / 160) * 512 rows
                    int full_angles = static_cast<int>((360.0f / hfov) * angles);  // e.g., (360 / 160) * 512 = 1152 rows
                    cv::Mat full_polar_image = cv::Mat::zeros(full_angles, radius, CV_8UC1);  // New polar image for 360 degrees

                    // Calculate the angular range for 0–10 degrees, 10–170 degrees, and 170–180 degrees in the 360-degree image
                    float degrees_per_row = 360.0f / full_angles;  // Degrees per row in the new image
                    int rows_0_to_10 = static_cast<int>(10.0f / degrees_per_row);  // Rows for 0–10 degrees
                    int rows_10_to_170 = static_cast<int>((170.0f - 10.0f) / degrees_per_row);  // Rows for 10–170 degrees
                    int start_row_10_deg = rows_0_to_10;  // Starting row for 10 degrees
                    int end_row_170_deg = start_row_10_deg + rows_10_to_170;  // Ending row for 170 degrees

                    // Map the original 160-degree data (512 rows) to the 10–170 degree range (160 degrees)
                    // The original polar image has 512 rows for 160 degrees, so we stretch/compress it into the 10–170 degree range
                    for (int y = 0; y < rows_10_to_170; y++) {
                        // Map the row in the 10–170 degree range to the original 0–160 degree range
                        float original_angle = (y / static_cast<float>(rows_10_to_170)) * hfov;  // Angle in original 160-degree range
                        int original_row = static_cast<int>(original_angle / (hfov / angles));  // Corresponding row in original image
                        original_row = std::min(original_row, angles - 1);  // Clamp to valid range

                        // Copy the row from the original polar image to the new polar image
                        for (int x = 0; x < radius; x++) {
                            full_polar_image.at<uchar>(start_row_10_deg + y, x) = polar_image_opencv.at<uchar>(original_row, x);
                        }
                    }

                    // Rows outside 10–170 degrees are already 0 (black) due to cv::Mat::zeros

                    // Set polar warp parameters for the new 360-degree polar image
                    radius = full_polar_image.cols;   // Radius stays the same
                    // radius = 484;   // Radius stays the same
                    angles = full_polar_image.rows;   // New number of angular steps (e.g., 1152)
                    //// Adjust output size for a landscape appearance
                    // int output_height = 2 * radius;  // Height proportional to radius
                    // int output_width = output_height;  // Make it square to represent a full circle
                    int output_height = IMAGE_HEIGHT;  // Height proportional to radius
                    int output_width = IMAGE_WIDTH;  // Make it square to represent a full circle
                    cv::Size dsize(output_width, output_height);

                    // Center at the middle of the output image for a full circle
                    cv::Point2f center(dsize.width / 2.0f, 0.0f);

                    // Adjust maxRadius for the full circle mapping
                    double maxRadius = static_cast<double>(radius);
                    // Warp Polar → Cartesian
                    cv::Mat cartesian_image_polar;
                    cv::warpPolar(full_polar_image, cartesian_image_polar, dsize, center, maxRadius,
                                cv::WARP_POLAR_LINEAR | cv::WARP_INVERSE_MAP);
                    cv::rotate(cartesian_image_polar, cartesian_image_polar, cv::ROTATE_180);
                    // Publish the Cartesian image as sensor_msgs::CompressedImage
                    std::vector<int> params_cart_from_polar;
                    params_cart_from_polar.push_back(cv::IMWRITE_JPEG_QUALITY);
                    params_cart_from_polar.push_back(90);  // JPEG quality (0-100)
                    std::vector<uchar> compressed_data_cart_1;
                    cv::imencode(".jpg", cartesian_image_polar, compressed_data_cart_1, params_cart_from_polar);

                    Msg_CompressedImage cart_msg_compressed_from_polar;
                    cart_msg_compressed_from_polar.header.stamp = ros::Time::now();
                    cart_msg_compressed_from_polar.format = "jpeg";
                    cart_msg_compressed_from_polar.data.assign(compressed_data_cart_1.begin(), compressed_data_cart_1.end());
                    PUBLISH_ROS(pub_image_cartesian_compressed, cart_msg_compressed_from_polar);


                    // Publish as std_msgs::UInt8MultiArray
                    std_msgs::UInt8MultiArray cart_msg_compressed;
                    cart_msg_compressed.data.assign(compressed_data_cart_1.begin(), compressed_data_cart_1.end());

                    PUBLISH_ROS(pub_image_cartesian, cart_msg_compressed);



                    /////////////////////////////////////////* Convert Polar to Cartesian using generateForwardImage *////////////////////////////////////
                    double imageRes = 0;
                    m_lhForwardImage->generateForwardImage(curFrameInfo, m_frameDataBuf, m_panSectorMemory, imageRes);
                    cv::Mat cartesian_image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, m_panSectorMemory);
                    cv::flip(cartesian_image, cartesian_image, 0);

                    
                    // Remove 27 pixels from the top
                    int top_crop = 27;
                    // Remove 7 pixels from the left and 7 pixels from the right
                    int left_crop = 7;
                    int right_crop = 7;

                    // Calculate new dimensions after cropping
                    int cropped_height = IMAGE_HEIGHT - top_crop;
                    int cropped_width = IMAGE_WIDTH - left_crop - right_crop;

                    // Define the region of interest (ROI) for cropping
                    cv::Rect roi(left_crop, top_crop, cropped_width, cropped_height);
                    cv::Mat cropped_image = cartesian_image(roi);

                    // Resize the cropped image to 968x512
                    int target_width = 968;
                    int target_height = 512;
                    cv::Mat resized_image;
                    cv::resize(cropped_image, resized_image, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);


                    // Publish as CompressedImage
                    Msg_CompressedImage compressed_msg;
                    compressed_msg.header.stamp = ros::Time::now();
                    compressed_msg.format = "jpeg";
                    // Compress the resized image using cv::imencode
                    std::vector<uchar> compressed_data_FD_cart;
                    cv::imencode(".jpg", resized_image, compressed_data_FD_cart);
                    // Assign the compressed data to the message
                    compressed_msg.data.assign(compressed_data_FD_cart.begin(), compressed_data_FD_cart.end());
                    // Publish the compressed image
                    PUBLISH_ROS(pub_image_cartesian_fd_compressed, compressed_msg);


                } 
            }
        } else {
            count++;
            if (count >= 10) {
                count = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
    delete[] data;
}
