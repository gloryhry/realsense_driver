/*
 * _______________#########_______________________ 
 * ______________############_____________________ 
 * ______________#############____________________ 
 * _____________##__###########___________________ 
 * ____________###__######_#####__________________ 
 * ____________###_#######___####_________________ 
 * ___________###__##########_####________________ 
 * __________####__###########_####_______________ 
 * ________#####___###########__#####_____________ 
 * _______######___###_########___#####___________ 
 * _______#####___###___########___######_________ 
 * ______######___###__###########___######_______ 
 * _____######___####_##############__######______ 
 * ____#######__#####################_#######_____ 
 * ____#######__##############################____ 
 * ___#######__######_#################_#######___ 
 * ___#######__######_######_#########___######___ 
 * ___#######____##__######___######_____######___ 
 * ___#######________######____#####_____#####____ 
 * ____######________#####_____#####_____####_____ 
 * _____#####________####______#####_____###______ 
 * ______#####______;###________###______#________ 
 * ________##_______####________####______________ 
 * 
 * @Author: Glory Huang
 * @Date: 2023-04-18 20:27:11
 * @LastEditors: Glory Huang
 * @LastEditTime: 2023-04-19 00:12:00
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#ifndef REALSENSE_DRIVER_H_
#define REALSENSE_DRIVER_H_

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
// #include <example.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class Camera
{
public:
    Camera()
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        std::string color_topic, depth_topic, infrared_topic;
        private_nh.param<std::string>("color_topic", color_topic, "/camera/color/image_raw");
        private_nh.param<std::string>("depth_topic", depth_topic, "/camera/depth/image_raw");
        private_nh.param<std::string>("infrared_topic", infrared_topic, "/camera/infrared/image_raw");

        color_pub = nh.advertise<sensor_msgs::Image>(color_topic, 1);
        depth_pub = nh.advertise<sensor_msgs::Image>(depth_topic, 1);
        infrared_pub = nh.advertise<sensor_msgs::Image>(infrared_topic, 1);
    }
    ~Camera() {}

public:
    ros::Publisher color_pub;
    ros::Publisher depth_pub;
    ros::Publisher infrared_pub;

    rs2::context ctx;
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config pipe_config;

    cv::Mat color_raw;
    cv::Mat depth_raw;
    cv::Mat infrared_raw;

public:
    float get_depth_scale(rs2::device dev)
    {
        // 遍历设备的传感器
        for (rs2::sensor &sensor : dev.query_sensors())
        {
            // 检查传感器是否是深度传感器
            if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
                return dpt.get_depth_scale();
        }
        throw std::runtime_error("Device does not have a depth sensor");
    }

    bool profile_changed(const std::vector<rs2::stream_profile> &current, const std::vector<rs2::stream_profile> &prev)
    {
        for (auto &&sp : prev)
        {
            // if previous profile is in current ( maybe just added another)
            auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile &current_sp)
                                    { return sp.unique_id() == current_sp.unique_id(); });
            if (itr == std::end(current))
            {
                return true;
            }
        }
        return false;
    }

    int rs_imshow()
    {
        try
        {
            rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

            auto devs = ctx.query_devices();
            int device_num = devs.size();
            std::cout << "获取相机设备号：" << device_num << std::endl;                                   // Device amount
            rs2::device dev = *devs.begin();
            std::cout << "rs2_camera_name： " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;                   //相机名字
            std::cout << "rs2_camera_serial_number： " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl; //序列号
            std::cout << "rs2_firmware_version： " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;  //固件版本

            pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
            pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
            pipe_config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
            // Start streaming with default recommended configuration
            // The default video configuration contains Depth and Color streams
            // If a device is capable to stream IMU data, both Gyro and A   ccelerometer are enabled by default
            rs2::pipeline_profile profile = pipe.start(pipe_config);
            auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            const auto depth_intrinsics = depth_stream.get_intrinsics();

            auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            auto color_intrinsics = color_stream.get_intrinsics();

            auto infrared_stream = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
            auto infrared_intrinsics = infrared_stream.get_intrinsics();

            float depth_scale = get_depth_scale(profile.get_device());
            std::cout << "depth_scale = " << depth_scale << std::endl;

            // 选择彩色图像数据流来作为对齐对象
            rs2_stream align_to = RS2_STREAM_INFRARED; // 对齐的是彩色图，所以彩色图是不变的
            // 将深度图对齐到RGB图
            rs2::align align(align_to);

            const auto window_name = "Display Image";
            cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
            while (ros::ok())
            {
                rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                ros::Time now_time = ros::Time::now();
                // 正在对齐深度图到其他图像流，我们要确保对齐的图像流不发生改变
                if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
                {
                    // 如果profile发生改变，则更新align对象，重新获取深度图像像素到长度单位的转换比例
                    profile = pipe.get_active_profile();
                    align = rs2::align(align_to);
                    depth_scale = get_depth_scale(profile.get_device());
                }
                // 获取对齐后的帧
                auto processed = align.process(data);
                // 尝试获取对齐后的深度图像帧和其他帧
                rs2::frame aligned_color_frame = processed.get_color_frame();                         // RGB图
                rs2::frame aligned_depth_frame = processed.get_depth_frame();                         // 深度图
                rs2::frame aligned_infrared_frame = processed.get_infrared_frame();                   // 红外图
                // 获取对齐之前的color图像
                // rs2::frame before_depth_frame = data.get_depth_frame().apply_filter(color_map); // 获取对齐之前的深度图

                // Query frame size (width and height)
                const int color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
                const int color_h = aligned_color_frame.as<rs2::video_frame>().get_height();
                const int depth_w = aligned_depth_frame.as<rs2::video_frame>().get_width();
                const int depth_h = aligned_depth_frame.as<rs2::video_frame>().get_height();
                // const int b_color_w = before_depth_frame.as<rs2::video_frame>().get_width();
                // const int b_color_h = before_depth_frame.as<rs2::video_frame>().get_height();
                const int infrared_w = aligned_infrared_frame.as<rs2::video_frame>().get_width();
                const int infrared_h = aligned_infrared_frame.as<rs2::video_frame>().get_height();
                // 如果其中一个未能获取，继续迭代
                if (!aligned_depth_frame || !aligned_color_frame || !aligned_infrared_frame)
                {
                    continue;
                }
                // Create OpenCV matrix of size (w,h) from the colorized depth data
                color_raw = cv::Mat(cv::Size(color_w, color_h), CV_8UC3, (void *)aligned_color_frame.get_data(), cv::Mat::AUTO_STEP);
                depth_raw = cv::Mat(cv::Size(depth_w, depth_h), CV_16UC1, (void *)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
                infrared_raw = cv::Mat(cv::Size(infrared_w, infrared_h), CV_8UC1, (void *)aligned_infrared_frame.get_data());

                // // PointCloud
                // // 创建一个深度帧和一个内参结构体
                // rs2::depth_frame depth_frame = processed.get_depth_frame();
                // rs2::video_stream_profile depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
                // rs2_intrinsics intrin = depth_profile.get_intrinsics();
                // // 获取深度值和像素坐标
                // float depth_value = depth_frame.get_distance(x, y);
                // float pixel[2] = {(float)x, (float)y};
                // float point[3];
                // rs2_deproject_pixel_to_point(point, &intrin, pixel, depth_value);

                std_msgs::Header header;
                header.frame_id = "camera";
                header.stamp = now_time;
                sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(header, "rgb8", color_raw).toImageMsg();
                sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, "mono16", depth_raw).toImageMsg();
                sensor_msgs::ImagePtr infrared_msg = cv_bridge::CvImage(header, "mono8", infrared_raw).toImageMsg();
                color_pub.publish(*color_msg);
                depth_pub.publish(*depth_msg);
                infrared_pub.publish(*infrared_msg);
            }
            return EXIT_SUCCESS;
        }
        catch (const rs2::error &e)
        {
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }
};

#endif