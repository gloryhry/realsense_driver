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
#include <thread>

#define Align_To_Color 0
#define Align_To_Infrared 1

class Camera
{
public:
    Camera() {}
    Camera(int number_, std::string serial_, bool color_, bool depth_, bool infrared_, int align_, ros::NodeHandle nh_)
    {
        nh = nh_;
        camera_number = number_;
        camera_serial_number = serial_;
        color = color_;
        depth = depth_;
        infrared = infrared_;
        align_to = align_;
        pipe_config.enable_device(camera_serial_number);
        if (color)
        {
            pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        }
        if (depth)
        {
            pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
        }
        if (infrared)
        {
            pipe_config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
        }

        profile = pipe.start(pipe_config);
        if (depth)
        {
            auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            const auto depth_intrinsics = depth_stream.get_intrinsics();
            float depth_scale = get_depth_scale(profile.get_device());
            std::cout << "Camera" << camera_number << " depth_scale = " << depth_scale << std::endl;
        }
        if (color)
        {
            auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            auto color_intrinsics = color_stream.get_intrinsics();
        }
        if (infrared)
        {
            auto infrared_stream = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
            auto infrared_intrinsics = infrared_stream.get_intrinsics();
        }

        // 选择彩色图像数据流来作为对齐对象

        // 对齐的是彩色图，所以彩色图是不变的
        if (align_to == Align_To_Color)
        {
            aligned = RS2_STREAM_COLOR;
        }
        if (align_to == Align_To_Infrared)
        {
            aligned = RS2_STREAM_INFRARED;
        }
        // 将深度图对齐到RGB图
        align = rs2::align(aligned);
    }
    void set_Camera(int number_, std::string serial_, bool color_, bool depth_, bool infrared_, int align_,ros::NodeHandle nh_)
    {
        nh = nh_;
        camera_number = number_;
        camera_serial_number = serial_;
        color = color_;
        depth = depth_;
        infrared = infrared_;
        align_to = align_;
        pipe_config.enable_device(camera_serial_number);
        if (depth)
        {
            pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        }
        if (color)
        {
            pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
        }
        if (infrared)
        {
            pipe_config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
        }

        profile = pipe.start(pipe_config);
        if (depth)
        {
            auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            const auto depth_intrinsics = depth_stream.get_intrinsics();
            float depth_scale = get_depth_scale(profile.get_device());
            std::cout << "Camera" << camera_number << " depth_scale = " << depth_scale << std::endl;
        }
        if (color)
        {
            auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            auto color_intrinsics = color_stream.get_intrinsics();
        }
        if (infrared)
        {
            auto infrared_stream = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
            auto infrared_intrinsics = infrared_stream.get_intrinsics();
        }

        // 选择彩色图像数据流来作为对齐对象
        // 对齐的是彩色图，所以彩色图是不变的
        if (align_to == Align_To_Color)
        {
            aligned = RS2_STREAM_COLOR;
        }
        if (align_to == Align_To_Infrared)
        {
            aligned = RS2_STREAM_INFRARED;
        }
        // 将深度图对齐到RGB图
        align = rs2::align(aligned);
    }
    void set_color_pub(std::string color_topic)
    {
        color_pub = nh.advertise<sensor_msgs::Image>(color_topic, 1);
        color_init = true;
    }
    void set_depth_pub(std::string depth_topic)
    {
        depth_pub = nh.advertise<sensor_msgs::Image>(depth_topic, 1);
        depth_init = true;
    }
    void set_infrared_pub(std::string infrared_topic)
    {
        infrared_pub = nh.advertise<sensor_msgs::Image>(infrared_topic, 1);
        infrared_init = true;
    }
    ~Camera() {}

public:
    ros::NodeHandle nh;
    int align_to = 0;
    bool color = true, depth = false, infrared = false;
    int camera_number;
    std::string camera_serial_number;
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config pipe_config;
    rs2::pipeline_profile profile;
    rs2_stream aligned;
    rs2::align align = rs2::align(RS2_STREAM_COLOR);
    cv::Mat color_raw;
    cv::Mat depth_raw;
    cv::Mat infrared_raw;
    ros::Publisher color_pub;
    ros::Publisher depth_pub;
    ros::Publisher infrared_pub;
    bool color_init = false,depth_init=false,infrared_init=false;

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
            while (ros::ok())
            {
                rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                ros::Time now_time = ros::Time::now();
                // 正在对齐深度图到其他图像流，我们要确保对齐的图像流不发生改变
                if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
                {
                    // 如果profile发生改变，则更新align对象，重新获取深度图像像素到长度单位的转换比例
                    profile = pipe.get_active_profile();
                    align = rs2::align(aligned);
                    float depth_scale = get_depth_scale(profile.get_device());
                }
                // 获取对齐后的帧
                auto processed = align.process(data);

                // 尝试获取对齐后的深度图像帧和其他帧
                rs2::frame aligned_color_frame, aligned_depth_frame, aligned_infrared_frame;
                if (color)
                {
                    aligned_color_frame = processed.get_color_frame(); // RGB图
                }
                if (depth)
                {
                    aligned_depth_frame = processed.get_depth_frame(); // 深度图
                }
                if (infrared)
                {
                    aligned_infrared_frame = processed.get_infrared_frame(); // 红外图
                }
                // 获取对齐之前的color图像
                // rs2::frame before_depth_frame = data.get_depth_frame().apply_filter(color_map); // 获取对齐之前的深度图

                // Query frame size (width and height)
                int color_w, color_h, depth_w, depth_h, infrared_w, infrared_h;
                if (aligned_color_frame)
                {
                    color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
                    color_h = aligned_color_frame.as<rs2::video_frame>().get_height();
                    color_raw = cv::Mat(cv::Size(color_w, color_h), CV_8UC3, (void *)aligned_color_frame.get_data(), cv::Mat::AUTO_STEP);
                }
                if (aligned_depth_frame)
                {
                    depth_w = aligned_depth_frame.as<rs2::video_frame>().get_width();
                    depth_h = aligned_depth_frame.as<rs2::video_frame>().get_height();
                    depth_raw = cv::Mat(cv::Size(depth_w, depth_h), CV_16UC1, (void *)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
                }
                if (aligned_infrared_frame)
                {
                    infrared_w = aligned_infrared_frame.as<rs2::video_frame>().get_width();
                    infrared_h = aligned_infrared_frame.as<rs2::video_frame>().get_height();
                    infrared_raw = cv::Mat(cv::Size(infrared_w, infrared_h), CV_8UC1, (void *)aligned_infrared_frame.get_data());
                }

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
                header.stamp = now_time;

                if (color && color_init)
                {
                    header.frame_id = "camera_color";
                    sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(header, "rgb8", color_raw).toImageMsg();
                    color_pub.publish(*color_msg);
                }
                if (depth && depth_init)
                {
                    header.frame_id = "camera_depth";
                    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, "mono16", depth_raw).toImageMsg();
                    depth_pub.publish(*depth_msg);
                }
                if (infrared && infrared_init)
                {
                    header.frame_id = "camera_infrared";
                    sensor_msgs::ImagePtr infrared_msg = cv_bridge::CvImage(header, "mono8", infrared_raw).toImageMsg();
                    infrared_pub.publish(*infrared_msg);
                }

                // sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(header, "rgb8", color_raw).toImageMsg();
                // sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, "mono16", depth_raw).toImageMsg();
                // sensor_msgs::ImagePtr infrared_msg = cv_bridge::CvImage(header, "mono8", infrared_raw).toImageMsg();
                // color_pub.publish(*color_msg);
                // depth_pub.publish(*depth_msg);
                // infrared_pub.publish(*infrared_msg);
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