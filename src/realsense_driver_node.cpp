/*
 *           佛曰:
 *                   写字楼里写字间，写字间里程序员；
 *                   程序人员写程序，又拿程序换酒钱。
 *                   酒醒只在网上坐，酒醉还来网下眠；
 *                   酒醉酒醒日复日，网上网下年复年。
 *                   但愿老死电脑间，不愿鞠躬老板前；
 *                   奔驰宝马贵者趣，公交自行程序员。
 *                   别人笑我忒疯癫，我笑自己命太贱；
 *                   不见满街漂亮妹，哪个归得程序员？
 *
 * @Author: Glory Huang
 * @Date: 2023-04-18 20:08:32
 * @LastEditors: Glory Huang
 * @LastEditTime: 2023-04-18 21:28:26
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <realsense_driver.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_driver");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string color_topic, depth_topic, infrared_topic;
    bool enable_color, enable_depth, enable_infrared;
    std::string camera_serials;
    int camera_number;
    private_nh.param<std::string>("color_topic", color_topic, "/color/image_raw");
    private_nh.param<std::string>("depth_topic", depth_topic, "/depth/image_raw");
    private_nh.param<std::string>("infrared_topic", infrared_topic, "/infrared/image_raw");
    private_nh.param<bool>("enable_color", enable_color, true);
    private_nh.param<bool>("enable_depth", enable_depth, false);
    private_nh.param<bool>("enable_infrared", enable_infrared, false);
    private_nh.param<std::string>("camera_serials", camera_serials, "");
    private_nh.param<int>("camera_number", camera_number, 0);

    color_topic = "/camera" + std::to_string(camera_number) + color_topic;
    depth_topic = "/camera" + std::to_string(camera_number) + depth_topic;
    infrared_topic = "/camera" + std::to_string(camera_number) + infrared_topic;

    ros::Publisher color_pub;
    ros::Publisher depth_pub;
    ros::Publisher infrared_pub;
    color_pub = nh.advertise<sensor_msgs::Image>(color_topic, 1);
    depth_pub = nh.advertise<sensor_msgs::Image>(depth_topic, 1);
    infrared_pub = nh.advertise<sensor_msgs::Image>(infrared_topic, 1);

    rs2::log_to_console(RS2_LOG_SEVERITY_WARN);
    rs2::context ctx;
    auto devs = ctx.query_devices();
    int device_num = devs.size();
    std::cout << "获取到相机设备数量：" << device_num << std::endl; // Device amount

    for (auto dev : devs)
    {
        std::cout << "-------------------" << std::endl;
        std::cout << "rs2_camera_name： " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;                   // 相机名字
        std::cout << "rs2_camera_serial_number： " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl; // 序列号
        std::cout << "rs2_firmware_version： " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;  // 固件版本
    }
    rs2::device dev = *devs.begin();

    Camera cam;
    if (camera_serials == "")
    {
        camera_serials = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    }
    cam.set_Camera(camera_number, camera_serials, enable_color, enable_depth, enable_infrared, Align_To_Color, nh);
    cam.set_color_pub(color_topic);
    cam.set_depth_pub(depth_topic);
    cam.set_infrared_pub(infrared_topic);
    cam.rs_imshow();

    return 0;
}