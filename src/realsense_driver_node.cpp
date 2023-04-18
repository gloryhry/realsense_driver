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
    ros::init(argc, argv, "kinco_node");
    // pkg_loc = ros::package::getPath("kinco_driver");
    

    
    Camera d430;
    d430.rs_imshow();

    // ros::spin();
    return 0;
}