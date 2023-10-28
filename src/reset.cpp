#include <ros/ros.h>
#include <airsim_ros/Reset.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reset_drone_node");
    ros::NodeHandle nh;

    // 创建一个服务客户端，用于重置无人机的位置
    ros::ServiceClient reset_client = nh.serviceClient<airsim_ros::Reset>("/airsim_node/reset");

    // 创建 Reset 服务请求
    airsim_ros::Reset reset;
    // 设置需要等待上一个任务完成
    // reset.request.waitOnLastTask = true;

    // 调用重置服务
    if (reset_client.call(reset))
    {
        ROS_INFO("Drone position reset successfully.");
    }
    else
    {
        ROS_ERROR("Failed to reset drone position.");
        return 1;
    }

    return 0;
}

