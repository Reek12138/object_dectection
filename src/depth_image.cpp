#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

ros::Publisher depth_image_pub;  // 声明发布者

void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& msg)
{
    // 获取焦距和基线信息
    // float f = msg->f;
    // float T = msg->T;

    float f = 320;
    float T = 95;


    // 获取视差图像消息
    sensor_msgs::Image disparity_image = msg->image;

    // 创建深度图像消息
    sensor_msgs::Image depth_image;
    depth_image.header = disparity_image.header;
    depth_image.height = disparity_image.height;
    depth_image.width = disparity_image.width;
    depth_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;  // 32位浮点数
    depth_image.step = 4 * depth_image.width;
    depth_image.data.resize(depth_image.height * depth_image.step);

    // 计算深度信息
    for (int i = 0; i < disparity_image.height; ++i)
    {
        for (int j = 0; j < disparity_image.width; ++j)
        {
            // 获取当前像素的视差值
            float disparity = *(reinterpret_cast<const float*>(&disparity_image.data[i * disparity_image.step + j * 4]));

            // 计算深度
            if (disparity > 0)
            {
                float depth = f * T / disparity;
                *(reinterpret_cast<float*>(&depth_image.data[i * depth_image.step + j * 4])) = depth;
            }
            else
            {
                // 处理无效视差值
                *(reinterpret_cast<float*>(&depth_image.data[i * depth_image.step + j * 4])) = 0.0;
            }
        }
    }

    // 发布深度图像
    depth_image_pub.publish(depth_image);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_from_disparity");
    ros::NodeHandle nh;

    ros::Subscriber disparity_sub = nh.subscribe<stereo_msgs::DisparityImage>("/airsim_node/drone_1/disparity", 1, disparityCallback);
    depth_image_pub = nh.advertise<sensor_msgs::Image>("/depth_image", 1);

    ros::spin();

    return 0;
}
