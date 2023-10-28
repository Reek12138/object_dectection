#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Float64.h"

class DepthImageProcessor {
public:
    DepthImageProcessor() {
        // Initialize the ROS node
        ros::NodeHandle nh;

        // Subscribe to the filtered depth image topic
        filtered_depth_sub = nh.subscribe<sensor_msgs::Image>("/filtered_depth_image", 1, &DepthImageProcessor::filteredDepthCallback, this);

        // Create a publisher for the depth mean value
        depth_mean_pub = nh.advertise<std_msgs::Float64>("/depth_mean_value", 1);

        // sum_depth = 0.0;
        // num_pixels = 0;
    }

    // void filteredDepthCallback(const sensor_msgs::Image::ConstPtr& filtered_depth_msg) {
    //     // Convert ROS depth image message to a cv::Mat
    //     cv_bridge::CvImagePtr cv_ptr;
    //     try {
    //         cv_ptr = cv_bridge::toCvCopy(filtered_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    //         cv::Mat filtered_depth_image = cv_ptr->image;

    //         // Compute the mean depth value
    //         double mean_depth = cv::mean(filtered_depth_image).val[0];

    //         // Publish the mean depth value
    //         std_msgs::Float64 depth_mean_msg;
    //         depth_mean_msg.data = mean_depth;
    //         depth_mean_pub.publish(depth_mean_msg);
    //     } catch (cv_bridge::Exception& e) {
    //         ROS_ERROR("cv_bridge exception: %s", e.what());
    //         return;
    //     }
    // }
    void filteredDepthCallback(const sensor_msgs::Image::ConstPtr& filtered_depth_msg) {
    // Convert ROS depth image message to a cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(filtered_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat filtered_depth_image = cv_ptr->image;

        // Exclude depth values less than 10 mm
        cv::Mat mask = filtered_depth_image >= 10;
        cv::Mat valid_depth_image;
        filtered_depth_image.copyTo(valid_depth_image, mask);

        // Compute the mean depth value
        double mean_depth = cv::mean(valid_depth_image).val[0];

        // Publish the mean depth value
        std_msgs::Float64 depth_mean_msg;
        depth_mean_msg.data = mean_depth;
        depth_mean_pub.publish(depth_mean_msg);
        } 
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }
    }


private:
    ros::Subscriber filtered_depth_sub;
    ros::Publisher depth_mean_pub;
    double sum_depth;
    int num_pixels;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_image_processor");
    DepthImageProcessor depth_image_processor;
    ros::spin();
    return 0;
}
