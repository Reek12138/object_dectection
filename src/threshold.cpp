#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class DepthImageFilter {
public:
    DepthImageFilter() {
        // Initialize the ROS node
        ros::NodeHandle nh;

        // Subscribe to the depth image topic
        depth_image_sub = nh.subscribe<sensor_msgs::Image>("/depth_image_with_bboxes", 1, &DepthImageFilter::depthImageCallback, this);

        // Create a publisher for the filtered depth image
        filtered_depth_pub = nh.advertise<sensor_msgs::Image>("/filtered_depth_image", 1);
    }

    void depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_image_msg) {
        // Convert ROS depth image message to a cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat depth_image = cv_ptr->image;

            // Apply depth filtering (setting values outside of the range [10, 160] to 0)
            cv::Mat filtered_depth_image = cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_32FC1);
            cv::Mat mask = (depth_image >= 2000) & (depth_image <= 6000);//深度距离，单位为毫米，选取这个距离内的像素点
            depth_image.copyTo(filtered_depth_image, mask);

            // Convert the filtered depth image to a sensor_msgs::Image
            sensor_msgs::Image filtered_depth_image_msg;
            cv_bridge::CvImage cv_filtered_depth_msg;
            cv_filtered_depth_msg.header = depth_image_msg->header;
            cv_filtered_depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            cv_filtered_depth_msg.image = filtered_depth_image;
            cv_filtered_depth_msg.toImageMsg(filtered_depth_image_msg);

            // Publish the filtered depth image
            filtered_depth_pub.publish(filtered_depth_image_msg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

private:
    ros::Subscriber depth_image_sub;
    ros::Publisher filtered_depth_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_image_filter");
    DepthImageFilter depth_image_filter;
    ros::spin();
    return 0;
}
