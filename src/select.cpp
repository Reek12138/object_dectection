#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <yolov5_ros_msgs/BoundingBoxes.h>
#include <yolov5_ros_msgs/BoundingBox.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class DepthFromBoundingBoxes {
public:
    DepthFromBoundingBoxes() {
        // Initialize the ROS node
        ros::NodeHandle nh;

        // Subscribe to the bounding boxes topic
        bboxes_sub = nh.subscribe<yolov5_ros_msgs::BoundingBoxes>("/yolov5/BoundingBoxes_left", 1, &DepthFromBoundingBoxes::bboxesCallback, this);

        // Subscribe to the depth image topic
        depth_image_sub = nh.subscribe<sensor_msgs::Image>("/depth_image", 1, &DepthFromBoundingBoxes::depthImageCallback, this);

        // Create a publisher for the resulting depth image
        depth_pub = nh.advertise<sensor_msgs::Image>("/depth_image_with_bboxes", 1);

        // Initialize current_depth_image and depth_header if needed
        // e.g., current_depth_image = cv::Mat(depth_image_height, depth_image_width, CV_32FC1);
        // depth_header = std_msgs::Header();
    }

    void bboxesCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr& bboxes_msg) {
        if (current_depth_image.empty()) {
            ROS_WARN("No valid depth image available.");
            return;
        }

        for (const yolov5_ros_msgs::BoundingBox& bbox_msg : bboxes_msg->bounding_boxes) {
            int x_min = bbox_msg.xmin;
            int y_min = bbox_msg.ymin;
            int x_max = bbox_msg.xmax;
            int y_max = bbox_msg.ymax;

            if (x_min >= x_max || y_min >= y_max) {
                ROS_WARN("Invalid bounding box dimensions.");
                continue;
            }

            // Ensure that bounding box coordinates are within the image bounds
            x_min = std::max(0, x_min);
            y_min = std::max(0, y_min);
            x_max = std::min(current_depth_image.cols - 1, x_max);
            y_max = std::min(current_depth_image.rows - 1, y_max);

            // Extract depth information from the depth image for the given bounding box
            cv::Mat bbox_depth = current_depth_image(cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min));

            // Convert the extracted depth information to a sensor_msgs::Image
            sensor_msgs::Image depth_image_msg;
            cv_bridge::CvImage cv_depth_msg;
            cv_depth_msg.header.stamp = depth_header.stamp;  // Copy the timestamp from depth image's header
            cv_depth_msg.header.frame_id = depth_header.frame_id;  // Copy the frame ID from depth image's header
            cv_depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            cv_depth_msg.image = bbox_depth;
            cv_depth_msg.toImageMsg(depth_image_msg);

            // Publish the depth image with bounding box
            depth_pub.publish(depth_image_msg);
        }
    }

    // float depth_threshold=0.2;
    // void bboxesCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr& bboxes_msg) {
    // if (current_depth_image.empty()) {
    //     ROS_WARN("No valid depth image available.");
    //     return;
    // }

    // // Define a depth threshold
    // float depth_threshold = 0.1;  // Adjust this threshold as needed

    // for (const yolov5_ros_msgs::BoundingBox& bbox_msg : bboxes_msg->bounding_boxes) {
    //     int x_min = bbox_msg.xmin;
    //     int y_min = bbox_msg.ymin;
    //     int x_max = bbox_msg.xmax;
    //     int y_max = bbox_msg.ymax;

    //     if (x_min >= x_max || y_min >= y_max) {
    //         ROS_WARN("Invalid bounding box dimensions.");
    //         continue;
    //     }

    //     // Ensure that bounding box coordinates are within the image bounds
    //     x_min = std::max(0, x_min);
    //     y_min = std::max(0, y_min);
    //     x_max = std::min(current_depth_image.cols - 1, x_max);
    //     y_max = std::min(current_depth_image.rows - 1, y_max);

    //     // Extract depth information from the depth image for the given bounding box
    //     cv::Mat bbox_depth = current_depth_image(cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min));

    //     // Apply depth threshold to filter out black (zero depth) pixels
    //     cv::Mat mask = bbox_depth < depth_threshold;
    //     bbox_depth.setTo(0, mask);  // Set values less than threshold to zero

    //     // Convert the extracted depth information to a sensor_msgs::Image
    //     sensor_msgs::Image depth_image_msg;
    //     cv_bridge::CvImage cv_depth_msg;
    //     cv_depth_msg.header.stamp = depth_header.stamp;  // Copy the timestamp from depth image's header
    //     cv_depth_msg.header.frame_id = depth_header.frame_id;  // Copy the frame ID from depth image's header
    //     cv_depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    //     cv_depth_msg.image = bbox_depth;
    //     cv_depth_msg.toImageMsg(depth_image_msg);

    //     // Publish the depth image with bounding box
    //     depth_pub.publish(depth_image_msg);
    //     }   
    // }


    void depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_image_msg) {
        // Convert ROS depth image message to a cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            current_depth_image = cv_ptr->image;
            depth_header = depth_image_msg->header;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

private:
    ros::Subscriber bboxes_sub;
    ros::Subscriber depth_image_sub;
    ros::Publisher depth_pub;
    cv::Mat current_depth_image;
    std_msgs::Header depth_header;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_from_bounding_boxes");
    DepthFromBoundingBoxes depth_from_bboxes;
    ros::spin();
    return 0;
}



