#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <yolov5_ros_msgs/BoundingBoxes.h>
#include <yolov5_ros_msgs/BoundingBox.h>
#include <yolov5_ros_msgs/circle_pose.h>  // 包含自定义消息头文件
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CircleDepthCalculator {
public:
    CircleDepthCalculator() : nh("~") {
        left_sub = nh.subscribe("/yolov5/BoundingBoxes_left", 1, &CircleDepthCalculator::leftBboxesCallback, this);
        right_sub = nh.subscribe("/yolov5/BoundingBoxes_right", 1, &CircleDepthCalculator::rightBboxesCallback, this);
        depth_pub = nh.advertise<yolov5_ros_msgs::circle_pose>("/circle_pose_3d_coordinates", 1); // 使用自定义消息类型

        // Initialize left_bboxes and right_bboxes
        left_bboxes.bounding_boxes.clear();
        right_bboxes.bounding_boxes.clear();

        // Load camera parameters
        // Modify these parameters with your specific camera info
        fx = 320.0;
        fy = 320.0;
        cx = 320.0;
        cy = 240.0;
        baseline = 0.095; // 95mm
        inner_radius = 1.2; // 1200mm
        outer_radius = 1.56; // 1560mm

        cv::namedWindow("Depth Image", cv::WINDOW_NORMAL);
    }

    void leftBboxesCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &data) {
        left_bboxes = *data;
        calculateCenterCoordinates(left_bboxes, right_bboxes);
    }

    void rightBboxesCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &data) {
        right_bboxes = *data;
        calculateCenterCoordinates(left_bboxes, right_bboxes);
    }

    void calculateCenterCoordinates(const yolov5_ros_msgs::BoundingBoxes &left_bboxes, const yolov5_ros_msgs::BoundingBoxes &right_bboxes) {
        if (left_bboxes.bounding_boxes.size() > 0 && right_bboxes.bounding_boxes.size() > 0) {
            for (const yolov5_ros_msgs::BoundingBox &left_bbox : left_bboxes.bounding_boxes) {
                for (const yolov5_ros_msgs::BoundingBox &right_bbox : right_bboxes.bounding_boxes) {
                    if (left_bbox.Class == "circle" && right_bbox.Class == "circle") {
                        // Calculate the center coordinates
                        int left_center_x = (left_bbox.xmin + left_bbox.xmax) / 2;
                        int right_center_x = (right_bbox.xmin + right_bbox.xmax) / 2;
                        int top_y = std::min(left_bbox.ymin, right_bbox.ymin);
                        int bottom_y = std::max(left_bbox.ymax, right_bbox.ymax);

                        int center_x = (left_center_x + right_center_x) / 2;
                        int center_y = (top_y + bottom_y) / 2;

                        // Calculate depth (Z)
                        double disparity = left_center_x - right_center_x;
                        double Z =3 * baseline * fx / disparity;

                        // Calculate X and Y in camera coordinates
                        double X = (center_x - cx) * Z / fx;
                        double Y = (center_y - cy) * Z / fy;

                        // Publish the 3D coordinates using custom message
                        publish3DCoordinates(X, Y, Z);
                    }
                    else if(left_bbox.Class == "circle_half" && right_bbox.Class == "circle_half")
                    {
                        // Calculate the center coordinates
                        int left_center_x = (left_bbox.xmin + left_bbox.xmax) / 2;
                        int right_center_x = (right_bbox.xmin + right_bbox.xmax) / 2;
                        int top_y = std::min(left_bbox.ymin, right_bbox.ymin);
                        int bottom_y = std::max(left_bbox.ymax, right_bbox.ymax);

                        int center_x = (left_center_x + right_center_x) / 2;
                        int center_y = (top_y + bottom_y) / 2;

                        // Calculate depth (Z)
                        double disparity = left_center_x - right_center_x;
                        double Z = baseline * fx / disparity;

                        // Calculate X and Y in camera coordinates
                        double X = (center_x - cx) * Z / fx;
                        double Y = (center_y - cy) * Z / fy;

                        // Publish the 3D coordinates using custom message
                        publish3DCoordinates(X, Y, Z);
                    }
                }
            }
        }
    }

    void publish3DCoordinates(double X, double Y, double Z) {
        // Create a custom message to publish the 3D coordinates
        yolov5_ros_msgs::circle_pose pose_msg;
        pose_msg.x = X;
        pose_msg.y = Y;
        pose_msg.z = Z;

        depth_pub.publish(pose_msg);
    }

    void run() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber left_sub;
    ros::Subscriber right_sub;
    ros::Publisher depth_pub;
    int image_width;
    int image_height;
    double fx, fy, cx, cy;
    double baseline;
    double inner_radius, outer_radius;
    yolov5_ros_msgs::BoundingBoxes left_bboxes;
    yolov5_ros_msgs::BoundingBoxes right_bboxes;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_depth_calculator");
    CircleDepthCalculator calculator;
    calculator.run();
    return 0;
}


