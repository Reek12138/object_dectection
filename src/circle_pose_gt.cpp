#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <yolov5_ros_msgs/BoundingBoxes.h>
#include <yolov5_ros_msgs/BoundingBox.h>
#include <yolov5_ros_msgs/circle_pose.h>  // 包含自定义消息头文件
#include <std_msgs/Float64.h>  // 包含 Float32 消息类型
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h> 

class CircleDepthCalculator {
public:
    CircleDepthCalculator() : nh("~") {
        left_sub = nh.subscribe("/yolov5/BoundingBoxes_left", 1, &CircleDepthCalculator::leftBboxesCallback, this);
        // right_sub = nh.subscribe("/yolov5/BoundingBoxes_right", 1, &CircleDepthCalculator::rightBboxesCallback, this);
        depth_sub = nh.subscribe("/depth_mean_value", 1, &CircleDepthCalculator::depthCallback, this);  // 添加深度订阅者
        // depth_pub = nh.advertise<yolov5_ros_msgs::circle_pose>("/circle_pose_3d_coordinates_gt", 1); // 使用自定义消息类型
        // pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, &CircleDepthCalculator::poseCallback, this);
        pose_pub = nh.advertise<geometry_msgs::Pose>("/circle_pose_3d_gt", 1); // 使用 geometry_msgs::Pose 消息类型

        // pos_vis_pub_= nh.advertise<geometry_msgs::PoseStamped>("/calculated_vis", 1);   // 可视化
        // pos_first_cir_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/fisrt_cir_vis", 1); // 第一个圆环
        // Initialize left_bboxes and right_bboxes
        left_bboxes.bounding_boxes.clear();
        // right_bboxes.bounding_boxes.clear();

        // Load camera parameters
        // Modify these parameters with your specific camera info
        fx = 320;
        fy = 320;
        cx = 320;
        cy = 240;
        baseline = 0.095; // 95mm
        inner_radius = 1.2; // 1200mm
        outer_radius = 1.56; // 1560mm
    }

    void leftBboxesCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &data) {
        left_bboxes = *data;
        // calculateCenterCoordinates(left_bboxes, right_bboxes);
        calculateCenterCoordinates(left_bboxes);
    }

    // void rightBboxesCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &data) {
    //     right_bboxes = *data;
    //     calculateCenterCoordinates(left_bboxes, right_bboxes);
    // }

    void depthCallback(const std_msgs::Float64::ConstPtr &depth_msg) {
        depth_z = depth_msg->data;
        // 从深度订阅中接收深度信息
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    // 从消息中获取位姿信息
    double cir_x = pose_msg->pose.position.x;
    double cir_y = pose_msg->pose.position.y;
    double cir_z = pose_msg->pose.position.z;

    // ROS_INFO("Received pose: X = %f, Y = %f, Z = %f", cir_x, cir_y, cir_z);
    }

    // void calculateCenterCoordinates(const yolov5_ros_msgs::BoundingBoxes &left_bboxes, const yolov5_ros_msgs::BoundingBoxes &right_bboxes) {
    void calculateCenterCoordinates(const yolov5_ros_msgs::BoundingBoxes &left_bboxes) {
        // if (left_bboxes.bounding_boxes.size() > 0 && right_bboxes.bounding_boxes.size() > 0) {
        if (left_bboxes.bounding_boxes.size() > 0 ) {
            for (const yolov5_ros_msgs::BoundingBox &left_bbox : left_bboxes.bounding_boxes) {
                // for (const yolov5_ros_msgs::BoundingBox &right_bbox : right_bboxes.bounding_boxes) {
                    // if (left_bbox.Class == "circle" && right_bbox.Class == "circle") {
                    if (left_bbox.Class == "circle" ) {
                        // Calculate the center coordinates
                        int left_center_x = (left_bbox.xmin + left_bbox.xmax) / 2;
                        // int right_center_x = (right_bbox.xmin + right_bbox.xmax) / 2;
                        
                        // int top_y = std::min(left_bbox.ymin, right_bbox.ymin);
                        // int bottom_y = std::max(left_bbox.ymax, right_bbox.ymax);

                        int top_y = left_bbox.ymax;
                        int bottom_y = left_bbox.ymin;

                        // int center_x = (left_center_x + right_center_x) / 2;
                        int center_x = left_center_x ;
                        int center_y = (top_y + bottom_y) / 2;

                        // Use the received depth information
                        double Z = depth_z/225;
                        // double Z_2 = depth_z/100;

                        // Calculate X and Y in camera coordinates
                        double X = (center_x - cx) * Z / fx;
                        double Y = (center_y - cy) * Z / fy;
                        // double X = X_c ;

                        // Publish the 3D coordinates using custom message
                        publish3DCoordinates(X, Y, Z);
                    }
                    // if (left_bbox.Class == "circle_half" && right_bbox.Class == "circle_half") {
                    if (left_bbox.Class == "circle_half" ) {
                        // Calculate the center coordinates
                        int left_center_x = (left_bbox.xmin + left_bbox.xmax) / 2;
                        // int right_center_x = (right_bbox.xmin + right_bbox.xmax) / 2;
                        
                        // int top_y = std::min(left_bbox.ymin, right_bbox.ymin);
                        // int bottom_y = std::max(left_bbox.ymax, right_bbox.ymax);

                        int top_y = left_bbox.ymax;
                        int bottom_y = left_bbox.ymin;

                        // int center_x = (left_center_x + right_center_x) / 2;
                        int center_x = left_center_x ;
                        int center_y = (top_y + bottom_y) / 2;

                        // Use the received depth information
                        double Z = depth_z/225;
                        // double Z_2 = depth_z/100;

                        // Calculate X and Y in camera coordinates
                        double X = (center_x - cx) * Z / fx;
                        double Y = (center_y - cy) * Z / fy;
                        // double X = X_c - 0.0475;

                        // Publish the 3D coordinates using custom message
                        publish3DCoordinates(X, Y, Z);
                    }
                // }
            }
        }
    }

    void publish3DCoordinates(double X, double Y, double Z) {
        // Create a custom message to publish the 3D coordinates
        // yolov5_ros_msgs::circle_pose pose_msg;
        // pose_msg.x = X;
        // pose_msg.y = Y;
        // pose_msg.z = Z;
        // depth_pub.publish(pose_msg);

        // geometry_msgs::PoseStamped posestamp_msg;
        // posestamp_msg.header.stamp = ros::Time::now();
        // posestamp_msg.header.frame_id = "map";
        // posestamp_msg.pose.position.x = Z + 15.76 + 0.26;
        // // posestamp_msg.pose.position.x = Z + 15.76 ;
        // posestamp_msg.pose.position.y = X - 9.28;
        // posestamp_msg.pose.position.z = Y + 0.866;

        // posestamp_msg.pose.position.x = Z + 17.2265 + 0.26;
        // posestamp_msg.pose.position.y = X + 9.32833;
        // posestamp_msg.pose.position.z = -Y -0.6895;

        // posestamp_msg.pose.position.x = Z + cir_x + 0.26;
        // posestamp_msg.pose.position.y = -X + cir_y;
        // posestamp_msg.pose.position.z = Y - cir_z;

        // posestamp_msg.pose.position.x = cir_x ;
        // posestamp_msg.pose.position.y = cir_y;
        // posestamp_msg.pose.position.z = cir_z;

        // + 15.76
        // - 9.28;
        // + 0.866

        // x: 17.226598739624023
        // y: 9.32833480834961
        // z: -0.6895652413368225



        // posestamp_msg.pose.orientation.x = 0;
        // posestamp_msg.pose.orientation.y = 0;
        // posestamp_msg.pose.orientation.z = 0;
        // posestamp_msg.pose.orientation.w = 1;
        
        // pos_vis_pub_.publish(posestamp_msg);
        // posestamp_msg.header.frame_id = "map";
        // posestamp_msg.pose.position.x = 20.166597366333008;
        // posestamp_msg.pose.position.y = 9.188346862792969;
        // posestamp_msg.pose.position.z = -0.9761993288993835;
        // pos_first_cir_pub_.publish(posestamp_msg);

        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = X;
        pose_msg.position.y = Y;
        pose_msg.position.z = Z;
        pose_msg.orientation.x = 0;
        pose_msg.orientation.y = 0;
        pose_msg.orientation.z = 0;
        pose_msg.orientation.w = 1;
        
        pose_pub.publish(pose_msg);
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
    // ros::Subscriber right_sub;

    ros::Subscriber pose_sub;

    ros::Subscriber depth_sub;  // 添加深度信息订阅者
    ros::Publisher depth_pub;
    // ros::Publisher pos_vis_pub_;

    ros::Publisher pose_pub;

    // ros::Publisher pos_first_cir_pub_;
    int image_width;
    int image_height;
    double fx, fy, cx, cy;
    double baseline;
    double inner_radius, outer_radius;

    yolov5_ros_msgs::BoundingBoxes left_bboxes;
    // yolov5_ros_msgs::BoundingBoxes right_bboxes;
    float depth_z;  // 存储接收到的深度信息

    // double cir_x;
    // double cir_y;
    // double cir_z;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_depth_calculator");
    CircleDepthCalculator calculator;

    calculator.run();
    return 0;
}
