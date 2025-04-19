#include <algorithm>
#include "angles/angles.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

constexpr double PI = 3.14159265;

class CirclePublisher : public rclcpp::Node{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    double radius;
    double percent;
    geometry_msgs::msg::Twist move_cmd;
    geometry_msgs::msg::Twist stop_move_cmd;
    double linear_speed;
    double angular_speed;
    double stop_time;
    double current_yaw;
    bool odom_updated;

    // rclcpp::TimerBase::SharedPtr check_manual_timer;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub;

    double obstacle_threshold = 0.5;
    bool can_forward;
    bool can_backward;

    bool is_manual;

public:
    CirclePublisher() : Node("circle_movement_node"){
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&CirclePublisher::odomCallback, this, std::placeholders::_1)
        );

        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&CirclePublisher::laserCallback, this, std::placeholders::_1)
        );

        manual_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/manual_toggle", 10,
            std::bind(&CirclePublisher::manualCallback, this, std::placeholders::_1));

        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub;

        this->declare_parameter<double>("radius", 0.5);
        this->get_parameter("radius", radius);
        if(radius<0)radius*=-1;
        RCLCPP_INFO(this->get_logger(), "circle radius: %.2f",radius);

        this->declare_parameter<double>("percent", 100.0);
        this->get_parameter("percent", percent);
        percent = std::clamp(percent, 0.0, 100.0);
        RCLCPP_INFO(this->get_logger(), "circle percent to do: %.2f",percent);

        linear_speed = 0.2;
        angular_speed = linear_speed/radius;
        stop_time = 1.0;
        odom_updated=false;

        is_manual = false;

        stop_move_cmd.linear.x = 0.0;
        stop_move_cmd.angular.z = 0.0;

        can_forward = true;
        can_backward = true;
    }
    void move(){
        rclcpp::Rate rate(50); 

        move_cmd.linear.x = linear_speed;
        move_cmd.angular.z = angular_speed;

        double angle = 2*PI*(percent/100.0); 

        while (!odom_updated && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for odometry update...");
            rclcpp::spin_some(shared_from_this());
            rate.sleep();
        }

        double total_rot = 0.0;
        double last_yaw = current_yaw;

        while (rclcpp::ok() && total_rot < angle) {
            check_for_manual();
            if(can_forward){
                publisher->publish(move_cmd);
            } else{
                publisher->publish(stop_move_cmd);
            }
            rclcpp::spin_some(shared_from_this());
            double delta = angles::shortest_angular_distance(last_yaw, current_yaw);
            total_rot += fabs(delta);
            // RCLCPP_INFO(this->get_logger(), "total rot: %.2f",total_rot);
            last_yaw = current_yaw;
            rate.sleep();
        }
        stopRobot();
    }
private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Convert quaternion to yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw = yaw;
        odom_updated=true;
    }
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        double angle_min = msg->angle_min;
        double angle_increment = msg->angle_increment;
        int total_readings = msg->ranges.size();
    
        double front_min = std::numeric_limits<double>::infinity();
        double back_min = std::numeric_limits<double>::infinity();
    
        for (int i = 0; i < total_readings; ++i) {
            double angle_deg = (angle_min + i * angle_increment) * 180.0 / M_PI; /// rad na stopien
            if (angle_deg < 0) angle_deg += 360;

            double range = msg->ranges[i];
            if (!std::isfinite(range)) continue;
    
            if ((angle_deg >= 0 && angle_deg <= 90) || (angle_deg >= 270 && angle_deg < 360)) {
                if (range < front_min) front_min = range;
            }
            else {
                if (range < back_min) back_min = range;
            }
        }
    
        can_forward = front_min > obstacle_threshold;
        can_backward = back_min > obstacle_threshold;
    }
    void manualCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        is_manual = msg->data;
    }   
    void check_for_manual(){
        rclcpp::Rate rate(10); 
        if (is_manual) {
            stopRobot();
            RCLCPP_INFO(this->get_logger(), "Manual is ON. Program stopped");
            while (is_manual && rclcpp::ok()) {
                rclcpp::spin_some(shared_from_this());
                rate.sleep();
            }
            RCLCPP_INFO(this->get_logger(), "Manual is OFF. Resuming");
        }
    } 
    // void checkIfManual(){
    //     size_t publisher_count = this->count_publishers("/cmd_vel");

    //     if(publisher_count > 1){
    //         RCLCPP_WARN(this->get_logger(), "Detected more than one publsiher, ending process");
    //         rclcpp::shutdown();
    //     }
    // }
    void stopRobot(){
        rclcpp::Rate rate(10); 
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(stop_time)) {
            publisher->publish(stop_move_cmd);
            rate.sleep();
        }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto circle = std::make_shared<CirclePublisher>();
  circle->move();

  rclcpp::shutdown();
  return 0;
}

