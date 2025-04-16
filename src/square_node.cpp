#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

constexpr double PI = 3.14159265;

class SquarePublisher : public rclcpp::Node{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    double side;
    geometry_msgs::msg::Twist move_cmd;
    double linear_speed;
    double angular_speed;
    double stop_time;
    double current_x;
    double current_y;
    double current_yaw;

public:
    SquarePublisher() : Node("square_movement_node"){
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SquarePublisher::odomCallback, this, std::placeholders::_1));

        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        this->declare_parameter<double>("side", 0.5);
        this->get_parameter("side", side);
        RCLCPP_INFO(this->get_logger(), "square side: %.2f",side);

        linear_speed = 0.15;
        angular_speed = 0.5;
        stop_time = 1.0;

    }
    void move(){
        double linear_sleep = side/linear_speed;
        RCLCPP_INFO(this->get_logger(), "sleep for linear move: %.2f",linear_sleep);
        double rotate_sleep = ((90*PI)/180)/angular_speed;
        auto start = std::chrono::steady_clock::now();
        rclcpp::Rate rate(50); // 10 Hz

        for(int i=0;i<4;i++){
            move_cmd.linear.x = linear_speed;
            move_cmd.angular.z = 0.0;

            double start_x = current_x; 

            while ( start_x + side != current_x ) {
                publisher->publish(move_cmd);
                rate.sleep();
            }

            move_cmd.linear.x = 0.0;
            move_cmd.angular.z = angular_speed;
            start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(rotate_sleep)) {
                publisher->publish(move_cmd);
                rate.sleep();
            }
        }
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0;
        start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(stop_time)) {
            publisher->publish(move_cmd);
            rate.sleep();
        }
    }
private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;

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
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto square = std::make_shared<SquarePublisher>();
  square->move();

  rclcpp::shutdown();
  return 0;
}