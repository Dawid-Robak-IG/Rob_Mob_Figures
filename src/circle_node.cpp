#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

constexpr double PI = 3.14159265;

class CirclePublisher : public rclcpp::Node{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    double radius;
    geometry_msgs::msg::Twist move_cmd;
    double linear_speed;
    double angular_speed;
    double stop_time;
    double current_x;
    double current_y;
    double current_yaw;
    double percent;

public:
    SquarePublisher() : Node("circle_movement_node"){
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SquarePublisher::odomCallback, this, std::placeholders::_1));

        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        this->declare_parameter<double>("radius", 0.5);
        this->get_parameter("radius", radius);
        RCLCPP_INFO(this->get_logger(), "circle radius: %.2f",radius);
        linear_speed = 0.2;
        angular_speed = linear_speed/radius;

        stop_time = 1.0;

        this->declare_parameter<double>("percent", 100.0);
        this->get_parameter("percent", percent);
        percent = std::clamp(percent, 0.0, 100.0);

    }
    void move(){
        move_cmd.linear.x = linear_speed;   
        move_cmd.angular.z = angular_speed;  

        double angle = 2*PI*(percent/100.0); 
        double drive_duration = angle/angular_speed;

        rclcpp::Rate rate(50); 
        auto start = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double>(drive_duration); 

        while (rclcpp::ok() && (std::chrono::steady_clock::now()-start<drive_duration) ) {
            publisher->publish(move_cmd);
            rclcpp::spin_some(shared_from_this()); // Keep odometry updating if needed
            rate.sleep();
        }

        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0;
        start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(stop_time)) {
            publisher->publish(move_cmd);
            rclcpp::spin_some(shared_from_this());
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