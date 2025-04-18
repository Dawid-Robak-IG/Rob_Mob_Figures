#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

constexpr double PI = 3.14159265;

class SpiralPublisher : public rclcpp::Node{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    double increment_speed;
    double decrease_speed;
    geometry_msgs::msg::Twist move_cmd;
    geometry_msgs::msg::Twist stop_move_cmd;
    double linear_speed;
    double angular_speed;
    double stop_time;
    double current_yaw;
    bool odom_updated;

    // rclcpp::TimerBase::SharedPtr check_manual_timer;
    rclcpp::TimerBase::SharedPtr increment_timer;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub;

    double obstacle_threshold = 0.5;
    bool can_forward;
    bool can_backward;

    bool is_manual;

public:
    SpiralPublisher() : Node("spiral_movement_node"){
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SpiralPublisher::laserCallback, this, std::placeholders::_1)
        );

        manual_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/manual_toggle", 10,
            std::bind(&SpiralPublisher::manualCallback, this, std::placeholders::_1));


        increment_timer = this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&SpiralPublisher::incrementMove,this));
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub;

        this->declare_parameter<double>("inc_spd", 0.01);
        this->get_parameter("inc_spd", increment_speed);
        RCLCPP_INFO(this->get_logger(), "spiral increment_speed: %.2f",increment_speed);

        this->declare_parameter<double>("dec_spd", -0.02);
        this->get_parameter("dec_spd", decrease_speed);
        RCLCPP_INFO(this->get_logger(), "spiral decrease_speed (for angular): %.2f",decrease_speed);

        linear_speed = 0.1;
        angular_speed = 0.82;
        stop_time = 1.0;
        odom_updated=false;

        is_manual = false;

        stop_move_cmd.linear.x = 0.0;
        stop_move_cmd.angular.z = 0.0;

        can_forward = true;
        can_backward = true;
    }
    void move(){
        double f = 50;
        rclcpp::Rate rate(f); 

        move_cmd.linear.x = linear_speed;
        move_cmd.angular.z = angular_speed;

        while (rclcpp::ok() && angular_speed>0.0) {
            check_for_manual();
            move_cmd.linear.x = linear_speed;
            move_cmd.angular.z = angular_speed;
            if(can_forward && can_backward){
                publisher->publish(move_cmd);
            } else{
                publisher->publish(stop_move_cmd);
            }
            rclcpp::spin_some(shared_from_this());
            rate.sleep();
        }
        stopRobot();
    }
private:
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
    void incrementMove(){
        linear_speed += increment_speed;
        linear_speed = std::clamp(linear_speed,0.0,0.26);
        angular_speed += decrease_speed;
        angular_speed = std::clamp(angular_speed,0.0,1.82);
    }
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
  auto spiral = std::make_shared<SpiralPublisher>();
  spiral->move();

  rclcpp::shutdown();
  return 0;
}

