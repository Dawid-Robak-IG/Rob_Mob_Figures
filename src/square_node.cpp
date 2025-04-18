#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

constexpr double PI = 3.14159265;

class SquarePublisher : public rclcpp::Node{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    double side;
    geometry_msgs::msg::Twist move_cmd;
    geometry_msgs::msg::Twist stop_move_cmd;
    double linear_speed;
    double angular_speed;
    double stop_time;
    double current_x;
    double current_y;
    double current_yaw;
    bool odom_updated;

    // rclcpp::TimerBase::SharedPtr check_manual_timer;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub;

    double obstacle_threshold = 0.5;
    bool can_forward;
    bool can_backward;

    bool is_manual;

public:
    SquarePublisher() : Node("square_movement_node"){
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SquarePublisher::odomCallback, this, std::placeholders::_1)
        );

        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SquarePublisher::laserCallback, this, std::placeholders::_1)
        );

        manual_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/manual_toggle", 10,
            std::bind(&SquarePublisher::manualCallback, this, std::placeholders::_1));

        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub;

        this->declare_parameter<double>("side", 0.5);
        this->get_parameter("side", side);
        RCLCPP_INFO(this->get_logger(), "square side: %.2f",side);

        this->declare_parameter<double>("speed", 0.15);
        this->get_parameter("speed", linear_speed);
        linear_speed = std::clamp(linear_speed, 0.1, 0.26);
        RCLCPP_INFO(this->get_logger(), "square linear speed: %.2f",linear_speed);

        angular_speed = 0.5;
        stop_time = 1.0;
        odom_updated=false;

        // check_manual_timer = this->create_wall_timer(
        //     std::chrono::milliseconds(500),  // Check every 500 ms
        //     std::bind(&SquarePublisher::checkIfManual, this)
        // );

        is_manual = false;

        stop_move_cmd.linear.x = 0.0;
        stop_move_cmd.angular.z = 0.0;
    }
    void move(){
        rclcpp::Rate rate(50); 

        for(int i=0;i<4;i++){
            move_cmd.linear.x = linear_speed;
            move_cmd.angular.z = 0.0;

            while (!odom_updated && rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "Waiting for odometry update...");
                rclcpp::spin_some(shared_from_this());
                rate.sleep();
            }

            double start_x = current_x; 
            double start_y = current_y;

            while (rclcpp::ok() && hypot(current_x - start_x, current_y - start_y) < side){
                check_for_manual();
                if(can_forward){
                    publisher->publish(move_cmd);
                } else{
                    publisher->publish(stop_move_cmd);
                }
                
                rclcpp::spin_some(shared_from_this());
                rate.sleep();
            }

            move_cmd.linear.x = 0.0;
            move_cmd.angular.z = angular_speed;

            double target_yaw = current_yaw + M_PI_2;
            if (target_yaw > M_PI) target_yaw -= 2 * M_PI;

            while (rclcpp::ok() && fabs(current_yaw - target_yaw) > 0.05) {
                check_for_manual();
                publisher->publish(move_cmd);
                rclcpp::spin_some(shared_from_this());
                rate.sleep();
            }
        }
        stopRobot();
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
  auto square = std::make_shared<SquarePublisher>();
  square->move();

  rclcpp::shutdown();
  return 0;
}