#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>

class ManualControlNode : public rclcpp::Node {
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_pub;
    std::thread input_thread;
    std_msgs::msg::Bool toggle_data; 
    rclcpp::TimerBase::SharedPtr send_toggle;
public:
    static bool enabled_raw_count; // to not put new settign to old ones
    ManualControlNode() : Node("manual_control_node") {
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        manual_pub = this->create_publisher<std_msgs::msg::Bool>("/manual_toggle", 10);
        input_thread = std::thread(&ManualControlNode::keyboardLoop, this);
        toggle_data.data = true;
        send_toggle = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&ManualControlNode::sendToggle, this)
        );
    }

    ~ManualControlNode() {
        sendToggle();
        if (input_thread.joinable()) input_thread.join();
        setTerminalRawMode(false);
    }

private:
    void keyboardLoop() {
        setTerminalRawMode(true);
        RCLCPP_INFO(this->get_logger(), "Manual control started. WASD to move, SPACE to stop, Q to quit.");

        char c;
        geometry_msgs::msg::Twist cmd;

        while (1) {
            if (read(STDIN_FILENO, &c, 1) < 0) continue;

            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;

            switch (c) {
                case 'w': cmd.linear.x = 0.26; publisher->publish(cmd); break;
                case 's': cmd.linear.x = -0.26; publisher->publish(cmd); break;
                case 'a': cmd.angular.z = 1.0; publisher->publish(cmd); break;
                case 'd': cmd.angular.z = -1.0; publisher->publish(cmd); break;
                case ' ': cmd.linear.x = 0.0; cmd.angular.z = 0.0; publisher->publish(cmd); break;
                case 'q': 
                    toggle_data.data = false;
                    sendToggle();
                    RCLCPP_INFO(this->get_logger(), "Exiting manual control.");
                    rclcpp::shutdown();
                    return;
                default: continue;
            }
        }
    }
    void setTerminalRawMode(bool enable) {
        static struct termios oldt;
        struct termios newt;
        if (enable && enabled_raw_count==false) {
            tcgetattr(STDIN_FILENO, &oldt);
            enabled_raw_count=true;
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        } else {
            enabled_raw_count=false;
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        }
    }
    void sendToggle(){
        manual_pub->publish(toggle_data);
    }
};

bool ManualControlNode::enabled_raw_count=false;

int main(int argc, char *argv[]) {
    ManualControlNode::enabled_raw_count=false;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}