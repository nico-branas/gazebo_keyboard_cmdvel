#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>

class KeyboardToCmdVel : public rclcpp::Node{

  private:

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr key_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  double linear_speed_;
  double angular_speed_;

  void onKey(const std_msgs::msg::Int32 & msg)
  {
    geometry_msgs::msg::Twist t;

    // Qt key codes (comme dans ton SDF)
    constexpr int KEY_LEFT  = 16777234;
    constexpr int KEY_UP    = 16777235;
    constexpr int KEY_RIGHT = 16777236;
    constexpr int KEY_DOWN  = 16777237;

    switch (msg.data)
    {
      case KEY_UP:
        t.linear.x = linear_speed_;
        break;
      case KEY_DOWN:
        t.linear.x = -linear_speed_;
        break;
      case KEY_LEFT:
        t.angular.z = angular_speed_;
        break;
      case KEY_RIGHT:
        t.angular.z = -angular_speed_;
        break;
      default:
        // touche non gérée -> on ignore
        return;
    }

    cmd_vel_pub_->publish(t);
  }



public:
  KeyboardToCmdVel() : Node("keyboard_to_cmdvel"){

    // Paramètres simples (faciles à adapter)
    linear_speed_ = this->declare_parameter<double>("linear_speed", 0.5);
    angular_speed_ = this->declare_parameter<double>("angular_speed", 0.5);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    key_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/keyboard/keypress", 10,
      std::bind(&KeyboardToCmdVel::onKey, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening /keyboard/keypress (std_msgs/Int32), publishing /cmd_vel (Twist)");
  }

};

int main(int argc, char ** argv){
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardToCmdVel>());
  rclcpp::shutdown();
  return 0;
}