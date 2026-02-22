#include <memory>
#include <rclcpp/rclcpp.hpp>
//#include <std_msgs/msg/int32.hpp>
//#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

//Permettra le traitement de données IMU
class ImuReading : public rclcpp::Node{

  private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        /**RCLCPP_INFO(get_logger(), "Received IMU data:\n orientation=(%f, %f, %f, %f),\n angular_velocity=(%f, %f, %f),\n linear_acceleration=(%f, %f, %f)",
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        **/
       (void)msg; // Pour éviter les warnings de variable non utilisée
        }


public:
  ImuReading() : Node("imu_reading"){

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10,
      std::bind(&ImuReading::onImu, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening /imu (sensor_msgs/Imu)");
  }

};

int main(int argc, char ** argv){
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuReading>());
  rclcpp::shutdown();
  return 0;
}