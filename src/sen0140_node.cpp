#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "sen0140_ros2/vcm5883l.hpp"

using namespace std::chrono_literals;

constexpr double LSB_PER_GAUSS = 3000.0;
constexpr double TESLA_PER_GAUSS = 1e-4;

class Sen0140Node : public rclcpp::Node
{
public:
    Sen0140Node()
    : Node("sen0140_node")
    {
        declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
        declare_parameter<int>("mag.odr", 50);
        declare_parameter<std::string>("mag.frame_id", "imu_link");

        const auto i2c_device =
            get_parameter("i2c_device").as_string();

        const int odr =
            get_parameter("mag.odr").as_int();

        frame_id_ =
            get_parameter("mag.frame_id").as_string();

        magnetometer_ =
            std::make_unique<sen0140_ros2::Vcm5883l>(
                i2c_device);

        magnetometer_->initialize(odr);

        publisher_ =
            create_publisher<sensor_msgs::msg::MagneticField>(
                "magnetic_field",
                rclcpp::SensorDataQoS());

        const auto period =
            std::chrono::duration<double>(1.0 / odr);

        timer_ = create_wall_timer(
            period,
            std::bind(&Sen0140Node::read_magnetometer, this));
    }

private:
    void read_magnetometer()
    {
        const auto field = magnetometer_->read();

        sensor_msgs::msg::MagneticField msg;

        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        
        msg.magnetic_field.x = field.x;
        msg.magnetic_field.y = field.y;
        msg.magnetic_field.z = field.z;

        publisher_->publish(msg);
    }

    std::unique_ptr<sen0140_ros2::Vcm5883l> magnetometer_;

    rclcpp::Publisher<
        sensor_msgs::msg::MagneticField>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sen0140Node>());
    rclcpp::shutdown();

    return 0;
}