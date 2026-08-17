#include <chrono>
#include <memory>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "sen0140_ros2/bmp280.hpp"
#include "sen0140_ros2/vcm5883l.hpp"

using namespace std::chrono_literals;

class Sen0140Node : public rclcpp::Node
{
public:
    Sen0140Node()
    : Node("sen0140_node")
    {
        declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
        declare_parameter<int>("mag.odr", 50);
        declare_parameter<std::string>("mag.frame_id", "imu_link");
        declare_parameter<int>("baro.temp_oversampling", 2);
        declare_parameter<int>("baro.pressure_oversampling", 5);
        declare_parameter<double>("baro.rate", 10.0);
        declare_parameter<std::string>("baro.frame_id", "imu_link");

        // Magnetometer VCM5883L publisher
        const auto i2c_device = get_parameter("i2c_device").as_string();
        const int mag_odr = get_parameter("mag.odr").as_int();

        mag_frame_id_ = get_parameter("mag.frame_id").as_string();

        magnetometer_ =
            std::make_unique<sen0140_ros2::Vcm5883l>(
                i2c_device);

        magnetometer_->initialize(mag_odr);
        mag_publisher_ =
            create_publisher<sensor_msgs::msg::MagneticField>(
                "magnetic_field",
                rclcpp::SensorDataQoS());
        const auto mag_period =
            std::chrono::duration<double>(1.0 / mag_odr);

        mag_timer_ = create_wall_timer(
            mag_period,
            std::bind(&Sen0140Node::read_magnetometer, this));

        // Barometer BMP280 publisher
        const int baro_temp_oversampling =
            get_parameter(
                "baro.temp_oversampling").as_int();

        const int baro_pressure_oversampling =
            get_parameter(
                "baro.pressure_oversampling").as_int();

        baro_rate_ =
            get_parameter("baro.rate").as_double();

        baro_frame_id_ =
            get_parameter(
                "baro.frame_id").as_string();

        if (baro_temp_oversampling < 0 ||
            baro_temp_oversampling > 5 ||
            baro_pressure_oversampling < 0 ||
            baro_pressure_oversampling > 5)
        {
            throw std::invalid_argument(
                "BMP280 oversampling must be between 0 and 5");
        }

        if (baro_rate_ <= 0.0) {
            throw std::invalid_argument(
                "baro.rate must be greater than 0");
        }

        barometer_ =
            std::make_unique<
                sen0140_ros2::Bmp280>(
                    i2c_device);

        barometer_->initialize(
            static_cast<
                sen0140_ros2::Oversampling>(
                    baro_temp_oversampling),
            static_cast<
                sen0140_ros2::Oversampling>(
                    baro_pressure_oversampling));

        pressure_publisher_ =
            create_publisher<
                sensor_msgs::msg::FluidPressure>(
                    "pressure",
                    rclcpp::SensorDataQoS());

        temperature_publisher_ =
            create_publisher<
                sensor_msgs::msg::Temperature>(
                    "temperature",
                    rclcpp::SensorDataQoS());

        last_baro_start_ = now();

        baro_timer_ =
            create_wall_timer(
                5ms,
                std::bind(
                    &Sen0140Node::read_barometer,
                    this));
    }

    

private:
    void read_magnetometer()
    {
        const auto field = magnetometer_->read();

        sensor_msgs::msg::MagneticField msg;

        msg.header.stamp = now();
        msg.header.frame_id = mag_frame_id_;
        
        msg.magnetic_field.x = field.x;
        msg.magnetic_field.y = field.y;
        msg.magnetic_field.z = field.z;

        mag_publisher_->publish(msg);
    }

    
    void read_barometer()
    {
        if (!baro_measurement_pending_) {
            const double elapsed =
                (now() - last_baro_start_).seconds();

            if (elapsed < 1.0 / baro_rate_) {
                return;
            }

            barometer_->start_measurement();

            last_baro_start_ = now();
            baro_measurement_pending_ = true;

            return;
        }

        if (barometer_->is_measuring()) {
            return;
        }

        const auto data =
            barometer_->read_measurement();

        baro_measurement_pending_ = false;

        const auto stamp = now();

        sensor_msgs::msg::FluidPressure pressure_msg;
        pressure_msg.header.stamp = stamp;
        pressure_msg.header.frame_id = baro_frame_id_;
        pressure_msg.fluid_pressure = data.pressure;

        sensor_msgs::msg::Temperature temperature_msg;
        temperature_msg.header.stamp = stamp;
        temperature_msg.header.frame_id = baro_frame_id_;
        temperature_msg.temperature = data.temperature;

        pressure_publisher_->publish(pressure_msg);
        temperature_publisher_->publish(temperature_msg);
    }

    std::unique_ptr<sen0140_ros2::Vcm5883l> magnetometer_;

    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;

    rclcpp::TimerBase::SharedPtr mag_timer_;

    std::string mag_frame_id_;
    
    std::unique_ptr<sen0140_ros2::Bmp280> barometer_;

    rclcpp::Publisher<
        sensor_msgs::msg::FluidPressure>::SharedPtr
        pressure_publisher_;

    rclcpp::Publisher<
        sensor_msgs::msg::Temperature>::SharedPtr
        temperature_publisher_;

    rclcpp::TimerBase::SharedPtr baro_timer_;

    std::string baro_frame_id_;

    bool baro_measurement_pending_ = false;

    double baro_rate_;

    rclcpp::Time last_baro_start_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sen0140Node>());
    rclcpp::shutdown();

    return 0;
}