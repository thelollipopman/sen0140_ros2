#include <chrono>
#include <memory>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sen0140_ros2/adxl345.hpp"
#include "sen0140_ros2/itg3200.hpp"
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

        declare_parameter<int>("mag.address", 0x0D);
        declare_parameter<std::string>("mag.frame_id", "imu_link");
        declare_parameter<double>("mag.publish_rate", 50.0);
        declare_parameter<double>("mag.output_data_rate", 50.0);
        
        declare_parameter<int>("baro.address", 0x77);
        declare_parameter<std::string>("baro.frame_id", "imu_link");
        declare_parameter<int>("baro.oversampling_setting", 4);
        declare_parameter<double>("baro.publish_rate", 10.0);
        
        declare_parameter<std::string>("imu.frame_id","imu_link");
        declare_parameter<double>("imu.publish_rate", 100.0);


        declare_parameter<int>("accel.address", 0x53);
        declare_parameter<double>("accel.output_data_rate", 200.0);
        declare_parameter<int>("accel.range", 3);

        declare_parameter<int>("gyro.address", 0x68);
        declare_parameter<double>("gyro.output_data_rate", 200.0);
        declare_parameter<int>("gyro.dlpf_cfg", 2);
        
        

        const auto i2c_device = get_parameter("i2c_device").as_string();


        // Magnetometer VCM5883L publisher

        const int mag_address =
            get_parameter("mag.address").as_int();

        const double mag_output_data_rate = 
            get_parameter("mag.output_data_rate").as_double();

        const double mag_publish_rate = 
            get_parameter("mag.publish_rate").as_double();

        mag_frame_id_ = get_parameter("mag.frame_id").as_string();

        magnetometer_ =
            std::make_unique<sen0140_ros2::Vcm5883l>(
                i2c_device, static_cast<uint8_t>(mag_address));

        magnetometer_->initialize(mag_output_data_rate);

        mag_publisher_ =
            create_publisher<sensor_msgs::msg::MagneticField>(
                "magnetic_field",
                rclcpp::SensorDataQoS());

        const auto mag_period =
            std::chrono::duration<double>(1.0 / mag_publish_rate);

        mag_timer_ = create_wall_timer(
            mag_period,
            std::bind(&Sen0140Node::read_magnetometer, this));

        // Barometer BMP280 publisher

        const int baro_address =
            get_parameter("baro.address").as_int();

        const int baro_oversampling_setting =
            get_parameter(
                "baro.oversampling_setting").as_int();

        if (baro_oversampling_setting < 0 ||
            baro_oversampling_setting > 4)
        {
            throw std::invalid_argument(
                "baro.oversampling_setting must be between 0 and 4");
        }

        const double baro_publish_rate =
            get_parameter("baro.publish_rate").as_double();
        
        if (baro_publish_rate <= 0.0) {
            throw std::invalid_argument(
                "baro.publish_rate must be greater than 0");
        }

        baro_frame_id_ =
            get_parameter(
                "baro.frame_id").as_string();

        barometer_ =
            std::make_unique<
                sen0140_ros2::Bmp280>(
                    i2c_device,
                    static_cast<uint8_t>(
                        baro_address));

        barometer_->initialize(
            static_cast<
                sen0140_ros2::OversamplingSetting>(
                    baro_oversampling_setting));

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

        const auto baro_period =
            std::chrono::duration<double>(
                1.0 / baro_publish_rate);

        baro_timer_ =
            create_wall_timer(
                baro_period,
                std::bind(
                    &Sen0140Node::read_barometer,
                    this));
        
        // Accelerometer ADXL345 + gyroscope ITG-3200 publishers
        const int accel_address =
            get_parameter("accel.address").as_int();

        const double accel_output_data_rate =
            get_parameter("accel.output_data_rate").as_double();

        const int accel_range =
            get_parameter("accel.range").as_int();

        const int gyro_address =
            get_parameter("gyro.address").as_int();

        const double gyro_output_data_rate =
            get_parameter("gyro.output_data_rate").as_double();

        const int gyro_dlpf_cfg =
            get_parameter("gyro.dlpf_cfg").as_int();

        imu_frame_id_ =
            get_parameter("imu.frame_id").as_string();
        
        const double imu_publish_rate = 
            get_parameter("imu.publish_rate").as_double();

        if (accel_range < 0 || accel_range > 3) {
            throw std::invalid_argument(
                "accel.range must be between 0 and 3");
        }

        if (gyro_dlpf_cfg < 0 || gyro_dlpf_cfg > 6) {
            throw std::invalid_argument(
                "gyro.dlpf_cfg must be between 0 and 6");
        }

        accelerometer_ =
            std::make_unique<sen0140_ros2::Adxl345>(
                i2c_device, static_cast<uint8_t>(accel_address));

        gyroscope_ =
            std::make_unique<sen0140_ros2::Itg3200>(
                i2c_device, static_cast<uint8_t>(gyro_address));

        accelerometer_->initialize(
            accel_output_data_rate,
            static_cast<sen0140_ros2::AccelRange>(
                accel_range));

        gyroscope_->initialize(
            gyro_output_data_rate,
            static_cast<uint8_t>(
                gyro_dlpf_cfg));

        imu_publisher_ =
            create_publisher<sensor_msgs::msg::Imu>(
                "imu",
                rclcpp::SensorDataQoS());
        
        const auto imu_period =
            std::chrono::duration<double>(1.0 / imu_publish_rate);

        imu_timer_ =
            create_wall_timer(
                imu_period,
                std::bind(
                    &Sen0140Node::read_imu,
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
        const auto data =
            barometer_->read();

        const auto stamp = now();

        sensor_msgs::msg::FluidPressure pressure_msg;

        pressure_msg.header.stamp = stamp;
        pressure_msg.header.frame_id =
            baro_frame_id_;

        pressure_msg.fluid_pressure =
            data.pressure;

        sensor_msgs::msg::Temperature temperature_msg;

        temperature_msg.header.stamp = stamp;
        temperature_msg.header.frame_id =
            baro_frame_id_;

        temperature_msg.temperature =
            data.temperature;

        pressure_publisher_->publish(
            pressure_msg);

        temperature_publisher_->publish(
            temperature_msg);
    }

    void read_imu()
    {
        if (!accelerometer_->is_data_ready() ||
            !gyroscope_->is_data_ready())
        {
            return;
        }

        /*
        * Neither sensor provides a usable measurement
        * timestamp over I2C, so this is an approximation
        * of the common measurement time.
        */
        const auto stamp = now();

        const auto acceleration =
            accelerometer_->read();

        const auto angular_velocity =
            gyroscope_->read();

        sensor_msgs::msg::Imu msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = imu_frame_id_;

        msg.linear_acceleration.x =
            acceleration.x;

        msg.linear_acceleration.y =
            acceleration.y;

        msg.linear_acceleration.z =
            acceleration.z;

        msg.angular_velocity.x =
            angular_velocity.x;

        msg.angular_velocity.y =
            angular_velocity.y;

        msg.angular_velocity.z =
            angular_velocity.z;

        /*
        * This driver does not estimate orientation.
        * ROS convention is covariance[0] = -1 when
        * orientation is unavailable.
        */
        msg.orientation_covariance[0] = -1.0;

        imu_publisher_->publish(msg);
    }

    // Magnetometer
    std::unique_ptr<sen0140_ros2::Vcm5883l> magnetometer_;

    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;

    rclcpp::TimerBase::SharedPtr mag_timer_;

    std::string mag_frame_id_;


    // Barometer
    std::unique_ptr<sen0140_ros2::Bmp280> barometer_;

    rclcpp::Publisher<
        sensor_msgs::msg::FluidPressure>::SharedPtr
        pressure_publisher_;

    rclcpp::Publisher<
        sensor_msgs::msg::Temperature>::SharedPtr
        temperature_publisher_;

    rclcpp::TimerBase::SharedPtr baro_timer_;

    std::string baro_frame_id_;

    // IMU: Accel + Gyro
    std::unique_ptr<
    sen0140_ros2::Adxl345>
    accelerometer_;

    std::unique_ptr<
        sen0140_ros2::Itg3200>
        gyroscope_;

    rclcpp::Publisher<
        sensor_msgs::msg::Imu>::SharedPtr
        imu_publisher_;

    rclcpp::TimerBase::SharedPtr
        imu_timer_;

    std::string imu_frame_id_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sen0140Node>());
    rclcpp::shutdown();

    return 0;
}