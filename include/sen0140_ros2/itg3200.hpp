#ifndef SEN0140_ROS2__ITG3200_HPP_
#define SEN0140_ROS2__ITG3200_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace sen0140_ros2
{

struct AngularVelocity
{
    double x;
    double y;
    double z;
};

class Itg3200
{
public:
    explicit Itg3200(
        const std::string & i2c_device,
        uint8_t address = 0x68);

    ~Itg3200();

    void initialize(
        uint8_t sample_rate_divider,
        uint8_t dlpf_cfg);

    bool is_data_ready();

    AngularVelocity read();

    double read_temperature();

private:
    void configure(
        uint8_t sample_rate_divider,
        uint8_t dlpf_cfg);

    void write_register(
        uint8_t reg,
        uint8_t value);

    uint8_t read_register(
        uint8_t reg);

    void read_registers(
        uint8_t start_reg,
        uint8_t * buffer,
        std::size_t length);

    int fd_;
};

}  // namespace sen0140_ros2

#endif  // SEN0140_ROS2__ITG3200_HPP_