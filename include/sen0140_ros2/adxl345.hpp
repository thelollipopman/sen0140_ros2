#ifndef SEN0140_ROS2__ADXL345_HPP_
#define SEN0140_ROS2__ADXL345_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace sen0140_ros2
{

struct Acceleration
{
    double x;
    double y;
    double z;
};

enum class AccelRange : uint8_t
{
    G2  = 0,
    G4  = 1,
    G8  = 2,
    G16 = 3
};

class Adxl345
{
public:
    explicit Adxl345(
        const std::string & i2c_device,
        uint8_t address = 0x53);

    ~Adxl345();

    void initialize(
        double odr_hz,
        AccelRange range);

    bool is_data_ready();

    Acceleration read();

private:
    void configure(
        double odr_hz,
        AccelRange range);

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

#endif  // SEN0140_ROS2__ADXL345_HPP_