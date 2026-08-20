#pragma once

#include <cstdint>
#include <string>
#include <cstddef>

namespace sen0140_ros2
{

struct MagneticField
{
    double x;
    double y;
    double z;
};

class Vcm5883l
{
public:
    Vcm5883l(const std::string & i2c_device, uint8_t address = 0x0C);
    ~Vcm5883l();

    void initialize(uint8_t odr_hz);
    void configure(uint8_t odr_hz);

    MagneticField read();

private:
    int fd_;

    void write_register(uint8_t reg, uint8_t value);
    uint8_t read_register(uint8_t reg);
    void read_registers(uint8_t start_reg, uint8_t * buffer, std::size_t length);
};

}  // namespace sen0140_ros2