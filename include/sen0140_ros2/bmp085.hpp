#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace sen0140_ros2
{

struct BarometerData
{
    double pressure;     // Pa
    double temperature;  // deg C
};

class Bmp085
{
public:
    Bmp085(
        const std::string & i2c_device,
        uint8_t address = 0x77);

    ~Bmp085();

    void initialize(int oversampling);

    BarometerData read();

private:
    int fd_;
    int oversampling_;

    // Factory calibration coefficients
    int16_t ac1_;
    int16_t ac2_;
    int16_t ac3_;
    uint16_t ac4_;
    uint16_t ac5_;
    uint16_t ac6_;
    int16_t b1_;
    int16_t b2_;
    int16_t mb_;
    int16_t mc_;
    int16_t md_;

    void write_register(uint8_t reg, uint8_t value);

    void read_registers(
        uint8_t start_reg,
        uint8_t * buffer,
        std::size_t length);

    void read_calibration();

    int32_t read_uncompensated_temperature();
    int32_t read_uncompensated_pressure();
    int16_t read_s16(uint8_t reg);
    uint16_t read_u16(uint8_t reg);

    BarometerData compensate(
        int32_t ut,
        int32_t up);
};

}  // namespace sen0140_ros2