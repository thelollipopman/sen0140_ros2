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

enum class OversamplingSetting : uint8_t
{
    ULTRA_LOW_POWER = 0,
    LOW_POWER = 1,
    STANDARD_RESOLUTION = 2,
    HIGH_RESOLUTION = 3,
    ULTRA_HIGH_RESOLUTION = 4
};

class Bmp280
{
public:
    Bmp280(
        const std::string & i2c_device,
        uint8_t address = 0x77);

    ~Bmp280();

    void initialize(
        OversamplingSetting oversampling_setting);

    BarometerData read();

private:
    int fd_;

    OversamplingSetting oversampling_setting_;

    uint16_t dig_T1_;
    int16_t  dig_T2_;
    int16_t  dig_T3_;

    uint16_t dig_P1_;
    int16_t  dig_P2_;
    int16_t  dig_P3_;
    int16_t  dig_P4_;
    int16_t  dig_P5_;
    int16_t  dig_P6_;
    int16_t  dig_P7_;
    int16_t  dig_P8_;
    int16_t  dig_P9_;

    int32_t t_fine_;

    void write_register(
        uint8_t reg,
        uint8_t value);

    void read_registers(
        uint8_t start_reg,
        uint8_t * buffer,
        std::size_t length);

    uint16_t read_u16_le(uint8_t reg);
    int16_t read_s16_le(uint8_t reg);

    void read_calibration();

    bool is_nvm_updating();

    uint8_t make_ctrl_meas() const;

    void read_raw(
        int32_t & adc_pressure,
        int32_t & adc_temperature);

    double compensate_temperature(
        int32_t adc_temperature);

    double compensate_pressure(
        int32_t adc_pressure);
};

}