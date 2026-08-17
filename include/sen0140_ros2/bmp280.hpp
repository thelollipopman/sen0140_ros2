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

enum class Oversampling : uint8_t
{
    SKIP = 0,
    X1   = 1,
    X2   = 2,
    X4   = 3,
    X8   = 4,
    X16  = 5
};

class Bmp280
{
public:
    Bmp280(
        const std::string & i2c_device,
        uint8_t address = 0x77);

    ~Bmp280();

    void initialize(
        Oversampling temp_oversampling,
        Oversampling pressure_oversampling);

    void start_measurement();

    bool is_measuring();

    bool is_nvm_updating();

    BarometerData read_measurement();

private:
    int fd_;
    Oversampling temperature_oversampling_;
    Oversampling pressure_oversampling_;

    // Factory calibration coefficients
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
    double compensate_temperature(int32_t adc_temperature);
    double compensate_pressure(int32_t adc_pressure);

    void write_register(uint8_t reg, uint8_t value);

    void read_registers(
        uint8_t start_reg,
        uint8_t * buffer,
        std::size_t length);

    void read_calibration();

    int16_t read_s16_le(uint8_t reg);
    uint16_t read_u16_le(uint8_t reg);
    uint8_t make_ctrl_meas(uint8_t mode) const;
    void read_raw(int32_t & adc_pressure, int32_t & adc_temperature);
};

}  // namespace sen0140_ros2