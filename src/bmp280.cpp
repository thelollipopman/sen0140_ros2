#include "sen0140_ros2/bmp280.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>




namespace sen0140_ros2
{

Bmp280::Bmp280(const std::string & i2c_device, uint8_t address): fd_(-1)
{
    fd_ = ::open(i2c_device.c_str(), O_RDWR);

    if (fd_ < 0) {
        throw std::runtime_error(
            "Failed to open I2C device");
    }

    if (ioctl(fd_, I2C_SLAVE, address) < 0) {
        ::close(fd_);
        fd_ = -1;

        throw std::runtime_error(
            "Failed to select BMP280 I2C address");
    }
}


Bmp280::~Bmp280()
{
    if (fd_ >= 0) {
        ::close(fd_);
    }
}


void Bmp280::initialize(
    OversamplingSetting oversampling_setting)
{
    uint8_t id;
    read_registers(0xD0, &id, 1);

    if (id != 0x58) {
        throw std::runtime_error(
            "Unexpected BMP280 chip ID");
    }

    while (is_nvm_updating()) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }

    read_calibration();

    oversampling_setting_ =
        oversampling_setting;

    // t_standby = 0.5 ms
    // IIR filter off
    // SPI 3-wire disabled
    write_register(0xF5, 0x00);

    // Start continuous normal-mode operation.
    write_register(
        0xF4,
        make_ctrl_meas());
}


void Bmp280::write_register(
    uint8_t reg,
    uint8_t value)
{
    uint8_t buffer[2] = {
        reg,
        value
    };

    const ssize_t bytes_written =
        ::write(fd_, buffer, 2);

    if (bytes_written != 2) {
        throw std::runtime_error(
            "Failed to write BMP280 register");
    }
}


void Bmp280::read_registers(
    uint8_t start_reg,
    uint8_t * buffer,
    std::size_t length)
{
    const ssize_t bytes_written =
        ::write(fd_, &start_reg, 1);

    if (bytes_written != 1) {
        throw std::runtime_error(
            "Failed to select BMP280 register");
    }

    const ssize_t bytes_read =
        ::read(fd_, buffer, length);

    if (bytes_read !=
        static_cast<ssize_t>(length))
    {
        throw std::runtime_error(
            "Failed to read BMP280 registers");
    }
}


uint16_t Bmp280::read_u16_le(uint8_t reg)
{
    uint8_t data[2];
    read_registers(reg, data, 2);

    return
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);
}


int16_t Bmp280::read_s16_le(uint8_t reg)
{
    return static_cast<int16_t>(
        read_u16_le(reg));
}


void Bmp280::read_calibration()
{
    dig_T1_ = read_u16_le(0x88);
    dig_T2_ = read_s16_le(0x8A);
    dig_T3_ = read_s16_le(0x8C);

    dig_P1_ = read_u16_le(0x8E);
    dig_P2_ = read_s16_le(0x90);
    dig_P3_ = read_s16_le(0x92);
    dig_P4_ = read_s16_le(0x94);
    dig_P5_ = read_s16_le(0x96);
    dig_P6_ = read_s16_le(0x98);
    dig_P7_ = read_s16_le(0x9A);
    dig_P8_ = read_s16_le(0x9C);
    dig_P9_ = read_s16_le(0x9E);
}

double Bmp280::compensate_temperature(
    int32_t adc_temperature)
{
    const double var1 =
        (static_cast<double>(adc_temperature) / 16384.0 -
         static_cast<double>(dig_T1_) / 1024.0) *
        static_cast<double>(dig_T2_);

    const double temp =
        static_cast<double>(adc_temperature) / 131072.0 -
        static_cast<double>(dig_T1_) / 8192.0;

    const double var2 =
        temp * temp *
        static_cast<double>(dig_T3_);

    t_fine_ =
        static_cast<int32_t>(var1 + var2);

    return (var1 + var2) / 5120.0;
}

double Bmp280::compensate_pressure(
    int32_t adc_pressure)
{
    double var1 =
        static_cast<double>(t_fine_) / 2.0 -
        64000.0;

    double var2 =
        var1 * var1 *
        static_cast<double>(dig_P6_) /
        32768.0;

    var2 +=
        var1 *
        static_cast<double>(dig_P5_) *
        2.0;

    var2 =
        var2 / 4.0 +
        static_cast<double>(dig_P4_) *
        65536.0;

    var1 =
        (static_cast<double>(dig_P3_) *
         var1 * var1 / 524288.0 +
         static_cast<double>(dig_P2_) *
         var1) /
        524288.0;

    var1 =
        (1.0 + var1 / 32768.0) *
        static_cast<double>(dig_P1_);

    if (var1 == 0.0) {
        throw std::runtime_error(
            "BMP280 pressure compensation division by zero");
    }

    double pressure =
        1048576.0 -
        static_cast<double>(adc_pressure);

    pressure =
        (pressure - var2 / 4096.0) *
        6250.0 /
        var1;

    var1 =
        static_cast<double>(dig_P9_) *
        pressure *
        pressure /
        2147483648.0;

    var2 =
        pressure *
        static_cast<double>(dig_P8_) /
        32768.0;

    pressure +=
        (var1 +
         var2 +
         static_cast<double>(dig_P7_)) /
        16.0;

    return pressure;
}

uint8_t Bmp280::make_ctrl_meas() const
{
    uint8_t osrs_t;
    uint8_t osrs_p;

    switch (oversampling_setting_) {
        case OversamplingSetting::ULTRA_LOW_POWER:
            osrs_t = 1;  // x1
            osrs_p = 1;  // x1
            break;

        case OversamplingSetting::LOW_POWER:
            osrs_t = 1;  // x1
            osrs_p = 2;  // x2
            break;

        case OversamplingSetting::STANDARD_RESOLUTION:
            osrs_t = 1;  // x1
            osrs_p = 3;  // x4
            break;

        case OversamplingSetting::HIGH_RESOLUTION:
            osrs_t = 1;  // x1
            osrs_p = 4;  // x8
            break;

        case OversamplingSetting::ULTRA_HIGH_RESOLUTION:
            osrs_t = 2;  // x2
            osrs_p = 5;  // x16
            break;

        default:
            throw std::runtime_error(
                "Invalid BMP280 oversampling setting");
    }

    constexpr uint8_t NORMAL_MODE = 0x03;

    return
        (osrs_t << 5) |
        (osrs_p << 2) |
        NORMAL_MODE;
}



BarometerData Bmp280::read()
{
    int32_t adc_pressure;
    int32_t adc_temperature;

    read_raw(
        adc_pressure,
        adc_temperature);

    const double temperature =
        compensate_temperature(
            adc_temperature);

    const double pressure =
        compensate_pressure(
            adc_pressure);

    return {
        pressure,
        temperature
    };
}

bool Bmp280::is_nvm_updating()
{
    uint8_t status;
    read_registers(0xF3, &status, 1);

    return (status & 0x01) != 0;
}

void Bmp280::read_raw(
    int32_t & adc_pressure,
    int32_t & adc_temperature)
{
    uint8_t data[6];

    read_registers(0xF7, data, 6);

    adc_pressure =
        (static_cast<int32_t>(data[0]) << 12) |
        (static_cast<int32_t>(data[1]) << 4) |
        (static_cast<int32_t>(data[2]) >> 4);

    adc_temperature =
        (static_cast<int32_t>(data[3]) << 12) |
        (static_cast<int32_t>(data[4]) << 4) |
        (static_cast<int32_t>(data[5]) >> 4);
}

}  // namespace sen0140_ros2