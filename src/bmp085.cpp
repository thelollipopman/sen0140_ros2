#include "sen0140_ros2/bmp085.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace sen0140_ros2
{

Bmp085::Bmp085(
    const std::string & i2c_device,
    uint8_t address)
: fd_(-1),
  oversampling_(0)
{
    fd_ = open(i2c_device.c_str(), O_RDWR);

    if (fd_ < 0) {
        throw std::runtime_error(
            "Failed to open I2C device");
    }

    if (ioctl(fd_, I2C_SLAVE, address) < 0) {
        close(fd_);
        fd_ = -1;

        throw std::runtime_error(
            "Failed to select BMP085 I2C address");
    }
}


Bmp085::~Bmp085()
{
    if (fd_ >= 0) {
        close(fd_);
    }
}


void Bmp085::initialize(int oversampling)
{
    if (oversampling < 0 || oversampling > 3) {
        throw std::invalid_argument(
            "BMP085 oversampling must be between 0 and 3");
    }

    oversampling_ = oversampling;

    read_calibration();
}


void Bmp085::write_register(
    uint8_t reg,
    uint8_t value)
{
    uint8_t buffer[2] = {
        reg,
        value
    };

    const ssize_t bytes_written =
        write(fd_, buffer, 2);

    if (bytes_written != 2) {
        throw std::runtime_error(
            "Failed to write BMP085 register");
    }
}


void Bmp085::read_registers(
    uint8_t start_reg,
    uint8_t * buffer,
    std::size_t length)
{
    const ssize_t bytes_written =
        write(fd_, &start_reg, 1);

    if (bytes_written != 1) {
        throw std::runtime_error(
            "Failed to select BMP085 register");
    }

    const ssize_t bytes_read =
        read(fd_, buffer, length);

    if (bytes_read !=
        static_cast<ssize_t>(length))
    {
        throw std::runtime_error(
            "Failed to read BMP085 registers");
    }
}


int16_t Bmp085::read_s16(uint8_t reg)
{
    uint8_t data[2];

    read_registers(reg, data, 2);

    const uint16_t raw =
        (static_cast<uint16_t>(data[0]) << 8) |
        static_cast<uint16_t>(data[1]);

    return static_cast<int16_t>(raw);
}


uint16_t Bmp085::read_u16(uint8_t reg)
{
    uint8_t data[2];

    read_registers(reg, data, 2);

    return
        (static_cast<uint16_t>(data[0]) << 8) |
        static_cast<uint16_t>(data[1]);
}


void Bmp085::read_calibration()
{
    ac1_ = read_s16(0xAA);
    ac2_ = read_s16(0xAC);
    ac3_ = read_s16(0xAE);

    ac4_ = read_u16(0xB0);
    ac5_ = read_u16(0xB2);
    ac6_ = read_u16(0xB4);

    b1_ = read_s16(0xB6);
    b2_ = read_s16(0xB8);

    mb_ = read_s16(0xBA);
    mc_ = read_s16(0xBC);
    md_ = read_s16(0xBE);
}


int32_t Bmp085::read_uncompensated_temperature()
{
    // Start temperature conversion.
    write_register(0xF4, 0x2E);

    // Maximum temperature conversion time is 4.5 ms.
    std::this_thread::sleep_for(
        std::chrono::milliseconds(5));

    uint8_t data[2];

    read_registers(0xF6, data, 2);

    return
        (static_cast<int32_t>(data[0]) << 8) |
        static_cast<int32_t>(data[1]);
}


int32_t Bmp085::read_uncompensated_pressure()
{
    const uint8_t command =
        static_cast<uint8_t>(
            0x34 + (oversampling_ << 6));

    write_register(0xF4, command);

    switch (oversampling_) {
        case 0:
            std::this_thread::sleep_for(
                std::chrono::milliseconds(5));
            break;

        case 1:
            std::this_thread::sleep_for(
                std::chrono::milliseconds(8));
            break;

        case 2:
            std::this_thread::sleep_for(
                std::chrono::milliseconds(14));
            break;

        case 3:
            std::this_thread::sleep_for(
                std::chrono::milliseconds(26));
            break;
    }

    uint8_t data[3];

    read_registers(0xF6, data, 3);

    const int32_t raw =
        (static_cast<int32_t>(data[0]) << 16) |
        (static_cast<int32_t>(data[1]) << 8) |
        static_cast<int32_t>(data[2]);

    return raw >> (8 - oversampling_);
}


BarometerData Bmp085::compensate(
    int32_t ut,
    int32_t up)
{
    // Temperature compensation

    const int32_t x1_temp =
        ((ut - static_cast<int32_t>(ac6_)) *
         static_cast<int32_t>(ac5_)) >> 15;

    const int32_t denominator =
        x1_temp + static_cast<int32_t>(md_);

    if (denominator == 0) {
        throw std::runtime_error(
            "BMP085 temperature compensation division by zero");
    }

    const int32_t x2_temp =
        (static_cast<int32_t>(mc_) << 11) /
        denominator;

    const int32_t b5 =
        x1_temp + x2_temp;

    // Temperature is in 0.1 degrees Celsius.
    const int32_t temperature_raw =
        (b5 + 8) >> 4;


    // Pressure compensation

    const int32_t b6 =
        b5 - 4000;

    int32_t x1 =
        (static_cast<int32_t>(b2_) *
         ((b6 * b6) >> 12)) >> 11;

    int32_t x2 =
        (static_cast<int32_t>(ac2_) *
         b6) >> 11;

    int32_t x3 =
        x1 + x2;

    const int32_t b3 =
        (((static_cast<int32_t>(ac1_) * 4 + x3)
          << oversampling_) + 2) >> 2;

    x1 =
        (static_cast<int32_t>(ac3_) *
         b6) >> 13;

    x2 =
        (static_cast<int32_t>(b1_) *
         ((b6 * b6) >> 12)) >> 16;

    x3 =
        ((x1 + x2) + 2) >> 2;

    const uint32_t b4 =
        (static_cast<uint32_t>(ac4_) *
         static_cast<uint32_t>(x3 + 32768))
        >> 15;

    if (b4 == 0) {
        throw std::runtime_error(
            "BMP085 pressure compensation division by zero");
    }

    const uint32_t b7 =
        static_cast<uint32_t>(
            up - b3) *
        static_cast<uint32_t>(
            50000 >> oversampling_);

    int32_t pressure;

    if (b7 < 0x80000000U) {
        pressure =
            static_cast<int32_t>(
                (b7 * 2U) / b4);
    } else {
        pressure =
            static_cast<int32_t>(
                (b7 / b4) * 2U);
    }

    x1 =
        (pressure >> 8) *
        (pressure >> 8);

    x1 =
        (x1 * 3038) >> 16;

    x2 =
        (-7357 * pressure) >> 16;

    pressure +=
        (x1 + x2 + 3791) >> 4;

    return BarometerData{
        static_cast<double>(pressure),
        static_cast<double>(temperature_raw) / 10.0
    };
}


BarometerData Bmp085::read()
{
    const int32_t ut =
        read_uncompensated_temperature();

    const int32_t up =
        read_uncompensated_pressure();

    return compensate(ut, up);
}

}  // namespace sen0140_ros2