#include "sen0140_ros2/adxl345.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <stdexcept>

namespace sen0140_ros2
{

namespace
{

constexpr uint8_t REG_DEVID       = 0x00;
constexpr uint8_t REG_BW_RATE     = 0x2C;
constexpr uint8_t REG_POWER_CTL   = 0x2D;
constexpr uint8_t REG_INT_SOURCE  = 0x30;
constexpr uint8_t REG_DATA_FORMAT = 0x31;
constexpr uint8_t REG_DATAX0      = 0x32;

constexpr uint8_t EXPECTED_DEVICE_ID = 0xE5;

constexpr double GRAVITY = 9.80665;

// Full-resolution mode has approximately 3.9 mg/LSB.
constexpr double SCALE_MPS2 =
    0.0039 * GRAVITY;

}  // namespace


Adxl345::Adxl345(
    const std::string & i2c_device,
    uint8_t address)
: fd_(-1)
{
    fd_ = ::open(
        i2c_device.c_str(),
        O_RDWR);

    if (fd_ < 0) {
        throw std::runtime_error(
            "Failed to open I2C device");
    }

    if (::ioctl(
            fd_,
            I2C_SLAVE,
            address) < 0)
    {
        ::close(fd_);
        fd_ = -1;

        throw std::runtime_error(
            "Failed to select ADXL345 I2C address");
    }
}


Adxl345::~Adxl345()
{
    if (fd_ >= 0) {
        ::close(fd_);
    }
}


void Adxl345::write_register(
    uint8_t reg,
    uint8_t value)
{
    const uint8_t buffer[2] = {
        reg,
        value
    };

    if (::write(
            fd_,
            buffer,
            2) != 2)
    {
        throw std::runtime_error(
            "Failed to write ADXL345 register");
    }
}


uint8_t Adxl345::read_register(
    uint8_t reg)
{
    uint8_t value;

    read_registers(
        reg,
        &value,
        1);

    return value;
}


void Adxl345::read_registers(
    uint8_t start_reg,
    uint8_t * buffer,
    std::size_t length)
{
    if (::write(
            fd_,
            &start_reg,
            1) != 1)
    {
        throw std::runtime_error(
            "Failed to select ADXL345 register");
    }

    if (::read(
            fd_,
            buffer,
            length) !=
        static_cast<ssize_t>(length))
    {
        throw std::runtime_error(
            "Failed to read ADXL345 registers");
    }
}


void Adxl345::initialize(
    double odr_hz,
    AccelRange range)
{
    const uint8_t device_id =
        read_register(REG_DEVID);

    if (device_id != EXPECTED_DEVICE_ID) {
        throw std::runtime_error(
            "Unexpected ADXL345 device ID");
    }

    configure(
        odr_hz,
        range);
}


void Adxl345::configure(
    double odr_hz,
    AccelRange range)
{
    uint8_t rate_code;

    uint8_t rate_code;

    if (odr_hz == 3200.0) {
        rate_code = 0x0F;
    }
    else if (odr_hz == 1600.0) {
        rate_code = 0x0E;
    }
    else if (odr_hz == 800.0) {
        rate_code = 0x0D;
    }
    else if (odr_hz == 400.0) {
        rate_code = 0x0C;
    }
    else if (odr_hz == 200.0) {
        rate_code = 0x0B;
    }
    else if (odr_hz == 100.0) {
        rate_code = 0x0A;
    }
    else if (odr_hz == 50.0) {
        rate_code = 0x09;
    }
    else if (odr_hz == 25.0) {
        rate_code = 0x08;
    }
    else if (odr_hz == 12.5) {
        rate_code = 0x07;
    }
    else if (odr_hz == 6.25) {
        rate_code = 0x06;
    }
    else if (odr_hz == 3.13) {
        rate_code = 0x05;
    }
    else if (odr_hz == 1.56) {
        rate_code = 0x04;
    }
    else if (odr_hz == 0.78) {
        rate_code = 0x03;
    }
    else if (odr_hz == 0.39) {
        rate_code = 0x02;
    }
    else if (odr_hz == 0.20) {
        rate_code = 0x01;
    }
    else if (odr_hz == 0.10) {
        rate_code = 0x00;
    }
    else {
        throw std::invalid_argument(
            "Unsupported ADXL345 output data rate");
    }

    // Place device in standby while configuring.
    write_register(
        REG_POWER_CTL,
        0x00);

    write_register(
        REG_BW_RATE,
        rate_code);

    // FULL_RES = 1
    // right justified
    // range = bits [1:0]
    const uint8_t data_format =
        0x08 |
        static_cast<uint8_t>(range);

    write_register(
        REG_DATA_FORMAT,
        data_format);

    // MEASURE = 1
    write_register(
        REG_POWER_CTL,
        0x08);
}


bool Adxl345::is_data_ready()
{
    const uint8_t source =
        read_register(REG_INT_SOURCE);

    return (source & 0x80) != 0;
}


Acceleration Adxl345::read()
{
    uint8_t data[6];

    read_registers(
        REG_DATAX0,
        data,
        6);

    const int16_t raw_x =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[0]) |
            (static_cast<uint16_t>(data[1]) << 8));

    const int16_t raw_y =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[2]) |
            (static_cast<uint16_t>(data[3]) << 8));

    const int16_t raw_z =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[4]) |
            (static_cast<uint16_t>(data[5]) << 8));

    return {
        static_cast<double>(raw_x) * SCALE_MPS2,
        static_cast<double>(raw_y) * SCALE_MPS2,
        static_cast<double>(raw_z) * SCALE_MPS2
    };
}

}  // namespace sen0140_ros2