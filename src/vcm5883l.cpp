#include "sen0140_ros2/vcm5883l.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace sen0140_ros2
{

constexpr uint8_t REG_X_LSB  = 0x00;
constexpr uint8_t REG_CONTROL_1 = 0x0B;
constexpr uint8_t REG_CONTROL_2 = 0x0A;
constexpr uint8_t REG_CHIP_ID = 0x0C;

constexpr uint8_t EXPECTED_CHIP_ID = 0x82;

constexpr double LSB_PER_GAUSS = 3000.0;
constexpr double TESLA_PER_GAUSS = 1e-4;
constexpr double TESLA_PER_LSB = TESLA_PER_GAUSS / LSB_PER_GAUSS;

Vcm5883l::Vcm5883l(
    const std::string & i2c_device,
    uint8_t address)
{
    fd_ = open(i2c_device.c_str(), O_RDWR);

    if (fd_ < 0) {
        throw std::runtime_error(
            "Failed to open I2C device: " +
            std::string(std::strerror(errno)));
    }

    if (ioctl(fd_, I2C_SLAVE, address) < 0) {
        close(fd_);
        throw std::runtime_error(
            "Failed to select VCM5883L I2C address");
    }
}

void Vcm5883l::write_register(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};

    if (::write(fd_, buffer, 2) != 2) {
        throw std::runtime_error("VCM5883L register write failed");
    }
}

uint8_t Vcm5883l::read_register(uint8_t reg)
{
    if (::write(fd_, &reg, 1) != 1) {
        throw std::runtime_error("VCM5883L register-address write failed");
    }

    uint8_t value;

    if (::read(fd_, &value, 1) != 1) {
        throw std::runtime_error("VCM5883L register read failed");
    }

    return value;
}

void Vcm5883l::read_registers(
    uint8_t start_reg,
    uint8_t * buffer,
    std::size_t length)
{
    if (::write(fd_, &start_reg, 1) != 1) {
        throw std::runtime_error("VCM5883L register-address write failed");
    }

    if (::read(fd_, buffer, length) != static_cast<ssize_t>(length)) {
        throw std::runtime_error("VCM5883L multi-byte read failed");
    }
}

void Vcm5883l::initialize(uint8_t odr_hz)
{
    const uint8_t chip_id = read_register(REG_CHIP_ID);

    if (chip_id != EXPECTED_CHIP_ID) {
        throw std::runtime_error(
            "VCM5883L chip ID mismatch");
    }

    configure(odr_hz);
}

void Vcm5883l::configure(uint8_t odr_hz)
{
    uint8_t reg_0a;
    uint8_t reg_0b = 0x00;  // normal SET/RESET behavior

    switch (odr_hz) {
        case 10:
            reg_0a = 0x4D;
            break;

        case 50:
            reg_0a = 0x49;
            break;

        case 100:
            reg_0a = 0x45;
            break;

        case 200:
            reg_0a = 0x41;
            reg_0b = 0x03;  // disable SET/RESET at 200 Hz
            break;

        default:
            throw std::invalid_argument(
                "Unsupported VCM-5883L output data rate: must be one of 10, 50, 100, or 200 Hz");
    }

    write_register(REG_CONTROL_1, reg_0b);
    write_register(REG_CONTROL_2, reg_0a);
}

MagneticField Vcm5883l::read()
{
    uint8_t data[6];

    read_registers(REG_X_LSB, data, 6);

    int16_t raw_x =
        static_cast<int16_t>(
            (static_cast<uint16_t>(data[1]) << 8) |
             data[0]);

    int16_t raw_y =
        static_cast<int16_t>(
            (static_cast<uint16_t>(data[3]) << 8) |
             data[2]);

    int16_t raw_z =
        static_cast<int16_t>(
            (static_cast<uint16_t>(data[5]) << 8) |
             data[4]);

    // Convert raw readings to Tesla as required by sensor_msgs/MagneticField

    return {
        static_cast<double>(raw_x) * TESLA_PER_LSB,
        static_cast<double>(raw_y) * TESLA_PER_LSB,
        static_cast<double>(raw_z) * TESLA_PER_LSB
    };
}

Vcm5883l::~Vcm5883l()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1; 
    }
}
}