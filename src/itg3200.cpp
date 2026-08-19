#include "sen0140_ros2/itg3200.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace sen0140_ros2
{

namespace
{

constexpr uint8_t REG_WHO_AM_I   = 0x00;
constexpr uint8_t REG_SMPLRT_DIV = 0x15;
constexpr uint8_t REG_DLPF_FS    = 0x16;
constexpr uint8_t REG_INT_CFG    = 0x17;
constexpr uint8_t REG_INT_STATUS = 0x1A;
constexpr uint8_t REG_TEMP_OUT_H = 0x1B;
constexpr uint8_t REG_GYRO_XOUT_H = 0x1D;
constexpr uint8_t REG_PWR_MGM    = 0x3E;

constexpr double GYRO_SENSITIVITY =
    14.375;

constexpr double DEG_TO_RAD =
    3.14159265358979323846 / 180.0;

}  // namespace


Itg3200::Itg3200(
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
            "Failed to select ITG-3200 I2C address");
    }
}


Itg3200::~Itg3200()
{
    if (fd_ >= 0) {
        ::close(fd_);
    }
}


void Itg3200::write_register(
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
            "Failed to write ITG-3200 register");
    }
}


uint8_t Itg3200::read_register(
    uint8_t reg)
{
    uint8_t value;

    read_registers(
        reg,
        &value,
        1);

    return value;
}


void Itg3200::read_registers(
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
            "Failed to select ITG-3200 register");
    }

    if (::read(
            fd_,
            buffer,
            length) !=
        static_cast<ssize_t>(length))
    {
        throw std::runtime_error(
            "Failed to read ITG-3200 registers");
    }
}


void Itg3200::initialize(
    double sample_rate_hz,
    uint8_t dlpf_cfg)
{
    /*
     * ITG-3200 requires up to 20 ms after power-up
     * before register access.
     */
    std::this_thread::sleep_for(
        std::chrono::milliseconds(20));

    const uint8_t who_am_i =
        read_register(REG_WHO_AM_I);

    /*
     * Bits 6:1 contain 110100 after reset.
     */
    if ((who_am_i & 0x7E) != 0x68) {
        throw std::runtime_error(
            "Unexpected ITG-3200 WHO_AM_I value");
    }

    configure(
        sample_rate_hz,
        dlpf_cfg);
}


void Itg3200::configure(
    double sample_rate_hz,
    uint8_t dlpf_cfg)
{
    if (dlpf_cfg > 6) {
        throw std::invalid_argument(
            "ITG-3200 DLPF configuration must be 0 to 6");
    }

    /*
     * Reset device.
     */
    write_register(
        REG_PWR_MGM,
        0x80);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(20));

    /*
     * Select PLL with X gyro reference.
     * CLK_SEL = 1
     */
    write_register(
        REG_PWR_MGM,
        0x01);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(1));

    /*
     * FS_SEL must be 3.
     *
     * Bits:
     * [4:3] FS_SEL
     * [2:0] DLPF_CFG
     */
    const uint8_t dlpf_fs =
        static_cast<uint8_t>(
            (3u << 3) |
            dlpf_cfg);

    write_register(
        REG_DLPF_FS,
        dlpf_fs);

    /*
     * Internal sample rate:
     *
     * DLPF_CFG = 0 -> 8000 Hz
     * DLPF_CFG = 1..6 -> 1000 Hz
     */
    const int internal_rate =
        (dlpf_cfg == 0)
        ? 8000
        : 1000;

    if (sample_rate_hz <= 0 ||
        sample_rate_hz > internal_rate)
    {
        throw std::invalid_argument(
            "Invalid ITG-3200 sample rate");
    }

    /*
     * Fsample =
     * Finternal / (SMPLRT_DIV + 1)
     */
    const int divider =
        (internal_rate / sample_rate_hz) - 1;

    if (divider < 0 ||
        divider > 255 ||
        internal_rate / (divider + 1) != sample_rate_hz)
    {
        throw std::invalid_argument(
            "ITG-3200 sample rate is not exactly achievable");
    }

    write_register(
        REG_SMPLRT_DIV,
        static_cast<uint8_t>(divider));

    /*
     * Enable RAW_RDY status.
     *
     * The physical interrupt pin does not need to
     * be connected for us to poll INT_STATUS.
     */
    write_register(
        REG_INT_CFG,
        0x01);
}


bool Itg3200::is_data_ready()
{
    const uint8_t status =
        read_register(REG_INT_STATUS);

    return (status & 0x01) != 0;
}


AngularVelocity Itg3200::read()
{
    uint8_t data[6];

    read_registers(
        REG_GYRO_XOUT_H,
        data,
        6);

    /*
     * ITG-3200 stores MSB first.
     */
    const int16_t raw_x =
        static_cast<int16_t>(
            (static_cast<uint16_t>(data[0]) << 8) |
            static_cast<uint16_t>(data[1]));

    const int16_t raw_y =
        static_cast<int16_t>(
            (static_cast<uint16_t>(data[2]) << 8) |
            static_cast<uint16_t>(data[3]));

    const int16_t raw_z =
        static_cast<int16_t>(
            (static_cast<uint16_t>(data[4]) << 8) |
            static_cast<uint16_t>(data[5]));

    const double scale =
        DEG_TO_RAD /
        GYRO_SENSITIVITY;

    return {
        static_cast<double>(raw_x) * scale,
        static_cast<double>(raw_y) * scale,
        static_cast<double>(raw_z) * scale
    };
}


double Itg3200::read_temperature()
{
    uint8_t data[2];

    read_registers(
        REG_TEMP_OUT_H,
        data,
        2);

    const int16_t raw =
        static_cast<int16_t>(
            (static_cast<uint16_t>(data[0]) << 8) |
            static_cast<uint16_t>(data[1]));

    /*
     * Datasheet:
     * sensitivity = 280 LSB / deg C
     * -13200 LSB corresponds to 35 deg C.
     */
    return
        35.0 +
        (static_cast<double>(raw) + 13200.0) /
        280.0;
}

}  // namespace sen0140_ros2