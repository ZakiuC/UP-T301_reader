#include "LaserSerial.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <iostream>

LaserSerialModule::LaserSerialModule() : fd_(-1), running_(false) {}

LaserSerialModule::LaserSerialModule(const std::string &device, speed_t baud) : fd_(-1), running_(false) {
    openPort(device, baud);
}

LaserSerialModule::~LaserSerialModule() {
    stopAsyncRead();
    closePort();
}

bool LaserSerialModule::openPort(const std::string &device, speed_t baud) {
    if (fd_ != -1) closePort();
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "Cannot open " << device << ": " << strerror(errno) << std::endl;
        return false;
    }
    return configurePort(baud);
}

void LaserSerialModule::closePort() {
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool LaserSerialModule::isOpen() const {
    return fd_ != -1;
}

bool LaserSerialModule::configurePort(speed_t baud) {
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "tcgetattr error: " << strerror(errno) << std::endl;
        return false;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr error: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool LaserSerialModule::readAllData(LaserData &d, double height_mm) {
    enum State { WAIT_HEAD1, WAIT_HEAD2, READ_PAYLOAD } state = WAIT_HEAD1;
    uint8_t buf;
    uint8_t payload[12];
    int idx = 0;

    while (true) {
        int rd = ::read(fd_, &buf, 1);
        if (rd <= 0) {
            if (rd < 0) std::cerr << "Read error: " << strerror(errno) << std::endl;
            continue;
        }
        switch (state) {
            case WAIT_HEAD1:
                if (buf == 0xFE) state = WAIT_HEAD2;
                break;
            case WAIT_HEAD2:
                if (buf == 0x0A) { state = READ_PAYLOAD; idx = 0; }
                else if (buf != 0xFE) state = WAIT_HEAD1;
                break;
            case READ_PAYLOAD:
                payload[idx++] = buf;
                if (idx >= 12) {
                    // payload[11] == 0x55 是包尾标志
                    if (payload[11] == 0x55) {
                        // 校验
                        uint8_t x = 0;
                        for (int i = 0; i < 10; ++i) x ^= payload[i];
                        if (x == payload[10]) {
                            // 解析原始字段
                            d.flow_x_integral =  int16_t(uint16_t(payload[0]) | (uint16_t(payload[1]) << 8));
                            d.flow_y_integral =  int16_t(uint16_t(payload[2]) | (uint16_t(payload[3]) << 8));
                            d.integration_timespan = uint16_t(payload[4]) | (uint16_t(payload[5]) << 8);
                            d.distance = uint16_t(payload[6]) | (uint16_t(payload[7]) << 8);
                            d.valid = payload[8];
                            d.confidence = payload[9];

                            // 计算
                            d.angle_x_rad = double(d.flow_x_integral) / 10000.0;  
                            d.angle_y_rad = double(d.flow_y_integral) / 10000.0;  
                            d.time_ms    = double(d.integration_timespan) / 1000.0; // μs -> ms
                            if (d.time_ms > 0) {
                                d.angular_vel_x = d.angle_x_rad / d.time_ms;  
                                d.angular_vel_y = d.angle_y_rad / d.time_ms;  
                            }
                            // 实际位移 = 角位移 × 高度
                            d.disp_x_mm = d.angle_x_rad * height_mm;
                            d.disp_y_mm = d.angle_y_rad * height_mm;
                            if (d.time_ms > 0) {
                                d.vel_x_mm_per_ms = d.disp_x_mm / d.time_ms;
                                d.vel_y_mm_per_ms = d.disp_y_mm / d.time_ms;
                            }
                            return true;
                        }
                    }
                    // 校验或包尾错，重来
                    state = WAIT_HEAD1;
                }
                break;
        }
    }
    return false;
}

void LaserSerialModule::setCallback(Callback cb) {
    callback_ = std::move(cb);
}

void LaserSerialModule::startAsyncRead() {
    if (!running_ && callback_) {
        running_ = true;
        readerThread_ = std::thread(&LaserSerialModule::readerThreadFunc, this);
    }
}

void LaserSerialModule::stopAsyncRead() {
    if (running_) {
        running_ = false;
        if (readerThread_.joinable()) readerThread_.join();
    }
}

void LaserSerialModule::readerThreadFunc() {
    while (running_) {
        LaserData d;

        if (readAllData(d, height_mm_) && callback_) {
            callback_(d);
        }
    }
}
