#ifndef LASERSERIAL_H
#define LASERSERIAL_H

#include <string>
#include <cstdint>
#include <functional>
#include <termios.h>
#include <thread>

struct LaserData {
    // 原始量
    int16_t flow_x_integral;       // X 角位移 ×10000
    int16_t flow_y_integral;       // Y 角位移 ×10000
    uint16_t integration_timespan; // 累计时间 us
    uint16_t distance;             // 激光测距 mm
    uint8_t  valid;                // 数据有效标志
    uint8_t  confidence;           // 置信度

    // 计算量（须传入高度 height_mm）
    double angle_x_rad;            // X 角位移 rad
    double angle_y_rad;            // Y 角位移 rad
    double time_ms;                // 累计时间 ms
    double angular_vel_x;          // X 角速度 rad/ms
    double angular_vel_y;          // Y 角速度 rad/ms
    double disp_x_mm;              // X 实际位移 mm
    double disp_y_mm;              // Y 实际位移 mm
    double vel_x_mm_per_ms;        // X 真实速度 mm/ms
    double vel_y_mm_per_ms;        // Y 真实速度 mm/ms
};

class LaserSerialModule {
public:
    using Callback = std::function<void(const LaserData &data)>;

    LaserSerialModule();
    explicit LaserSerialModule(const std::string &device, speed_t baud = B460800);
    ~LaserSerialModule();

    // 打开串口，返回是否成功
    bool openPort(const std::string &device, speed_t baud = B460800);
    // 关闭串口
    void closePort();
    // 串口是否已打开
    bool isOpen() const;

    // 同步读取一次距离，成功返回 true，并将距离写入 distance
    bool readAllData(LaserData &d, double height_mm);

    // 设置异步回调
    void setCallback(Callback cb);
    // 启动异步读取线程
    void startAsyncRead();
    // 停止异步读取线程
    void stopAsyncRead();
    // 设置高度
    void setHeightMm(double h) { height_mm_ = h; }
    

private:
    int fd_;
    bool configurePort(speed_t baud);

    double height_mm_ = 1.0;  // mm

    // 异步读取相关
    Callback callback_;
    std::thread readerThread_;
    bool running_;
    void readerThreadFunc();

    

};

#endif // LASERSERIAL_H
