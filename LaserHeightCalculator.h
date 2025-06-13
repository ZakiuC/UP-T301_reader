#ifndef LASERHEIGHTCALCULATOR_H
#define LASERHEIGHTCALCULATOR_H

#include "LaserSerial.h"
#include <atomic>
#include <mutex>
#include <condition_variable>

class LaserHeightCalculator {
public:
    using HeightCallback = std::function<void(double height)>;

    LaserHeightCalculator(const std::string& device1, const std::string& device2, 
                          double angle_deg, double center_buff, speed_t baud = B460800);
    ~LaserHeightCalculator();

    bool startAsyncCalculation(HeightCallback callback);
    void stopAsyncCalculation();
    bool getLatestHeight(double& height);

private:
    LaserSerialModule sensor1_; // 激光传感器1
    LaserSerialModule sensor2_; // 激光传感器2
    double angle_rad_;  // 传感器夹角（弧度）
    double center_buff_;    // 圆心补偿
    
    // 数据同步相关成员
    std::mutex data_mutex_;
    std::condition_variable data_cv_;
    std::atomic<bool> running_{false};
    std::thread calculation_thread_;
    
    uint16_t last_dist1_{0};
    uint16_t last_dist2_{0};
    double latest_height_{0.0};
    bool new_data_available_{false};
    
    HeightCallback height_callback_;

    void sensor1Callback(const LaserData& data);
    void sensor2Callback(const LaserData& data); 
    void calculationThreadFunc();
};

#endif // LASERHEIGHTCALCULATOR_H