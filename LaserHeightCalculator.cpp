#include "LaserHeightCalculator.h"
#include <cmath>
#include <iostream>

// 弧度制
constexpr double DEG_TO_RAD = M_PI / 180.0;

LaserHeightCalculator::LaserHeightCalculator(const std::string& device1, 
                                            const std::string& device2,
                                            double angle_deg,
                                            double center_buff,
                                            speed_t baud)
    : sensor1_(device1, baud),
      sensor2_(device2, baud),
      angle_rad_(angle_deg * DEG_TO_RAD),
      center_buff_(center_buff){
    
    // 设置回调
    sensor1_.setCallback([this](const LaserData& data) { this->sensor1Callback(data); });
    sensor2_.setCallback([this](const LaserData& data) { this->sensor2Callback(data); });
}

LaserHeightCalculator::~LaserHeightCalculator() {
    stopAsyncCalculation();
}

bool LaserHeightCalculator::startAsyncCalculation(HeightCallback callback) {
    if (running_) return false;
    
    height_callback_ = std::move(callback);
    running_ = true;
    
    // 启动
    sensor1_.startAsyncRead();
    sensor2_.startAsyncRead();
    
    calculation_thread_ = std::thread(&LaserHeightCalculator::calculationThreadFunc, this);
    return true;
}

void LaserHeightCalculator::stopAsyncCalculation() {
    if (!running_) return;
    
    running_ = false;
    data_cv_.notify_all();
    
    // 停止
    sensor1_.stopAsyncRead();
    sensor2_.stopAsyncRead();

    if (calculation_thread_.joinable()) {
        calculation_thread_.join();
    }
}

bool LaserHeightCalculator::getLatestHeight(double& height) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!new_data_available_) return false;
    
    height = latest_height_;
    new_data_available_ = false;
    return true;
}

void LaserHeightCalculator::sensor1Callback(const LaserData& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_dist1_ = data.distance;
    data_cv_.notify_one();  // 通知计算线程
}

void LaserHeightCalculator::sensor2Callback(const LaserData& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_dist2_ = data.distance;
    data_cv_.notify_one();  // 通知计算线程
}

void LaserHeightCalculator::calculationThreadFunc() {
    while (running_) {
        uint16_t dist1, dist2;
        
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            // 等待新数据
            data_cv_.wait(lock, [this] {
                return !running_ || (last_dist1_ > 0 && last_dist2_ > 0);
            });
            
            if (!running_) break;
            
            dist1 = last_dist1_;
            dist2 = last_dist2_;
            
            // 重置标志
            last_dist1_ = 0;
            last_dist2_ = 0;
        }
        
        std::cout << "Received distances - Sensor1: " << dist1 
          << " mm, Sensor2: " << dist2 << " mm" << std::endl;
          
        // 三角测量计算高度
        // 公式: h = (d1 * d2 * sin(θ)) / sqrt(d1² + d2² - 2*d1*d2*cos(θ))
        const double d1 = static_cast<double>(dist1) + center_buff_;
        const double d2 = static_cast<double>(dist2) + center_buff_;
        const double cos_theta = std::cos(angle_rad_);
        const double sin_theta = std::sin(angle_rad_);
        
        const double denominator = std::sqrt(d1*d1 + d2*d2 - 2*d1*d2*cos_theta);
        double height = 0.0;
        
        if (denominator > 1e-6) {  // 避免除以零
            height = (d1 * d2 * sin_theta) / denominator;
        }
        
        // 更新最新高度
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_height_ = height;
            new_data_available_ = true;
        }
        
        // 执行回调
        if (height_callback_) {
            height_callback_(height);
        }
    }
}