#include <iostream>
#include "LaserSerial.h"
#include "LaserHeightCalculator.h"

// 同步调用
// int main() {
//     LaserSerialModule laser("/dev/ttyUSB0", B460800);
//     if (!laser.isOpen()) return -1;

//     uint16_t dist;
//     if (laser.readDistance(dist)) {
//         std::cout << "Distance: " << dist << std::endl;
//     }
//     return 0;
// }

// 异步调用
// int main() {
//     LaserSerialModule laser("/dev/ttyUSB0", B460800);
//     if (!laser.isOpen()) return -1;

//     laser.setCallback([](uint16_t d) {
//         std::cout << "Distance: " << d << std::endl;
//     });
//     laser.startAsyncRead();

//     std::this_thread::sleep_for(std::chrono::seconds(10));
//     laser.stopAsyncRead();

//     return 0;
// }


// 双测距测高度
// int main() {
//     // 创建高度计算器（两个传感器，夹角30度）
//     LaserHeightCalculator calculator("/dev/ttyUSB0", "/dev/ttyUSB1", 30.0);
    
//     // 设置高度回调函数
//     auto callback = [](double height) {
//         std::cout << "障碍物高度: " << height << " mm" << std::endl;
//     };
    
//     // 启动异步计算
//     if (calculator.startAsyncCalculation(callback)) {
//         std::cout << "高度计算已启动，按Enter键停止..." << std::endl;
//         std::cin.get();
//         calculator.stopAsyncCalculation();
//     }
    
//     return 0;
// }

// 
int main() {
    LaserSerialModule laser;
    const std::string device = "/dev/ttyUSB0";
    const speed_t baud = B460800;
    if (!laser.openPort(device, baud)) {
        std::cerr << "打开串口失败: " << device << std::endl;
        return 1;
    }
    std::cout << "串口已打开: " << device << "@" << baud << std::endl;

    // 设置高度（单位 mm）
    double height_mm = 700.0;  
    laser.setHeightMm(height_mm);

    laser.setCallback([&](const LaserData &data){
        std::cout << "距离=" << data.distance << " mm, "
                  << "X_vel="  << data.vel_x_mm_per_ms << " mm/ms, "
                  << "Y_vel="  << data.vel_y_mm_per_ms << " mm/ms"
                  << std::endl;
    });

    // 启动
    laser.startAsyncRead();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 停止退出
    laser.stopAsyncRead();
    std::cout << "读取已停止，程序退出" << std::endl;

    return 0;
}