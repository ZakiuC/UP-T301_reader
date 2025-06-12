# UP-T301 光流激光二合一模块读取

## 资料

- [说明书](./UP-T301/光流激光二合一模块规格说明书UP-T301(V1.3)@20250421.pdf)


## 文件结构
.  
├── CMakeLists.txt  
├── LaserHeightCalculator.cpp  
├── LaserHeightCalculator.h  
├── LaserSerial.cpp  
├── LaserSerial.h  
├── main.cpp  
├── README.md  
├── UP-T301  
│ └── 光流激光二合一模块规格说明书UP-T301(V1.3)@20250421.pdf

## 数据

[LaserData](./LaserSerial.h#L10):包含了所有原始数据和计算后的数据
[LaserSerialModule](./LaserSerial.h#L31):传感器的串口类
[LaserHeightCalculator](./LaserHeightCalculator.h#L9):根据双传感器的夹角计算前方障碍高度的工具类

## 使用

### 同步获取数据

```cpp
// 1. 创建并打开串口
LaserSerialModule laser;
const std::string device = "/dev/ttyUSB0";
const speed_t baud = B460800;
if (!laser.openPort(device, baud)) {
    std::cerr << "打开串口失败: " << device << std::endl;
    return 1;
}
std::cout << "串口已打开: " << device << "@" << baud << std::endl;

// 2. 设置高度（单位 mm），用于位移计算
double height_mm = 1000.0;  // 例如相机或激光器距离被测平面 1m
laser.setHeightMm(height_mm);

// 3. 同步读取一次完整数据
{
    LaserData data;
    if (laser.readAllData(data, height_mm)) {
        std::cout << "[同步] 激光测距: " << data.distance << " mm，"
                    << "X 位移: "  << data.disp_x_mm  << " mm，"
                    << "Y 位移: "  << data.disp_y_mm  << " mm，"
                    << "置信度: "  << int(data.confidence)
                    << std::endl;
    } else {
        std::cerr << "[同步] 读取数据失败" << std::endl;
    }
}
```

### 异步获取数据

```cpp
// 1. 创建并打开串口
LaserSerialModule laser;
const std::string device = "/dev/ttyUSB0";
const speed_t baud = B460800;
if (!laser.openPort(device, baud)) {
    std::cerr << "打开串口失败: " << device << std::endl;
    return 1;
}
std::cout << "串口已打开: " << device << "@" << baud << std::endl;

// 2. 设置高度（单位 mm），用于位移计算
double height_mm = 1000.0;  // 例如相机或激光器距离被测平面 1m
laser.setHeightMm(height_mm);

// 3. 异步读取
// 设置回调：每收到一帧数据就打印
laser.setCallback([&](const LaserData &d){
    std::cout << "[异步] 距离=" << d.distance << " mm, "
                << "X_vel="  << d.vel_x_mm_per_ms << " mm/ms, "
                << "Y_vel="  << d.vel_y_mm_per_ms << " mm/ms"
                << std::endl;
});

// 启动异步读
laser.startAsyncRead();

// 主线程休眠，让异步持续工作，比如 5 秒
std::this_thread::sleep_for(std::chrono::seconds(5));

// 停止异步读并退出
laser.stopAsyncRead();
std::cout << "异步读取已停止，程序退出" << std::endl;
```