<<<<<<< HEAD
# STM32 Dart Guidance

## 项目说明

这是一个基于 STM32G431 微控制器的飞镖引导系统项目。该项目旨在为 RoboMaster 比赛中的飞镖发射系统提供精确的姿态控制和引导功能。通过集成 IMU（惯性测量单元）、PID 控制器和伺服电机驱动，实现飞镖的稳定发射和瞄准。

项目使用 STM32CubeMX 生成基础代码，并采用 CMake 进行构建管理，支持多种编译器工具链。

## 目录

- [STM32 Dart Guidance](#stm32-dart-guidance)
  - [项目说明](#项目说明)
  - [目录](#目录)
  - [安装](#安装)
    - [环境要求](#环境要求)
    - [构建步骤](#构建步骤)
  - [使用情况](#使用情况)
  - [功能](#功能)
  - [配置](#配置)
  - [许可证](#许可证)

## 安装

### 环境要求

- STM32CubeIDE 或其他支持 STM32 的 IDE
- CMake 3.16 或更高版本
- GCC ARM None EABI 工具链
- STM32CubeG4 HAL 库

### 构建步骤

1. 克隆项目到本地：
   ```bash
   git clone <repository-url>
   cd STM32
   ```

2. 配置 CMake：
   ```bash
   mkdir build
   cd build
   cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake
   ```

3. 构建项目：
   ```bash
   make
   ```

4. 烧录到 STM32 设备：
   使用 STM32CubeProgrammer 或其他烧录工具将生成的二进制文件烧录到 STM32G431 微控制器。

## 使用情况

1. 连接硬件：
   - 将 IMU 传感器（BMI088）连接到 SPI 接口
   - 连接伺服电机到 PWM 输出引脚
   - 确保电源和通信接口正确连接

2. 编译并烧录代码到 STM32 板子

3. 上电运行：
   系统将自动初始化传感器、启动控制循环，并根据配置参数进行姿态控制

4. 监控输出：
   通过 UART 接口查看调试信息和传感器数据

## 功能

- **IMU 数据采集**：集成 BMI088 传感器，支持加速度计和陀螺仪数据读取
- **姿态解算**：使用 Mahony 滤波算法进行姿态估计
- **PID 控制**：实现多轴 PID 控制器用于姿态稳定
- **伺服电机驱动**：控制伺服电机进行瞄准和发射
- **通信接口**：支持 UART 和 SPI 通信

## 配置

项目配置主要通过以下文件进行：

- `STM32.ioc`：STM32CubeMX 项目配置文件，定义引脚分配和外设配置
- `CMakeLists.txt`：CMake 构建配置文件
- `Core/Inc/` 目录下的头文件：包含宏定义和配置参数

主要配置参数：

- IMU 采样率和滤波参数（在 `imu.h` 中定义）
- PID 控制参数（在 `pid.h` 中定义）
- 伺服电机参数（在 `servo.h` 中定义）
- UART 通信波特率（在 `usart.h` 中定义）

## 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。
=======
# dart_guidance
制导镖主控
>>>>>>> ee8af9bd0bbeaf9f6b0ff70783571bcc57177b2b
