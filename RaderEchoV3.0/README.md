# RaderEcho V3.0 - 相控阵雷达回波模拟器

## 项目简介

本项目是一个对海监视相控阵雷达回波模拟器，采用 C++17 开发，支持完整的雷达信号仿真流程。

### 主要功能

1. **二维相控阵雷达仿真**
   - 纯相扫工作方式
   - 支持波位码导入
   - 可配置阵元数量和间距

2. **目标回波模型**
   - 支持多种运动轨迹（静止、匀速、匀加速、变加速、圆周运动、正弦机动）
   - 支持 Swerling 0-4 型 RCS 起伏模型
   - 多目标场景仿真

3. **杂波仿真**
   - 基于网格映像法的杂波模拟
   - 支持多种分布模型（Rayleigh、Weibull、LogNormal、K 分布）
   - ZMNL（零记忆非线性）杂波生成算法
   - SIRP（球不变随机过程）杂波生成算法
   - 杂波序列内存池管理

4. **参数配置**
   - 从文件读取雷达、天线、杂波参数
   - 灵活的波位码配置

## 项目结构

```
RaderEchoV3.0/
├── CMakeLists.txt          # 主 CMake 配置文件
├── core/                   # 核心库
│   ├── CMakeLists.txt
│   ├── include/            # 头文件
│   │   ├── Types.h              # 基本类型定义
│   │   ├── ClutterGenerator.h   # 杂波生成器
│   │   ├── ClutterManager.h     # 杂波管理器
│   │   ├── GridManager.h        # 网格管理器
│   │   ├── TargetManager.h      # 目标管理器
│   │   ├── SignalProcessor.h    # 信号处理器
│   │   ├── ParameterManager.h   # 参数管理器
│   │   ├── RadarSimulator.h     # 雷达模拟器
│   │   └── Logger.h             # 日志系统
│   └── src/              # 源文件
│       ├── ClutterGenerator.cpp
│       ├── ClutterManager.cpp
│       ├── GridManager.cpp
│       ├── TargetManager.cpp
│       ├── SignalProcessor.cpp
│       ├── ParameterManager.cpp
│       └── RadarSimulator.cpp
├── app/                  # 应用程序
│   ├── CMakeLists.txt
│   └── main.cpp
├── config/               # 配置文件
│   ├── simulation_config.txt
│   └── beam_codes.txt
├── out/                  # 输出目录
│   ├── data/
│   └── logs/
└── README.md
```

## 编译说明

### 环境要求

- C++17 兼容编译器（GCC 7+, Clang 5+, MSVC 2017+）
- CMake 3.16+
- 可选：FFTW3（用于高性能频谱处理）

### Linux/Mac 编译

```bash
cd RaderEchoV3.0
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Windows 编译（MinGW）

```bash
cd RaderEchoV3.0
mkdir build && cd build
cmake -G "MinGW Makefiles" ..
mingw32-make -j$(nproc)
```

### Windows 编译（Visual Studio）

```bash
cd RaderEchoV3.0
mkdir build && cd build
cmake -G "Visual Studio 16 2019" ..
cmake --build . --config Release
```

## 使用说明

### 运行仿真

```bash
# 使用默认配置文件
./build/bin/RadarApp

# 指定配置文件
./build/bin/RadarApp ../config/simulation_config.txt

# 显示帮助
./build/bin/RadarApp --help
```

### 配置文件说明

#### 主配置文件（simulation_config.txt）

```ini
# 仿真配置
simulation_name Sea Surveillance Radar Simulation
duration 60.0                     # 仿真时长 (秒)
beam_code_file ../config/beam_codes.txt
clutter_pool_size 100

# 雷达参数
frequency 9.4e9                   # 载频 (Hz)
bandwidth 20e6                    # 带宽 (Hz)
prf 1200                          # 脉冲重复频率 (Hz)
pulse_width 0.5e-6                # 脉宽 (s)
sample_rate 60e6                  # 采样率 (Hz)
peak_power 25e3                   # 峰值功率 (W)

# 天线参数
antenna_gain 30.0                 # 增益 (dB)
antenna_height 15.0               # 高度 (m)
az_beamwidth 1.5                  # 方位波束宽度 (deg)
el_beamwidth 20.0                 # 俯仰波束宽度 (deg)
num_elements_az 32                # 方位阵元数
num_elements_el 8                 # 俯仰阵元数

# 杂波参数
clutter_type K_Distribution       # Rayleigh/Weibull/LogNormal/K_Distribution
kdist_shape 5.0
spectrum_center_freq 200.0
spectrum_bandwidth 100.0
```

#### 波位码文件（beam_codes.txt）

```ini
# 格式：beam_code  azimuth  elevation
0   0.0     0.0
1   1.5     0.0
2   3.0     0.0
...
```

## 输出数据

### CPI 数据文件

二进制格式，包含：
- 文件头：CPI 索引、脉冲数、距离门数
- 波束信息：方位、俯仰
- 回波数据：复数矩阵 [距离门 × 脉冲数]

### 日志文件

位于 `out/logs/simulation.log`，包含仿真过程和调试信息。

## 技术特点

### 1. 网格划分法杂波模拟

- 将照射区域按距离、方位、俯仰划分网格
- 每个网格独立计算天线增益和杂波功率
- 支持多波位扫描覆盖

### 2. ZMNL 杂波生成

零记忆非线性变换方法：
- 生成独立高斯随机序列
- 通过非线性变换获得目标分布
- 适用于 Rayleigh、Weibull、LogNormal 分布

### 3. SIRP 杂波生成

球不变随机过程方法：
- 高斯过程 + 随机调制
- 支持 K 分布等非高斯杂波
- 保持时间相关性

### 4. 内存池管理

- 预生成杂波序列存入内存池
- 支持文件缓存和快速加载
- 减少实时计算开销

## 版本历史

### V3.0 (2026-03-13)

- 全新架构设计，支持二维相控阵雷达
- 实现网格划分法杂波模拟
- 添加 ZMNL 和 SIRP 杂波生成算法
- 支持波位码导入
- 以 CPI 为周期更新回波
- 实现杂波序列内存池

### V2.1 (之前版本)

- 机扫雷达回波仿真
- 以 PRT 为周期更新

## 参考资料

- MATLAB 仿真代码：`/reference/简单仿真.md`
- 项目文档：`/reference/`

## 许可证

本项目用于教学和研究目的。

## 联系方式

- GitHub: https://github.com/lhl-rgb/RaderEcho.git
