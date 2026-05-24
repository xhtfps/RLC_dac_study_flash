# RLC_dac_study_flash | RLC Impedance Measurement System

这是一个基于 **STM32F407** 的高精度RLC电路参数测量项目，通过DAC输出正弦激励信号，双通道ADC同步采集响应，结合FFT算法实现电阻、电感、电容的自动识别与精确测量，支持多档位自动切换与学习模式硬件校准。

This is a high-precision RLC circuit parameter measurement project based on **STM32F407**. It outputs sinusoidal excitation signals via DAC, synchronously collects responses through dual-channel ADC, and combines FFT algorithm to realize automatic identification and accurate measurement of resistors, inductors and capacitors. It supports automatic multi-range switching and learning-mode hardware calibration.

---

## 📌 项目简介 | Project Overview

本项目实现了一套完整的便携式阻抗测量系统：
- 使用 **TIM+DAC+运放** 生成高精度正弦扫频信号（10Hz ~ 200kHz）
- 双通道12位ADC同步采集激励与响应信号，最高采样率512kHz
- 基于CMSIS-DSP库的1024点FFT算法提取基波分量，计算阻抗幅值与相位差
- 4档自动量程切换（47Ω/820Ω/15kΩ/270kΩ），覆盖宽范围测量需求
- 支持纯元件（R/L/C）与复合网络（RC/RL/LC串并联）自动识别
- 内置学习模式校准系统，可校准相位偏移、ADC零点与元件测量误差
- 3.5寸TFT-LCD实时显示测量结果与扫频曲线
- USB虚拟串口输出原始数据，支持上位机数据分析与波形绘制

This project implements a complete portable impedance measurement system:
- Generates high-precision sinusoidal sweep signals (10Hz ~ 200kHz) using **TIM+DAC+Op-Amp**
- Dual-channel 12-bit ADC synchronously collects excitation and response signals with a maximum sampling rate of 512kHz
- 1024-point FFT algorithm based on CMSIS-DSP library extracts fundamental components and calculates impedance magnitude and phase difference
- 4-range automatic switching (47Ω/820Ω/15kΩ/270kΩ) covering a wide measurement range
- Supports automatic identification of pure components (R/L/C) and composite networks (RC/RL/LC series/parallel)
- Built-in learning-mode calibration system for phase offset, ADC zero point and component measurement error
- 3.5-inch TFT-LCD real-time display of measurement results and sweep curves
- USB virtual COM port outputs raw data for upper computer data analysis and waveform drawing

---

## 📂 项目结构 | Project Structure

```
RLC_dac_study_flash/
├── Claude/               # AI辅助开发记录 | AI Development Notes
├── DSP_LIB/              # CMSIS-DSP算法库 | CMSIS-DSP Library (FFT, Filtering)
├── _01_App/              # 应用层代码 | Application Layer (Main Logic, Measurement Flow)
├── _02_Core/             # 核心系统层 | Core System Layer (Clock, Interrupt)
├── _03_Drive/            # 硬件驱动层 | Hardware Drivers (DAC, ADC, TIM, GPIO)
├── _04_FWLib/            # STM32 HAL固件库 | STM32 HAL Firmware Library
├── _05_Os/               # RTOS层 | RTOS Layer (FreeRTOS Task Management)
├── _06_System/           # 系统工具层 | System Utilities (Delay, Debug Print)
├── _07_TFT_LCD/          # TFT-LCD显示驱动 | TFT-LCD Display Driver & UI
├── _08_USB/              # USB虚拟串口驱动 | USB Virtual COM Port Driver
├── .gitignore            # Git忽略配置 | Git Ignore File
├── CLAUDE.md             # AI开发日志 | AI Development Log
├── JLinkLog.txt          # J-Link调试日志 | J-Link Debug Log
├── README.md             # 项目说明文档 | Project Documentation (This File)
├── Read.txt              # 补充说明 | Supplementary Notes
├── keilkill.bat          # Keil进程清理脚本 | Keil Process Cleanup Script
└── main.c                # 主函数入口 | Main Function Entry
```

---

## ⚙️ 核心功能说明 | Core Features

### 1. 高精度DDS信号源 | High-Precision DDS Signal Generator
- 基于TIM+DMA+DAC的直接数字频率合成技术
- 输出频率范围：10Hz ~ 200kHz，幅值0.1V ~ 3.0Vpp可调
- 自动根据频率调整采样点数，保证信号失真度<0.5%
- 支持单频点输出与多频点自动扫频

- Direct Digital Synthesis based on TIM+DMA+DAC
- Output frequency range: 10Hz ~ 200kHz, amplitude 0.1V ~ 3.0Vpp adjustable
- Automatically adjusts sampling points according to frequency, ensuring signal distortion <0.5%
- Supports single-frequency output and multi-frequency automatic sweep

### 2. FFT信号分析与阻抗计算 | FFT Signal Analysis & Impedance Calculation
- 双通道ADC同步采样，消除相位误差
- 1024点实序列FFT算法，快速提取基波幅值与相位
- 自动补偿硬件相位偏移与ADC零点误差
- 实时计算阻抗模值、阻抗角与实部虚部分量

- Dual-channel ADC synchronous sampling to eliminate phase error
- 1024-point real-sequence FFT algorithm for fast extraction of fundamental amplitude and phase
- Automatically compensates for hardware phase offset and ADC zero error
- Real-time calculation of impedance magnitude, impedance angle and real/imaginary components

### 3. 智能元件识别与参数计算 | Intelligent Component Identification
- 根据相位角自动识别元件类型：
  - |相位| < 10°：纯电阻 R
  - 相位 > 30°：感性负载 L
  - 相位 < -30°：容性负载 C
- 支持RC/RL/LC串并联复合网络自动识别
- 自动计算LC电路谐振频率
- 内置开路、短路故障检测与保护

- Automatically identifies component type based on phase angle:
  - |Phase| < 10°: Pure Resistor R
  - Phase > 30°: Inductive Load L
  - Phase < -30°: Capacitive Load C
- Supports automatic identification of RC/RL/LC series/parallel composite networks
- Automatically calculates resonant frequency of LC circuits
- Built-in open/short circuit fault detection and protection

### 4. 多档位自动量程切换 | Automatic Multi-Range Switching
- 4档标准参考电阻：47Ω / 820Ω / 15kΩ / 270kΩ
- 根据测量阻抗值自动切换最优量程
- 量程切换边界预留10%回差，防止抖动
- 每档位独立硬件校准系数，保证全量程精度

- 4 standard reference resistors: 47Ω / 820Ω / 15kΩ / 270kΩ
- Automatically switches to the optimal range based on measured impedance
- 10% hysteresis reserved at range switching boundaries to prevent jitter
- Independent hardware calibration coefficients for each range to ensure full-range accuracy

### 5. 学习模式硬件校准 | Learning-Mode Hardware Calibration
- 电阻/电容/电感数值校准：输入标准值自动生成校准系数
- 相位校准：使用纯电阻校准全频段相位偏移
- ADC零点校准：短路探头消除硬件零点误差
- 校准数据永久存储在Flash中，掉电不丢失

- R/C/L value calibration: Automatically generates calibration coefficients by inputting standard values
- Phase calibration: Calibrates full-band phase offset using pure resistors
- ADC zero calibration: Eliminates hardware zero error by shorting probes
- Calibration data is permanently stored in Flash and retained after power-off

### 6. 人机交互与数据输出 | HMI & Data Output
- 3.5寸TFT-LCD彩色显示，直观展示测量结果
- 按键操作菜单，支持参数配置与功能选择
- USB虚拟串口输出原始数据与计算结果
- 支持扫频曲线实时绘制与数据导出

- 3.5-inch TFT-LCD color display for intuitive measurement results
- Key-operated menu for parameter configuration and function selection
- USB virtual COM port outputs raw data and calculation results
- Supports real-time sweep curve drawing and data export

---

## 🛠️ 开发环境 | Development Environment

| 项目 | 中文说明 | English Description |
|------|----------|---------------------|
| 主控芯片 | STM32F407ZGT6 | STM32F407ZGT6 |
| 开发环境 | Keil MDK-ARM 5.37 | Keil MDK-ARM 5.37 |
| 固件库 | STM32 HAL 库 + CMSIS-DSP 1.9.0 | STM32 HAL Library + CMSIS-DSP 1.9.0 |
| 调试工具 | J-Link V11 | J-Link V11 |
| 操作系统 | FreeRTOS 10.3.1 | FreeRTOS 10.3.1 |
| 显示屏 | 3.5寸TFT-LCD (320×480) | 3.5-inch TFT-LCD (320×480) |

---

## 📈 后续优化方向 | Future Improvements

- 扩展量程至1mΩ ~ 100MΩ，增加毫欧与兆欧测量功能
- 优化FFT窗函数与数字滤波算法，进一步降低噪声
- 增加温度补偿模块，提高电感电容测量的温度稳定性
- 开发PC端上位机软件，支持数据记录与分析
- 移植到STM32H7平台，支持更高采样率与测量带宽
- 增加电池供电模块，实现完全便携式设计

- Extend measurement range to 1mΩ ~ 100MΩ, adding milliohm and megaohm measurement
- Optimize FFT window function and digital filtering algorithm to further reduce noise
- Add temperature compensation module to improve temperature stability of inductor and capacitor measurements
- Develop PC-side upper computer software for data recording and analysis
- Port to STM32H7 platform to support higher sampling rate and measurement bandwidth
- Add battery power supply module for fully portable design

---

## 👨‍💻 作者信息 | Author Information

- 作者 | Author: xht
- 学校 | University: 湖南理工大学 电子信息工程专业 | Hunan University of Science and Technology, Electronic Information Engineering
- 项目背景 | Project Background: 全国大学生电子设计竞赛备战项目 | Preparation Project for National Undergraduate Electronic Design Contest (NUEDC)
