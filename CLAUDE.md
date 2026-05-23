# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F407ZG embedded project for **RLC impedance measurement and component identification**. Uses ADC+FFT to measure impedance magnitude and phase, then identifies R/L/C components and their configuration (series/parallel RC/RL/LC).

## Build System

This is a **Keil uVision 5** project. There is no command-line build — use the Keil IDE.
- Project file: `_04.uvprojx`
- Target MCU: STM32F407ZG
- Flash/Debug via J-Link
- After editing `.c`/`.h` files, rebuild in Keil (F7) before flashing (F8)

## Code Architecture

### Layer Structure (numbered directories)

| Directory | Role |
|-----------|------|
| `_01_App/` | Application logic — `User.c` is the main application entry |
| `_02_Core/` | CMSIS core headers and STM32 system files |
| `_03_Drive/` | Peripheral drivers and user-level driver wrappers |
| `_04_FWLib/` | STM32 Standard Peripheral Library (ST official, do not modify) |
| `_05_Os/` | Lightweight RTOS (µC/OS-style) with task creation, LCD UI framework |
| `_06_System/` | System utilities: `delay`, `usart`, `sys` (bit-band macros) |
| `_07_TFT_LCD/` | TFT LCD driver (800x480), font rendering, W25Q64 SPI flash |

### Entry Flow

1. `main.c` → creates 3 RTOS tasks: `User_main` (priority 0), `MyPs2KeyScan` (priority 2), `LED_main` (priority 3)
2. `OS_Init()` calls `System_init()`, `LED_Init()`, `PS2_Keyboard_Init()`, `OS_LCD_Init()`
3. `User_main()` in `_01_App/User.c` is the real application entry — initializes all hardware, then runs the menu loop

### Key Application File: `_01_App/User.c`

This is the core of the project (~2000 lines). It contains:
- **Menu system**: 5 menus selected via PS2 keyboard
  - Menu 1 (`MenuHaddler_1`): Single-frequency impedance + auto-identification (R/L/C, short/open)
  - Menu 2 (`MenuHaddler_2`): 5-point frequency sweep (100Hz–50kHz) + network identification (RC_S/RC_P/RL_S/RL_P/LC_S/LC_P)
  - Menu 3 (`MenuHaddler_3`): Raw data display (manual frequency/gear selection)
  - Menu 4 (`MenuHaddler_4`): Learning/calibration mode (stores correction factors to Flash)
  - Menu 5 (`MenuHaddler_5`): Reset learned calibration data
- **Impedance measurement**: `Get_Zabs()` = `Rref * V1 / V2 / Calibration[gear][freq]`
- **FFT processing**: `Get_FFTInformation()` — 10-sample median filter on phase/voltage
- **4-gear ranging**: 47Ω / 820Ω / 15kΩ / 270kΩ reference resistors, auto-switched via PC11/PC12 GPIO
- **Calibration tables**: `Resistance_Calibration[]`, `Capacitance_Calibration[4][5]`, `Inductance_Calibration[4][5]`
- **Flash storage**: Sector 10 = learning proportions (1203 floats), Sector 11 = raw values (1200 floats)

### Driver Layer Highlights (`_03_Drive/`)

- `Drive_DAC.c`: DDS waveform generator using DAC+DMA+TIM6. Supports sin/triangle/sawtooth/square. `setDDS(vpp, freq, duty, wave)` is the main API.
- `Drive_DMA_DSP_FFT.c`: Dual-channel ADC (ADC1+ADC3) with DMA, CMSIS DSP FFT for phase extraction
- `Drive_AD9959.c/h`: AD9959 DDS IC driver (external signal source, SPI bit-bang)
- `Drive_Flash.c/h`: Internal Flash read/write (sector erase + program)
- `Drive_ADF4351.c/h`: ADF4351 PLL synthesizer driver
- `Drive_PS2.c/h`: PS2 keyboard input (used for UI navigation)
- `User_*` files (e.g., `User_ADC.c`, `User_DAC.c`): Higher-level driver wrappers

### Master Header: `_05_Os/User_header.h`

All source files include `User_header.h` — it pulls in the RTOS, CMSIS, StdPeriph, and all driver headers. Adding a new driver header here makes it available everywhere.

### Hardware Pin Mapping (key signals)

- PA4/PA5: DAC outputs (CH1/CH2)
- PC11/PC12: Gear select GPIO (47Ω/820Ω/15kΩ/270kΩ via 2-bit encoding)
- ADC1/ADC3: Dual-channel analog input (V1=component voltage, V2=reference resistor voltage)
- SPI/GPIO: AD9959 DDS on PE/PF ports
- FSMC: TFT LCD (800x480)

## Key Conventions

- **Naming**: `Drive_*` = low-level peripheral driver, `User_*` = high-level application wrapper, `App_*` = application module
- **All source includes `User_header.h`** — never include individual StdPeriph headers directly
- **Flash sector layout**: Sectors 0-9 = code, Sector 10 = calibration proportions, Sector 11 = calibration raw values
- **Type aliases**: `u8`=uint8_t, `u16`=uint16_t, `u32`=uint32_t (defined in `User_header.h`)
- **Phase convention**: -180° to +180°, positive = inductive (RL), negative = capacitive (RC)
- **LCD coordinate system**: 800x480, menu on left 250px, data display on right
- External peripherals (AD9959, ADF4351, SI5351A, ADS1256) are present in the driver layer but some may be unused in current firmware (commented out in `Init_All()`)
