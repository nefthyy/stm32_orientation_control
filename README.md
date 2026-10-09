# STM32 Orientation Control

Orientation estimation on an STM32F401RE (Nucleo-F401RE) using an MPU6050 IMU. The firmware reads accelerometer, gyroscope and temperature data over I2C, filters it, fuses it into roll/pitch/yaw with a complementary filter, and streams the resulting roll/pitch/yaw over UART to a PC-side Python reader.

> **Status:** work in progress.

## Hardware

| Component | Notes |
|-----------|-------|
| Nucleo-F401RE | STM32F401RET6, 84 MHz system clock |
| MPU6050 | 6-axis IMU (accel + gyro + temp), I2C address `0x68` |

### Wiring

| MPU6050 | Nucleo pin | Function |
|---------|-----------|----------|
| SCL | PB8 | I2C1 SCL (400 kHz) |
| SDA | PB9 | I2C1 SDA |
| INT | PA0 | EXTI0, data-ready interrupt |
| VCC | 3V3 | |
| GND | GND | |

Other pins used: PA2/PA3 (USART2 to the ST-LINK virtual COM port, 115200 baud) and PA5 (on-board LED, toggled each output cycle).

## How it works

1. **Sensor setup** – `MPU6050_Init` verifies `WHO_AM_I`, wakes the sensor, sets the digital low-pass filter to 44 Hz, sample rate to 1 kHz, and enables the data-ready interrupt.
2. **Interrupt-driven sampling** – the MPU6050 pulses INT on each new sample; the EXTI callback sets a flag and the main loop sleeps with `__WFI()` until it does.
3. **Windowed filtering** – samples are collected in windows of `WINDOW_SIZE` (5). Accelerometer and temperature use the **median** of the window (rejects spikes); gyroscope rates use the **mean**.
4. **Sensor fusion** – a complementary filter combines integrated gyro rates with accelerometer tilt angles:
   ```
   angle = ALPHA * (angle + gyro_rate * dt) + (1 - ALPHA) * accel_angle
   ALPHA = TAU / (TAU + dt)
   ```
   `TAU` (default 0.5 s) is the tuning parameter. Yaw is gyro-only and will drift.
5. **Output** – each window, roll, pitch and yaw are packed into a 12-byte frame and sent over USART2, and the LED on PA5 toggles.

## Project layout

```
orientation/
├── Core/Src/main.c          # peripheral init and main loop (sample → filter → fuse → send)
├── Core/Src/mpu6050.c       # MPU6050 driver, raw-data conversion, filtering, fusion
├── Core/Inc/mpu6050.h       # register/filter defines, struct State, function prototypes
├── Core/                    # CubeMX-generated init, interrupts, syscalls
├── Drivers/                 # STM32F4 HAL and CMSIS
├── blinky.ioc               # STM32CubeMX configuration
├── Makefile                 # arm-none-eabi-gcc build
├── STM32F401xx_FLASH.ld     # linker script
├── startup_stm32f401xe.s    # startup code
└── read.py                  # PC-side serial reader
```

## Building and flashing

Requires the [Arm GNU toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-none-eabi-gcc`) and `make`.

```sh
cd orientation
make
```

Output goes to `orientation/build/` (`blinky.elf`, `blinky.bin`, `blinky.hex`). Flash with STM32CubeProgrammer, or drag `blinky.bin` onto the Nucleo's USB mass-storage drive.

## Reading the data on a PC

The firmware sends one 12-byte frame per window: three little-endian `float32` values, **roll, pitch, yaw** in degrees. In Python:

```python
roll, pitch, yaw = struct.unpack('<3f', ser.read(12))
```

> **Note:** `read.py` has not been updated yet. It still expects the old 14-byte raw-sensor frame (seven big-endian `int16` values), so it will print garbage until it is changed to read the 12-byte frame above.

```sh
pip install pyserial
python orientation/read.py
```

Edit the port in `read.py` (`COM3` by default) to match your board.

## Code organisation

All sensor and filter code lives in `mpu6050.c`, with its declarations in `mpu6050.h`. `main.h` includes `mpu6050.h` (inside the CubeMX `USER CODE BEGIN Includes` block, so it survives regeneration), so anything that includes `main.h` can call these functions.

| Function | Purpose |
|----------|---------|
| `MPU6050_Init` | Check `WHO_AM_I` and configure power, DLPF, sample rate and interrupt |
| `set_bits` | Split the 14-byte burst read into raw `int16` values |
| `convert_accel_and_temp`, `convert_gyro` | Convert raw values to m/s², °F and °/s (using `Num_To_Accel`, `Num_To_Temp`, `Num_To_Rate`) |
| `append_values` | Store one converted sample into the window arrays |
| `median_value`, `mean_value` | Median (on a sorted copy, so the input is left untouched) and mean of a window |
| `update_state` | Fill `struct State` with the windowed accel/temp medians and gyro means |
| `update_roll_pitch_yaw` | Complementary-filter update of roll/pitch, gyro integration of yaw |

Tunable constants (`WINDOW_SIZE`, `SAMPLE_RATE_HZ`, `TAU`) are in `mpu6050.h`.

## Sensor scaling

| Quantity | Range | Conversion |
|----------|-------|------------|
| Acceleration | ±2 g | `raw / 16384 * 9.81` → m/s² |
| Angular rate | ±250 °/s | `raw / 131` → °/s |
| Temperature | — | `raw / 340 + 36.53` → °C, then → °F |
