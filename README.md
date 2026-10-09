# STM32 Orientation Control

Orientation estimation on an STM32F401RE (Nucleo-F401RE) using an MPU6050 IMU. The firmware samples the accelerometer, gyroscope and temperature sensor at 1 kHz over I2C. It filters each 5-sample window, combines the gyro and accelerometer readings into roll/pitch/yaw with a complementary filter, and streams the angles over UART at 200 Hz to a PC-side Python reader.

> **Status:** work in progress. All timing figures below are calculated from the clock, bus and register settings. None have been measured on the board yet (see [Verifying timing](#verifying-timing)).

## Hardware

| Component | Notes |
|-----------|-------|
| Nucleo-F401RE | STM32F401RET6, Cortex-M4F, 512 KB flash, 96 KB RAM |
| MPU6050 | 6-axis IMU (accel + gyro + temp), I2C address `0x68` (AD0 low) |

### Wiring

| MPU6050 | Nucleo pin | Function |
|---------|-----------|----------|
| SCL | PB8 | I2C1 SCL (400 kHz fast mode) |
| SDA | PB9 | I2C1 SDA |
| INT | PA0 | EXTI0, rising edge, internal pull-down |
| VCC | 3V3 | |
| GND | GND | |

Other pins used: PA2/PA3 (USART2 to the ST-LINK virtual COM port) and PA5 (LD2, toggled once per output frame).

## System configuration

### MCU clocks

| Domain | Frequency | Source |
|--------|-----------|--------|
| SYSCLK / HCLK | 84 MHz | HSI 16 MHz → PLL (M=16, N=336, P=4) |
| APB1 (I2C1, USART2) | 42 MHz | HCLK / 2 |
| APB2 | 84 MHz | HCLK / 1 |
| Flash | 2 wait states | voltage scale 2 |
| SysTick | 1 kHz | HAL time base, priority 0 |

The board runs from the internal HSI oscillator, not a crystal. That's fine here because the time step used in the fusion math (`dt`) comes from the MPU6050's sample clock, not the MCU's clock (see [Timebase](#timebase)).

### MPU6050 configuration

| Register | Value | Effect |
|----------|-------|--------|
| `PWR_MGMT_1` (0x6B) | `0x00` | Wake from sleep, clock = internal 8 MHz oscillator |
| `CONFIG` (0x1A) | `0x03` | DLPF_CFG=3: accel 44 Hz BW / 4.9 ms delay, gyro 42 Hz BW / 4.8 ms delay, 1 kHz internal rate |
| `SMPLRT_DIV` (0x19) | `0x00` | Output rate = 1 kHz / (1 + 0) = **1 kHz** |
| `INT_ENABLE` (0x38) | `0x01` | DATA_RDY_EN: INT pulses on every new sample |
| `GYRO_CONFIG` / `ACCEL_CONFIG` | not written | Reset defaults: ±250 °/s, ±2 g |

### Firmware build

`arm-none-eabi-gcc`, `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`, `-Og` (debug-friendly optimisation). All filter math is single-precision `float` so it runs on the hardware FPU.

| Section | Size |
|---------|------|
| Flash (`.text` + `.data`) | ~11.2 KB of 512 KB |
| RAM (`.data` + `.bss`) | ~2.2 KB of 96 KB (plus 1 KB min stack, 512 B min heap reserved) |
| `mpu6050.o` | 1088 B code, 4 B bss |

## Data pipeline

```
MPU6050 ──INT (1 kHz)──► EXTI0 ISR ──sets──► data_ready_flag
   ▲                                              │
   │ I2C burst read, 14 B                         ▼
   └──────────────────────────── main loop: __WFI() until flag
                                                  │
              set_bits → convert_* → append_values (×5 samples)
                                                  │
                     update_state: median(accel, temp), mean(gyro)
                                                  │
                     update_roll_pitch_yaw (complementary filter, dt = 5 ms)
                                                  │
                     12-byte frame ──USART2 115200──► PC (read.py)
```

1. **Sensor setup.** `MPU6050_Init` checks that `WHO_AM_I == 0x68` and then writes the four registers listed above.
2. **Interrupt-driven sampling.** The MPU6050's INT pin fires `HAL_GPIO_EXTI_Callback`, which sets the `volatile uint8_t data_ready_flag`. The main loop sleeps in `__WFI()` until the flag is set, then clears it and does a 14-byte burst read starting at `ACCEL_XOUT_H` (0x3B).
3. **Decoding.** `set_bits` combines each pair of big-endian bytes into an `int16`. `convert_accel_and_temp` and `convert_gyro` then scale them to m/s², °F and °/s.
4. **Windowing.** Five samples (`WINDOW_SIZE`) are collected. If a read fails it is retried on the next sample, so a window always holds 5 good samples.
5. **Window reduction.** `update_state` takes the **median** of the accel and temperature samples and the **mean** of the gyro samples.
6. **Fusion.** `update_roll_pitch_yaw` runs the complementary filter once per window.
7. **Output.** Roll, pitch and yaw are packed into `state_buffer` and sent with a blocking `HAL_UART_Transmit`, then LD2 toggles.

## Timing and overhead

### Per-operation cost (calculated)

| Operation | Calculation | Time |
|-----------|-------------|------|
| Sample period | 1 kHz ODR | 1000 µs |
| I2C burst read | ~155 bit-times (addr W + reg + restart + addr R + 14 data bytes, 9 bits each incl. ACK, plus start/stop) × 2.5 µs | **~390 µs** |
| UART frame | 12 bytes × 10 bits (8N1) / 115200 baud | **~1.04 ms** |
| Window reduction + fusion | 4 × 5-element bubble sorts, 3 means, `atan2f` ×2, `sqrtf` ×1 on the FPU | tens of µs (estimate) |
| EXTI ISR | HAL dispatch + one byte store | ~1 µs |

### Per-window budget (5 ms output period)

| Activity | Time | Share of 5 ms |
|----------|------|---------------|
| 5 × I2C reads (blocking, CPU polling) | ~1.95 ms | ~39 % |
| UART transmit (blocking, CPU polling) | ~1.04 ms | ~21 % |
| Compute | < 0.1 ms | < 2 % |
| Sleeping in `__WFI()` | ~1.9 ms | ~38 % |

The CPU is busy roughly 60 % of the time, almost all of it waiting on HAL polling loops rather than doing useful work. The math itself costs very little.

### How the loop stays in step with the sensor

The 1.04 ms UART transmit is longer than one sample period, so one data-ready edge always arrives while the frame is still being sent. The flag holds onto that edge. When the next window starts, it reads that sample straight away, about 0.5 ms after it arrived. That's still in time, because the sensor doesn't overwrite its data registers until the next sample 1 ms later. The other 4 samples follow on their own edges. As a result:

- Each window covers exactly 5 sensor samples, so the output rate is 200 Hz and matches `DT`.
- There is about **0.5 ms of slack** per window. If the UART frame or compute time grows by more than that, a data-ready edge falls inside the busy period. Because the flag is a single bit, two edges then look like one, a sample is silently skipped, and the real window becomes 6 ms while `DT` still says 5 ms.

### Timebase

`DT = WINDOW_SIZE / SAMPLE_RATE_HZ = 5 / 1000 = 5 ms`. It is a compile-time constant, not a measured interval. Because the main loop is paced by the MPU6050's data-ready interrupt, `dt` follows the sensor's own oscillator. The MCU's HSI accuracy and SysTick jitter have no effect on it. Any error in the sensor's oscillator scales directly into the integrated gyro angles.

### End-to-end latency (estimate)

| Stage | Delay |
|-------|-------|
| MPU6050 DLPF group delay | ~4.8–4.9 ms |
| 5-sample boxcar mean (centre of window) | ~2 ms |
| UART transmission | ~1 ms |
| **Total, motion → PC** | **~8 ms** |

## Signal processing and fusion

### Window reduction

- **Accelerometer and temperature use the median.** Accelerometers pick up short spikes from vibration and impacts. A median of 5 throws out up to 2 outliers per window, where a mean would let them pull the result off. The median is taken on a local copy (`t[]`), so the caller's array is not reordered.
- **Gyroscope uses the mean.** The filter integrates the gyro rate. The mean rate × window length is the best estimate of the angle covered during the window. A median would discard real short rotations and bias the integral.
- **Window size 5.** Five keeps the median cheap (a bubble sort of 5 elements is 10 compares), adds about 2 ms of delay, and gives a 200 Hz output rate that fits easily in a 115200-baud link.

### Complementary filter

```
accel_roll  = atan2(a_y, a_z)
accel_pitch = atan2(-a_x, sqrt(a_y² + a_z²))

roll  = ALPHA · (roll  + g_x · dt) + (1 − ALPHA) · accel_roll
pitch = ALPHA · (pitch + g_y · dt) + (1 − ALPHA) · accel_pitch
yaw   = yaw + g_z · dt

ALPHA = TAU / (TAU + dt) = 0.5 / 0.505 ≈ 0.9901
```

- `TAU` = 0.5 s is the time constant that decides which sensor wins. Below about 1 / (2π·τ) ≈ **0.32 Hz** the filter trusts the accelerometer, which doesn't drift. Above it, it trusts the gyro, which is smooth and unaffected by linear acceleration.
- `ALPHA` is calculated from `TAU` and `DT`, so changing the window size or sample rate keeps the same time constant.
- **Startup seeding (`primed`).** On the first window, roll and pitch are set straight from the accelerometer instead of blending in from 0. Otherwise the estimate would take several `TAU` (seconds) to converge after boot.
- A complementary filter was chosen over a Kalman filter or a Madgwick/Mahony filter. It costs one multiply-add per axis, has one parameter to tune, and gives good roll/pitch for a hand-held or slow-moving board.

## Known problems and mitigations

| # | Problem | Impact | Mitigation / next step |
|---|---------|--------|------------------------|
| 1 | **Yaw drifts.** It is integrated from the gyro only, with no absolute reference. | Unbounded heading drift (degrees per minute, depending on gyro bias). | Add a magnetometer (e.g. QMC5883L / HMC5883L) and fuse heading, or treat yaw as a short-term relative value only. |
| 2 | **No gyro bias calibration.** | A constant bias integrates into roll/pitch offset (held in check by the accelerometer) and into yaw drift (not held in check). | Average about 1–2 s of gyro data at startup while still and subtract it, or write the MPU6050 offset registers. |
| 3 | **Accelerometer tilt is wrong under linear acceleration.** | Fast translation or vibration tilts the accel reference. | The median window removes spikes and `TAU` keeps the gyro dominant above 0.3 Hz. Could also reduce accel weight when \|a\| differs from 1 g. |
| 4 | **Pitch near ±90° (gimbal lock).** `atan2(a_y, a_z)` breaks down when a_y, a_z → 0. | Roll jumps around when the board is pointing straight up or down. | Acceptable for the expected range of motion. Switch to a quaternion filter (Madgwick/Mahony) if full-range attitude is needed. |
| 5 | **Gyro rates are treated as Euler-angle rates** (g_x → roll, g_y → pitch). | Small error when roll and pitch are both large at once. | Apply the full body-rate → Euler-rate transform, or use quaternions. |
| 6 | **Blocking I2C and UART** use about 60 % of the CPU in polling loops. | Little headroom for more work, and only about 0.5 ms of slack before samples are dropped (see above). | Move the I2C read to `HAL_I2C_Mem_Read_DMA`/`_IT` started from the EXTI callback, and the UART to `HAL_UART_Transmit_DMA`. Count edges with a counter instead of a 1-bit flag, so missed samples can be detected. |
| 7 | **The UART frame has no sync, header or checksum.** | If the PC starts mid-frame or a byte is lost, every later frame is misaligned and decodes as garbage. | Add a sync word and CRC-8/16, or use COBS framing. |
| 8 | **`read.py` is out of date.** It still expects the old 14-byte raw frame. | The PC output is meaningless. | Update it to read 12-byte `'<3f'` frames (see below). |
| 9 | **The I2C bus has no recovery.** A failed read just retries on the next sample. | If the bus gets stuck (slave holding SDA low after a reset mid-transfer), every read fails and the loop never produces another window. | Detect N consecutive failures, toggle SCL 9 times as GPIO, then re-run `HAL_I2C_Init` and `MPU6050_Init`. |
| 10 | **Init writes are not checked**, and `Error_Handler` gives no indication. | A failed config write goes unnoticed. A bad `WHO_AM_I` hangs the board silently. | Check each `HAL_I2C_Mem_Write` result and blink LD2 in `Error_Handler`. |
| 11 | **`WHO_AM_I` must be `0x68`.** | Many "MPU6050" boards actually carry an MPU6500 or a clone (e.g. `0x70`) and fail to initialise. | Accept the known compatible IDs, or log the value that was read. |
| 12 | **Sensor clock is the internal 8 MHz oscillator** (`CLKSEL=0`). | The datasheet recommends a gyro PLL reference for better stability, and `dt` depends on this clock. | Set `PWR_MGMT_1 = 0x01` (PLL with X-gyro reference). |
| 13 | **Temperature is calculated but never sent.** | A few wasted conversions and medians per window. | Add it to the UART frame or drop it. |
| 14 | **Stale register comment**: `SMPLRT_DIV` comment says 9 / 100 Hz, but the code writes 0 (1 kHz). | Misleading when tuning. | Fix the comment. |

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

### Code organisation

All sensor and filter code is in `mpu6050.c`, with its declarations in `mpu6050.h`. `main.h` includes `mpu6050.h` inside the CubeMX `USER CODE BEGIN Includes` block, so the include survives code regeneration and anything that includes `main.h` can call these functions. `main.c` keeps only CubeMX init, the global sample/state variables, the EXTI callback and the main loop.

| Function | Purpose |
|----------|---------|
| `MPU6050_Init` | Check `WHO_AM_I` and configure power, DLPF, sample rate and interrupt |
| `set_bits` | Split the 14-byte burst read into raw `int16` values |
| `convert_accel_and_temp`, `convert_gyro` | Convert raw values to m/s², °F and °/s (using `Num_To_Accel`, `Num_To_Temp`, `Num_To_Rate`) |
| `append_values` | Store one converted sample in the window arrays |
| `median_value`, `mean_value` | Median (on a sorted copy) and mean of a window |
| `update_state` | Fill `struct State` with the window's accel/temp medians and gyro means |
| `update_roll_pitch_yaw` | Complementary-filter update of roll/pitch, gyro integration of yaw |

Tunable constants (`WINDOW_SIZE`, `SAMPLE_RATE_HZ`, `TAU`) are in `mpu6050.h`. `DT` and `ALPHA` are calculated from them. If you change `SMPLRT_DIV` or `DLPF_CFG` in `MPU6050_Init`, update `SAMPLE_RATE_HZ` to match, or `dt` will be wrong.

## Building and flashing

Requires the [Arm GNU toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-none-eabi-gcc`) and `make`.

```sh
cd orientation
make
```

Output goes to `orientation/build/` (`blinky.elf`, `blinky.bin`, `blinky.hex`; the Makefile `TARGET` is still `blinky`). Flash with STM32CubeProgrammer, or drag `blinky.bin` onto the Nucleo's USB mass-storage drive.

On WSL with the project on a Windows drive (`/mnt/c`), `make` may print `Clock skew detected`. That comes from filesystem timestamps and doesn't mean the code is wrong.

## Serial protocol

| Parameter | Value |
|-----------|-------|
| Port | USART2 → ST-LINK virtual COM |
| Format | 115200 baud, 8N1, no flow control |
| Rate | 200 frames/s (every 5 ms) |
| Frame | 12 bytes: `float32 roll, float32 pitch, float32 yaw` (degrees, little-endian, no header) |
| Link utilisation | 12 B × 200 Hz × 10 bit = 24 kbit/s ≈ **21 %** of 115200 |

```python
roll, pitch, yaw = struct.unpack('<3f', ser.read(12))
```

> **Note:** `read.py` has not been updated yet. It still expects the old 14-byte raw-sensor frame (seven big-endian `int16` values), so it will print garbage until it is changed to read the 12-byte frame above.

```sh
pip install pyserial
python orientation/read.py
```

Edit the port in `read.py` (`COM3` by default) to match your board.

## Sensor scaling

| Quantity | Range | Sensitivity | Conversion |
|----------|-------|-------------|------------|
| Acceleration | ±2 g | 16384 LSB/g | `raw / 16384 * 9.81` → m/s² |
| Angular rate | ±250 °/s | 131 LSB/(°/s) | `raw / 131` → °/s |
| Temperature | −40 to 85 °C | 340 LSB/°C | `raw / 340 + 36.53` → °C, then → °F |

## Verifying timing

To replace the calculated figures with measured ones:

- **Logic analyser / scope.** Probe INT (PA0), SCL (PB8) and USART2 TX (PA2). Check the 1 ms INT spacing, the ~390 µs I2C read, and that each UART frame starts every 5 ms with no missed INT edges.
- **LD2 (PA5)** toggles once per frame, so it should show a 100 Hz square wave (200 toggles/s).
- **DWT cycle counter.** Enable `DWT->CYCCNT` and time `update_state` and `update_roll_pitch_yaw` in 84 MHz cycles.
