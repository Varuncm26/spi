# STM32F411 MPU-6500 IMU Driver


This project implements **SPI + DMA communication** with the MPU-6500 6-axis IMU on an STM32F401 microcontroller. The sensor streams 3-axis accelerometer, 3-axis gyroscope, and temperature data. Raw samples are averaged before being output.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Microcontroller | STM32F411 (Black Pill) @ 84 MHz |
| Sensor | MPU-6500 6-axis IMU |
| Communication | SPI1, full-duplex, DMA-driven |
| SPI Mode | Mode 3 (CPOL=1, CPHA=1) |
| SCK | PA5 |
| MISO | PA6 |
| MOSI | PA7 |
| CS | PB0 (software-controlled) |
| MPU INT | PA0 (EXTI0 — data-ready interrupt) |
| TIM2 | Millisecond wall clock (update interrupt) |
| TIM5 | Free-running microsecond counter |

MPU-6500: REGISTER MAP ->   https://www.glynstore.com/content/docs/invensense/RM-MPU-6555A.pdf

MPU-6500: Datasheet -> https://datasheet.octopart.com/MPU-6500-InvenSense-datasheet-138896167.pdf

---

## Project Structure

```
├── main.c              # Entry point, ISR handlers (DMA, EXTI0, EXTI1, TIM2)
├── spi.c / spi.h       # SPI init, DMA-based transact, CS control, RX callback.
├── mpu6500.c / .h      # MPU-6500 init, read/write, scaling, calibration
├── mpuRegister.h       # Full MPU-6500 register map and bit definitions
├── swa.c / swa.h       # Array Averaging Filter + MPU data update logic
├── exti.c / exti.h     # EXTI init for hardware (MPU INT) and software interrupts
├── dma.c / dma.h       # DMA flag clear/check helpers 
├── timer.c / timer.h   # General-purpose timer init, wall clock timer, interrupt enable
├── wallclock.c / .h    # Millisecond/microsecond wall clock, blocking & non-blocking delays
├── boardfile.c / .h    # Board-level pin/DMA/buffer configuration arrays
└── clock_init.c / .h  # System clock config (HSE 25 MHz → PLL → 84 MHz SYSCLK)
```

---

## How the SPI Queue Works

```c
spiTransact(&spiData, txBuffer, length, callbackFn, ctx);
// ├── Copies txBuffer → spiBufferTx
// ├── Asserts CS low
// ├── Starts RX DMA, then TX DMA simultaneously
// └── Sets spiStatus = SPI_PROCESSING  (blocks re-entry)

DMA2_Stream0_IRQHandler()
└── spiProcessRx()                  // RX DMA complete
    ├── Deasserts CS high
    ├── Invokes rxCallBack(void* targetstruct, spiBufferRx, length)
    └── Sets spiStatus = SPI_IDLE
```

## Interrupt Handlers

There are 3 interrupts that drive the entire data pipeline. They fire in order, one after the other.

---

### 1. EXTI0_IRQHandler — MPU-6500 Data Ready

**Trigger:** The MPU-6500 pulls its INT pin high (PA0) every time a new sensor sample is ready inside the chip.

**What it does:**
1. Clears the EXTI0 pending flag
2. Calls `mpu6500Read()` which builds a 15-byte TX buffer (register address + 14 dummy bytes) and hands it to `spiTransact()`
3. `spiTransact()` loads the TX buffer into DMA, pulls CS low, and kicks off both the TX and RX DMA streams simultaneously
4. Returns immediately — the actual data arrives later via the DMA interrupt

This IRQ only starts the SPI transfer. It does not wait for data.

---

### 2. DMA2_Stream0_IRQHandler — SPI RX Transfer Complete

**Trigger:** DMA2 Stream 0 fires when it has finished moving the incoming SPI bytes from the SPI data register into `spiBufferRx` in memory.

**What it does:**
1. Calls `spiProcessRx()`
2. Inside `spiProcessRx()`:
   - Clears the DMA transfer-complete flag
   - Pulls CS high (ends the SPI transaction)
   - Calls `rxCallBack()` — which is `mpu6500ReadData()`
   - Sets `spiStatus = SPI_IDLE` so the next transaction can be accepted
3. Inside `mpu6500ReadData()` (the callback):
   - Copies `spiBufferRx` into `mpu->rxbuffer`
   - Combines the high and low bytes for each axis into 16-bit signed integers
   - Calls `MPU6500_Update()` which feeds each raw value into the averaging filter
   - Increments the global sample counter `count`
   - When `count` reaches 5, it calls `softwareInterruptTrigger()` and resets `count` to 0

This IRQ does the actual data parsing and filtering.

---

### 3. EXTI1_IRQHandler — Software Triggered, Averaged Data Ready

**Trigger:** Not a hardware pin. `softwareInterruptTrigger()` writes to the EXTI software interrupt register (`LL_EXTI_GenerateSWI_0_31`) which immediately pends EXTI1 in the NVIC. This is called from inside `mpu6500ReadData()` once every 5 samples.

**What it does:**
1. Clears the EXTI1 pending flag
2. Sets `flag = 1` in main
3. The main loop checks `flag`, prints the latest averaged accelerometer, gyroscope, and temperature values via `printf`, then clears `flag`

This IRQ is the bridge between the ISR context (where data is parsed) and the main loop (where data is printed). It fires at 1/5th the rate of EXTI0.

---

### Summary



##  MPU-6500 Setup Flow
,
### Initialization

```
mpu6500Init()
    │
    ├── WHO_AM_I check
    ├── Device RESET  (PWR_MGMT_1)
    ├── Signal path RESET
    ├── SET_CLOCK_SOURCE  → auto-select best clock
    ├── Set SMPLRT_DIV
    ├── Configure DLPF  (CONFIG register)
    ├── Set GYRO full-scale range  (GYRO_CONFIG)
    ├── Set ACCEL full-scale range  (ACCEL_CONFIG)
    ├── Set gyroScale / accelScale factors
    ├── Disable I2C interface  (USER_CTRL)
    └── Enable DATA_READY interrupt  (INT_ENABLE)
```

### Continuous Measurement Loop

```
MPU-6500 asserts INT pin (PA0) on every new sample
        │
        ▼
EXTI0_IRQHandler
        │  calls
        ▼
mpu6500Read(&mpuint, REG_ACCEL_XOUT_H, 15 bytes)
        │  triggers
        ▼
spiTransact()  ──────────────────  DMA TX + RX kicks off
        │
        ▼  (DMA RX complete)
DMA2_Stream0_IRQHandler
        │  calls
        ▼
spiProcessRx()
        │  invokes callback
        ▼
mpu6500ReadData()
        │
        │  Parses 15 RX bytes:
        │  [0]      = dummy (register address echo)
        │  [1–2]    = AX   (Accel X High/Low)
        │  [3–4]    = AY
        │  [5–6]    = AZ
        │  [7–8]    = TEMP
        │  [9–10]   = GX   (Gyro X High/Low)
        │  [11–12]  = GY
        │  [13–14]  = GZ
        │
        ▼
MPU6500_Update()
        │  feeds each axis into SWA filter via ArrayUpdate()
        │  increments global count
        │
        ├── count < 5  →  keep accumulating samples
        │
        └── count == 5 →  compute averages, reset sum
                │  calls
                ▼
        softwareInterruptTrigger()   ←  EXTI Line 1 software trigger
                │
                ▼
        EXTI1_IRQHandler
                │  sets flag = 1, resets count2
                ▼
           main loop
                │  on flag:
                ▼
        printf(ax, ay, az, gx, gy, gz, temp)  ← scaled float values
                │
                └──────────────────────────────────────► repeat
```

---

## Averaging Filter

```c
initSWA(&swa, windowSize);
// Zero-initialises array[], sum, avg

// Called on every new raw sample:
ArrayUpdate(&swa, rawValue);
// ├── array[count] = rawValue
// ├── sum += rawValue
// └── When count reaches MAX_WINDOW_SIZE (5):
//         avg  = sum / 5
//         sum  = 0
//         softwareInterruptTrigger()
```

Each axis has its own `SWA_t` instance inside `MPU6500_t`:

```c
typedef struct {
    SWA_t ax, ay, az;   // Accelerometer axes
    SWA_t gx, gy, gz;   // Gyroscope axes
    SWA_t temp;         // Temperature
} MPU6500_t;
```


---

## Clock Configuration

```
HSE (25 MHz)
    │  /M = 25
    ▼
  1 MHz  (PLL input)
    │  ×N = 336
    ▼
336 MHz  (VCO)
    ├──  /P = 4  →  84 MHz  SYSCLK
    └──  /Q = 7  →  48 MHz  USB clock

AHB  prescaler /1  →  84 MHz
APB2 prescaler /1  →  84 MHz   (SPI1, SYSCFG)
APB1 prescaler /2  →  42 MHz   (TIM2 timer clock = 84 MHz after ×2)
```

