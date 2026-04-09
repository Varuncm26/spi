#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>
#include "spi.h" /* Linux SPI User-Space Driver */

#define READ_FLAG 0x80 
#define WRITE_FLAG 0

typedef enum {
    GYRO_FS_250DPS = 0,
    GYRO_FS_500DPS,
    GYRO_FS_1000DPS,
    GYRO_FS_2000DPS
} mpu6500GyroFs_t;

typedef enum {
    ACCEL_FS_2G = 0,
    ACCEL_FS_4G,
    ACCEL_FS_8G,
    ACCEL_FS_16G
} mpuAccelFs_t;

typedef enum {
    DLPF_250HZ = 0,
    DLPF_184HZ = 1,
    DLPF_92HZ  = 2,
    DLPF_41HZ  = 3
} mpuDlpf_t;
 
typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} mpu6500RawData_t;

typedef struct {
    float ax, ay, az;
    float temp;
    float gx, gy, gz;
} mpu6500Data_t; 

typedef struct {
    spi_t *spi; /* Pointer to the initialized Linux SPI device */
    mpu6500Data_t mpuData;
    mpu6500RawData_t rawData;
    
    float gyroScale;
    float accelScale;
    uint8_t smplrtDiv;
} mpu6500_t; 

typedef struct {
    int16_t gxOffset, gyOffset, gzOffset;
    int16_t axOffset, ayOffset, azOffset;
} mpu6500Calibration_t;

// Initialization & Config
uint8_t mpu6500Init(mpu6500_t* mpu, spi_t* spi_dev, uint32_t gyroFs, uint32_t accelFs, uint8_t smplrtDiv);
void mpu6500SetScaleFactors(mpu6500_t *mpu, uint32_t gyroFs, uint32_t accelFs);

// Data Retrieval
void mpu6500ReadRegs(mpu6500_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len);
void mpu6500WriteReg(mpu6500_t *mpu, uint8_t reg, uint8_t data);
void mpu6500Update(mpu6500_t *mpu); // Call this in your main loop to fetch data

// Calibration
void mpu6500SetGyroOffset(mpu6500_t *mpu, int16_t xOffset, int16_t yOffset, int16_t zOffset);
void mpu6500SetAccelOffset(mpu6500_t *mpu, int16_t xOffset, int16_t yOffset, int16_t zOffset);
void mpu6500Calibrate(mpu6500_t *mpu, mpu6500Calibration_t *cal, uint16_t numSamples);

#endif /* MPU6500_H */
