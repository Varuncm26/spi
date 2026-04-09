#include "mpu6500.h"
#include "mpuRegister.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> /* For usleep() on Linux */


/* Helper function for delay */
static void delay_ms(uint32_t ms) {
    usleep(ms * 1000);
}

void mpu6500ReadRegs(mpu6500_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    if (!mpu || !mpu->spi) return;
    // Uses the optimized spi_write_then_read from your spi.c driver
    spi_write_then_read(mpu->spi, (READ_FLAG | (reg & 0x7F)), NULL, 0, buffer, len);
}

void mpu6500WriteReg(mpu6500_t *mpu, uint8_t reg, uint8_t data)
{ 
    if (!mpu || !mpu->spi) return;
    
    uint8_t txData[2];
    txData[0] = (WRITE_FLAG | (reg & 0x7F));
    txData[1] = data;
    
    spi_write(mpu->spi, txData, 2); 
    delay_ms(1); // Small delay to allow register write to settle
}

void mpu6500SetScaleFactors(mpu6500_t *mpu, uint32_t gyroFs, uint32_t accelFs) 
{
    switch (gyroFs) {
        case GYRO_FS_250DPS:  mpu->gyroScale = 1.0f / 131.0f;  break;
        case GYRO_FS_500DPS:  mpu->gyroScale = 1.0f / 65.5f;   break;
        case GYRO_FS_1000DPS: mpu->gyroScale = 1.0f / 32.8f;   break;
        case GYRO_FS_2000DPS: mpu->gyroScale = 1.0f / 16.4f;   break;
        default:              mpu->gyroScale = 1.0f / 131.0f;  break;
    }

    switch (accelFs) {
        case ACCEL_FS_2G:  mpu->accelScale = 1.0f / 16384.0f; break;
        case ACCEL_FS_4G:  mpu->accelScale = 1.0f / 8192.0f;  break;
        case ACCEL_FS_8G:  mpu->accelScale = 1.0f / 4096.0f;  break;
        case ACCEL_FS_16G: mpu->accelScale = 1.0f / 2048.0f;  break;
        default:           mpu->accelScale = 1.0f / 16384.0f; break;
    }
}

uint8_t mpu6500Init(mpu6500_t* mpu, spi_t* spi_dev, uint32_t gyroFs, uint32_t accelFs, uint8_t smplrtDiv)
{
    if (!mpu || !spi_dev) return 0;
    
    mpu->spi = spi_dev; // Attach the Linux SPI context
    
    uint8_t whoAmI = 0;
    mpu6500ReadRegs(mpu, MPU6500_REG_WHO_AM_I, &whoAmI, 1);
    printf("WHO_AM_I: 0x%02X\n", whoAmI);
    
    if(whoAmI != 0x70) {
        printf("ERROR: MPU6500 Not Found!\n");
        return 0; 
    }

    // Reset Device
    mpu6500WriteReg(mpu, MPU6500_REG_PWR_MGMT_1, MPU_BIT_DEVICE_RESET);
    delay_ms(100);
    
    // Reset Signal Path
    uint8_t sigPathResetParams = (MPU_BIT_GYRO_RST | MPU_BIT_ACCEL_RST | MPU_BIT_TEMP_RST);
    mpu6500WriteReg(mpu, MPU6500_REG_SIGNAL_PATH_RESET, sigPathResetParams);
    delay_ms(100);

    // Wake Up & Set Clock Source
    mpu6500WriteReg(mpu, MPU6500_REG_PWR_MGMT_1, MPU_CLKSEL_AUTO);
    delay_ms(10);
    
    // Disable I2C Interface (Enable SPI Only Mode)
    mpu6500WriteReg(mpu, MPU6500_REG_USER_CTRL, MPU_BIT_I2C_IF_DIS);
    
    // Set Sample Rate Divider
    mpu->smplrtDiv = smplrtDiv;
    mpu6500WriteReg(mpu, MPU6500_REG_SMPLRT_DIV, mpu->smplrtDiv);
    
    // General Configuration / DLPF
    mpu6500WriteReg(mpu, MPU6500_REG_CONFIG, DLPF_41HZ);
    
    // Gyroscope Configuration
    mpu6500WriteReg(mpu, MPU6500_REG_GYRO_CONFIG, (gyroFs << MPU_GYRO_FS_SHIFT));
    
    // Accelerometer Configuration
    mpu6500WriteReg(mpu, MPU6500_REG_ACCEL_CONFIG, (accelFs << MPU_ACCEL_FS_SHIFT));

    // Configure Interrupt 
    uint8_t intPinConfig = (MPU_BIT_LATCH_INT_EN | MPU_BIT_INT_ANYRD_2CLEAR );
    mpu6500WriteReg(mpu, MPU6500_REG_INT_PIN_CFG, intPinConfig); 
    mpu6500WriteReg(mpu, MPU6500_REG_INT_ENABLE, MPU_BIT_RAW_RDY_EN);

    // Assign Math Scale Factors
    mpu6500SetScaleFactors(mpu, gyroFs, accelFs);

    printf("MPU6500 Init SUCCESS\n");
    return 1;
}

// Single blocking update function to grab all sensor data at once
void mpu6500Update(mpu6500_t *mpu)
{
    if (!mpu) return;

    uint8_t buffer[14];
    // Read 14 bytes starting from ACCEL_XOUT_H (Accel, Temp, Gyro)
    mpu6500ReadRegs(mpu, MPU6500_REG_ACCEL_XOUT_H, buffer, 14);

    // Parse raw bytes into 16-bit signed integers
    mpu->rawData.ax   = (int16_t)((buffer[0] << 8)  | buffer[1]);
    mpu->rawData.ay   = (int16_t)((buffer[2] << 8)  | buffer[3]);
    mpu->rawData.az   = (int16_t)((buffer[4] << 8)  | buffer[5]);
    mpu->rawData.temp = (int16_t)((buffer[6] << 8)  | buffer[7]);
    mpu->rawData.gx   = (int16_t)((buffer[8] << 8)  | buffer[9]);
    mpu->rawData.gy   = (int16_t)((buffer[10] << 8) | buffer[11]);
    mpu->rawData.gz   = (int16_t)((buffer[12] << 8) | buffer[13]);

    // Convert raw values to physical units 
    mpu->mpuData.ax = (float)mpu->rawData.ax * mpu->accelScale;
    mpu->mpuData.ay = (float)mpu->rawData.ay * mpu->accelScale;
    mpu->mpuData.az = (float)mpu->rawData.az * mpu->accelScale;

    mpu->mpuData.gx = (float)mpu->rawData.gx * mpu->gyroScale;
    mpu->mpuData.gy = (float)mpu->rawData.gy * mpu->gyroScale;
    mpu->mpuData.gz = (float)mpu->rawData.gz * mpu->gyroScale;

    // Convert Temperature to Celsius
    mpu->mpuData.temp = ((((float)mpu->rawData.temp - 21.0f) / 333.87f) + 21.0f);
}

void mpu6500SetGyroOffset(mpu6500_t *mpu, int16_t xOffset, int16_t yOffset, int16_t zOffset)
{
    mpu6500WriteReg(mpu, MPU6500_REG_XG_OFFSET_H, (uint8_t)((xOffset >> 8) & 0xFF));
    mpu6500WriteReg(mpu, MPU6500_REG_XG_OFFSET_L, (uint8_t)( xOffset       & 0xFF));

    mpu6500WriteReg(mpu, MPU6500_REG_YG_OFFSET_H, (uint8_t)((yOffset >> 8) & 0xFF));
    mpu6500WriteReg(mpu, MPU6500_REG_YG_OFFSET_L, (uint8_t)( yOffset       & 0xFF));

    mpu6500WriteReg(mpu, MPU6500_REG_ZG_OFFSET_H, (uint8_t)((zOffset >> 8) & 0xFF));
    mpu6500WriteReg(mpu, MPU6500_REG_ZG_OFFSET_L, (uint8_t)( zOffset       & 0xFF));
}

void mpu6500SetAccelOffset(mpu6500_t *mpu, int16_t xOffset, int16_t yOffset, int16_t zOffset)
{
    mpu6500WriteReg(mpu, MPU6500_REG_XA_OFFSET_H, (uint8_t)((xOffset >> 7) & 0xFF));
    mpu6500WriteReg(mpu, MPU6500_REG_XA_OFFSET_L, (uint8_t)((xOffset & 0x7F) << 1));

    mpu6500WriteReg(mpu, MPU6500_REG_YA_OFFSET_H, (uint8_t)((yOffset >> 7) & 0xFF));
    mpu6500WriteReg(mpu, MPU6500_REG_YA_OFFSET_L, (uint8_t)((yOffset & 0x7F) << 1));

    mpu6500WriteReg(mpu, MPU6500_REG_ZA_OFFSET_H, (uint8_t)((zOffset >> 7) & 0xFF));
    mpu6500WriteReg(mpu, MPU6500_REG_ZA_OFFSET_L, (uint8_t)((zOffset & 0x7F) << 1));
}

void mpu6500Calibrate(mpu6500_t *mpu, mpu6500Calibration_t *cal, uint16_t numSamples)
{
    int32_t gxSum = 0, gySum = 0, gzSum = 0;
    int32_t axSum = 0, aySum = 0, azSum = 0;

    printf("Calibrating MPU6500... keep sensor still and flat.\n");
    delay_ms(1000); // Wait a second for things to settle

    for (uint16_t i = 0; i < numSamples; i++)
    {
        mpu6500Update(mpu); // Grab the latest readings
       
        gxSum += mpu->rawData.gx;
        gySum += mpu->rawData.gy;
        gzSum += mpu->rawData.gz;

        axSum += mpu->rawData.ax;
        aySum += mpu->rawData.ay;
        azSum += mpu->rawData.az;

        delay_ms(5);  
    }

    // Compute averages 
    int16_t gxAvg = (int16_t)(gxSum / numSamples);
    int16_t gyAvg = (int16_t)(gySum / numSamples);
    int16_t gzAvg = (int16_t)(gzSum / numSamples);

    int16_t axAvg = (int16_t)(axSum / numSamples);
    int16_t ayAvg = (int16_t)(aySum / numSamples);
    int16_t azAvg = (int16_t)(azSum / numSamples);

    // Calculate offsets
    cal->gxOffset = -gxAvg;
    cal->gyOffset = -gyAvg;
    cal->gzOffset = -gzAvg;

    cal->axOffset = -axAvg;
    cal->ayOffset = -ayAvg;
    cal->azOffset = 16384 - azAvg; // Assuming 1G is 16384 at 2G scale

    // Write offsets to hardware registers 
    mpu6500SetGyroOffset(mpu, cal->gxOffset, cal->gyOffset, cal->gzOffset);
    mpu6500SetAccelOffset(mpu, cal->axOffset, cal->ayOffset, cal->azOffset);

    printf("Calibration complete. Offsets written to sensor.\n");
}
