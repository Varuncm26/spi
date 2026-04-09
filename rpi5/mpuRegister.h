#ifndef MPUREGISTER_H
#define MPUREGISTER_H


/* Self-Test Registers */
#define MPU6500_REG_SELF_TEST_X_GYRO    0x00
#define MPU6500_REG_SELF_TEST_Y_GYRO    0x01
#define MPU6500_REG_SELF_TEST_Z_GYRO    0x02
#define MPU6500_REG_SELF_TEST_X_ACCEL   0x0D
#define MPU6500_REG_SELF_TEST_Y_ACCEL   0x0E
#define MPU6500_REG_SELF_TEST_Z_ACCEL   0x0F

/* Gyro Offset Registers */
#define MPU6500_REG_XG_OFFSET_H         0x13
#define MPU6500_REG_XG_OFFSET_L         0x14
#define MPU6500_REG_YG_OFFSET_H         0x15
#define MPU6500_REG_YG_OFFSET_L         0x16
#define MPU6500_REG_ZG_OFFSET_H         0x17
#define MPU6500_REG_ZG_OFFSET_L         0x18

/* Sample Rate & Filter Config */
#define MPU6500_REG_SMPLRT_DIV          0x19
#define MPU6500_REG_CONFIG              0x1A
#define MPU6500_REG_GYRO_CONFIG         0x1B
#define MPU6500_REG_ACCEL_CONFIG        0x1C
#define MPU6500_REG_ACCEL_CONFIG2       0x1D
#define MPU6500_REG_LP_ACCEL_ODR        0x1E
#define MPU6500_REG_WOM_THR             0x1F

/* FIFO & I2C Master */
#define MPU6500_REG_FIFO_EN             0x23
#define MPU6500_REG_I2C_MST_CTRL        0x24

/* Interrupt Configuration */
#define MPU6500_REG_INT_PIN_CFG         0x37
#define MPU6500_REG_INT_ENABLE          0x38
#define MPU6500_REG_INT_STATUS          0x3A

/* Sensor Data – Accelerometer (read-only) */
#define MPU6500_REG_ACCEL_XOUT_H        0x3B
#define MPU6500_REG_ACCEL_XOUT_L        0x3C
#define MPU6500_REG_ACCEL_YOUT_H        0x3D
#define MPU6500_REG_ACCEL_YOUT_L        0x3E
#define MPU6500_REG_ACCEL_ZOUT_H        0x3F
#define MPU6500_REG_ACCEL_ZOUT_L        0x40

/* Sensor Data – Temperature (read-only) */
#define MPU6500_REG_TEMP_OUT_H          0x41
#define MPU6500_REG_TEMP_OUT_L          0x42

/* Sensor Data – Gyroscope (read-only) */
#define MPU6500_REG_GYRO_XOUT_H         0x43
#define MPU6500_REG_GYRO_XOUT_L         0x44
#define MPU6500_REG_GYRO_YOUT_H         0x45
#define MPU6500_REG_GYRO_YOUT_L         0x46
#define MPU6500_REG_GYRO_ZOUT_H         0x47
#define MPU6500_REG_GYRO_ZOUT_L         0x48

/* Signal Path & Control */
#define MPU6500_REG_I2C_MST_DELAY_CTRL  0x67
#define MPU6500_REG_SIGNAL_PATH_RESET   0x68
#define MPU6500_REG_ACCEL_INTEL_CTRL    0x69
#define MPU6500_REG_USER_CTRL           0x6A

/* Power Management */
#define MPU6500_REG_PWR_MGMT_1          0x6B
#define MPU6500_REG_PWR_MGMT_2          0x6C

/* FIFO Registers */
#define MPU6500_REG_FIFO_COUNT_H        0x72
#define MPU6500_REG_FIFO_COUNT_L        0x73
#define MPU6500_REG_FIFO_R_W            0x74

/* Device Identity */
#define MPU6500_REG_WHO_AM_I            0x75

/* Accel Offset Registers (MPU-6500 mode) */
#define MPU6500_REG_XA_OFFSET_H         0x77
#define MPU6500_REG_XA_OFFSET_L         0x78
#define MPU6500_REG_YA_OFFSET_H         0x7A
#define MPU6500_REG_YA_OFFSET_L         0x7B
#define MPU6500_REG_ZA_OFFSET_H         0x7D
#define MPU6500_REG_ZA_OFFSET_L         0x7E


/* PWR_MGMT_1 (0x6B) Bit Definitions */
#define MPU_BIT_DEVICE_RESET    (1 << 7)
#define MPU_BIT_SLEEP           (1 << 6)
#define MPU_BIT_CYCLE           (1 << 5)
#define MPU_BIT_GYRO_STANDBY    (1 << 4)
#define MPU_BIT_TEMP_DIS        (1 << 3)
#define MPU_CLKSEL_AUTO         (0x01) // Auto selects best available clock 

/* SIGNAL_PATH_RESET (0x68) Bit Definitions */
#define MPU_BIT_GYRO_RST        (1 << 2)
#define MPU_BIT_ACCEL_RST       (1 << 1)
#define MPU_BIT_TEMP_RST        (1 << 0)

/* USER_CTRL (0x6A) Bit Definitions */
#define MPU_BIT_DMP_EN          (1 << 7)
#define MPU_BIT_FIFO_EN         (1 << 6)
#define MPU_BIT_I2C_MST_EN      (1 << 5)
#define MPU_BIT_I2C_IF_DIS      (1 << 4)
#define MPU_BIT_DMP_RST         (1 << 3)
#define MPU_BIT_FIFO_RST        (1 << 2)
#define MPU_BIT_I2C_MST_RST     (1 << 1)
#define MPU_BIT_SIG_COND_RST    (1 << 0)

/* GYRO_CONFIG (0x1B) & ACCEL_CONFIG (0x1C) Bit Shifts */
#define MPU_GYRO_FS_SHIFT       3
#define MPU_ACCEL_FS_SHIFT      3

/* INT_PIN_CFG (0x37) Bit Definitions */
#define MPU_BIT_ACTL                (1 << 7) // 1: Active low, 0: Active high
#define MPU_BIT_OPEN                (1 << 6) // 1: Open drain, 0: Push-pull
#define MPU_BIT_LATCH_INT_EN        (1 << 5) // 1: Latch until cleared, 0: 50us pulse
#define MPU_BIT_INT_ANYRD_2CLEAR    (1 << 4) // 1: Clear on any read, 0: Clear only by reading IUS
#define MPU_BIT_ACTL_FSYNC          (1 << 3) 
#define MPU_BIT_FSYNC_INT_MODE_EN   (1 << 2)
#define MPU_BIT_BYPASS_EN           (1 << 1)

/* INT_ENABLE (0x38) Bit Definitions */
#define MPU_BIT_WOM_EN              (1 << 6) // Wake on motion
#define MPU_BIT_FIFO_OFLOW_EN       (1 << 4) // FIFO overflow
#define MPU_BIT_FSYNC_INT_EN        (1 << 3) // FSYNC interrupt
#define MPU_BIT_RAW_RDY_EN          (1 << 0) // Raw Sensor Data Ready

#define MPU_BIT_RAW_DATA_RDY_INT    (1 << 0)
#endif
