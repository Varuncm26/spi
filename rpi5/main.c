#include <stdio.h>
#include <unistd.h>
#include "spi.h"
#include "mpu6500.h"

int main() {
    spi_t mySpi;
    mpu6500_t myImu;

    // Open SPI Bus 0, Chip Select 0 at 1MHz, SPI Mode 0
    // Adjust /dev/spidevX.X based on your Pi or Jetson setup
    if (spi_open(&mySpi, "/dev/spidev0.0", SPI_MODE_0, 1000000, 8) != SPI_OK) {
        return -1;
    }

    // Initialize the MPU passing the SPI context
    if (!mpu6500Init(&myImu, &mySpi, GYRO_FS_2000DPS, ACCEL_FS_8G, 0)) {
        spi_close(&mySpi);
        return -1;
    }

    // Main polling loop
    while(1) {
        // Fetch all 14 bytes in one blocking SPI call
        mpu6500Update(&myImu);
        
        printf("Accel [g]: X: %.2f Y: %.2f Z: %.2f | Gyro [dps]: X: %.2f Y: %.2f Z: %.2f\n", 
               myImu.mpuData.ax, myImu.mpuData.ay, myImu.mpuData.az,
               myImu.mpuData.gx, myImu.mpuData.gy, myImu.mpuData.gz);
               
        usleep(50000); // 50ms loop = 20Hz polling
    }

    spi_close(&mySpi);
    return 0;
}
