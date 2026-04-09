
#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//spi mode
#define SPI_MODE_0   0   /**< CPOL=0 CPHA=0 */
#define SPI_MODE_1   1   /**< CPOL=0 CPHA=1 */
#define SPI_MODE_2   2   /**< CPOL=1 CPHA=0 */
#define SPI_MODE_3   3   /**< CPOL=1 CPHA=1 */


typedef struct {
    int      fd;           /**< File descriptor returned by open()     */
    uint8_t  mode;         /**< SPI mode (0-3)                         */
    uint8_t  bits_per_word;/**< Bits per word (typically 8)            */
    uint32_t speed_hz;     /**< Clock frequency in Hz                  */
} spi_t;


#define SPI_OK        0
#define SPI_ERR_OPEN -1   /**< Could not open device node              */
#define SPI_ERR_CFG  -2   /**< ioctl configuration failed              */
#define SPI_ERR_XFER -3   /**< Transfer (SPI_IOC_MESSAGE) failed       */
#define SPI_ERR_ARG  -4   /**< NULL pointer or invalid argument        */


int spi_open(spi_t      *spi,
             const char *device,
             uint8_t     mode,
             uint32_t    speed_hz,
             uint8_t     bits_per_word);


void spi_close(spi_t *spi);


int spi_transfer(spi_t         *spi,
                 const uint8_t *tx_data,
                 uint8_t       *rx_data,
                 size_t         len);


int spi_write(spi_t         *spi,
              const uint8_t *tx_data,
              size_t         len);


int spi_read(spi_t   *spi,
             uint8_t *rx_data,
             size_t   len);


int spi_write_then_read(spi_t         *spi,
                        uint8_t        cmd,
                        const uint8_t *tx_data,
                        size_t         tx_len,
                        uint8_t       *rx_data,
                        size_t         rx_len);


const char *spi_strerror(int err);

#ifdef __cplusplus
}
#endif

#endif /* SPI_H */
