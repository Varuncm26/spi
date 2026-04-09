
#include "spi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>



int spi_open(spi_t      *spi,
             const char *device,
             uint8_t     mode,
             uint32_t    speed_hz,
             uint8_t     bits_per_word)
{
    if (!spi || !device || bits_per_word == 0) {
        errno = EINVAL;
        return SPI_ERR_ARG;
    }

    spi->fd           = -1;
    spi->mode         = mode;
    spi->bits_per_word = bits_per_word;
    spi->speed_hz     = speed_hz;

    spi->fd = open(device, O_RDWR);
    if (spi->fd < 0) {
        fprintf(stderr, "spi_open: cannot open %s: %s\n",
                device, strerror(errno));
        return SPI_ERR_OPEN;
    }

   
    if (ioctl(spi->fd, SPI_IOC_WR_MODE, &spi->mode) < 0) {
        fprintf(stderr, "spi_open: SPI_IOC_WR_MODE: %s\n", strerror(errno));
        goto cfg_err;
    }
    
    if (ioctl(spi->fd, SPI_IOC_RD_MODE, &spi->mode) < 0) {
        fprintf(stderr, "spi_open: SPI_IOC_RD_MODE: %s\n", strerror(errno));
        goto cfg_err;
    }

    /* ------ bits per word ---------------------------------------- */
    if (ioctl(spi->fd, SPI_IOC_WR_BITS_PER_WORD, &spi->bits_per_word) < 0) {
        fprintf(stderr, "spi_open: SPI_IOC_WR_BITS_PER_WORD: %s\n",
                strerror(errno));
        goto cfg_err;
    }
    if (ioctl(spi->fd, SPI_IOC_RD_BITS_PER_WORD, &spi->bits_per_word) < 0) {
        fprintf(stderr, "spi_open: SPI_IOC_RD_BITS_PER_WORD: %s\n",
                strerror(errno));
        goto cfg_err;
    }

    
    if (ioctl(spi->fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi->speed_hz) < 0) {
        fprintf(stderr, "spi_open: SPI_IOC_WR_MAX_SPEED_HZ: %s\n",
                strerror(errno));
        goto cfg_err;
    }
    if (ioctl(spi->fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi->speed_hz) < 0) {
        fprintf(stderr, "spi_open: SPI_IOC_RD_MAX_SPEED_HZ: %s\n",
                strerror(errno));
        goto cfg_err;
    }

    return SPI_OK;

cfg_err:
    close(spi->fd);
    spi->fd = -1;
    return SPI_ERR_CFG;
}

void spi_close(spi_t *spi)
{
    if (spi && spi->fd >= 0) {
        close(spi->fd);
        spi->fd = -1;
    }
}


static int do_transfer(spi_t         *spi,
                       const uint8_t *tx,
                       uint8_t       *rx,
                       size_t         len)
{
    if (len == 0)  return SPI_OK;
    if (!spi || spi->fd < 0) return SPI_ERR_ARG;

    struct spi_ioc_transfer xfer;
    memset(&xfer, 0, sizeof(xfer));

    xfer.tx_buf        = (unsigned long)(uintptr_t)tx;  /* NULL == 0 → kernel sends zeroes */
    xfer.rx_buf        = (unsigned long)(uintptr_t)rx;  /* NULL == 0 → kernel discards     */
    xfer.len           = (uint32_t)len;
    xfer.speed_hz      = spi->speed_hz;
    xfer.bits_per_word = spi->bits_per_word;
    xfer.delay_usecs   = 0;
    xfer.cs_change     = 0;

    if (ioctl(spi->fd, SPI_IOC_MESSAGE(1), &xfer) < 0) {
        fprintf(stderr, "spi: transfer failed: %s\n", strerror(errno));
        return SPI_ERR_XFER;
    }
    return SPI_OK;
}


int spi_transfer(spi_t         *spi,
                 const uint8_t *tx_data,
                 uint8_t       *rx_data,
                 size_t         len)
{
    if (!spi) return SPI_ERR_ARG;
    return do_transfer(spi, tx_data, rx_data, len);
}

int spi_write(spi_t         *spi,
              const uint8_t *tx_data,
              size_t         len)
{
    if (!spi || !tx_data) return SPI_ERR_ARG;
    return do_transfer(spi, tx_data, NULL, len);
}

int spi_read(spi_t   *spi,
             uint8_t *rx_data,
             size_t   len)
{
    if (!spi || !rx_data) return SPI_ERR_ARG;
    return do_transfer(spi, NULL, rx_data, len);
}

int spi_write_then_read(spi_t         *spi,
                        uint8_t        cmd,
                        const uint8_t *tx_data,
                        size_t         tx_len,
                        uint8_t       *rx_data,
                        size_t         rx_len)
{
    if (!spi) return SPI_ERR_ARG;

    size_t   total  = 1 + tx_len + rx_len;
    uint8_t *tx_buf = (uint8_t *)calloc(total, 1);
    uint8_t *rx_buf = (uint8_t *)calloc(total, 1);

    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return SPI_ERR_ARG;
    }


    tx_buf[0] = cmd;
    if (tx_data && tx_len > 0)
        memcpy(tx_buf + 1, tx_data, tx_len);

    int ret = do_transfer(spi, tx_buf, rx_buf, total);

    if (ret == SPI_OK && rx_data && rx_len > 0)
        memcpy(rx_data, rx_buf + 1 + tx_len, rx_len);

    free(tx_buf);
    free(rx_buf);
    return ret;
}



const char *spi_strerror(int err)
{
    switch (err) {
    case SPI_OK:       return "Success";
    case SPI_ERR_OPEN: return "Cannot open spidev node";
    case SPI_ERR_CFG:  return "ioctl configuration failed";
    case SPI_ERR_XFER: return "SPI transfer failed";
    case SPI_ERR_ARG:  return "Invalid argument or NULL pointer";
    default:           return "Unknown SPI error";
    }
}
