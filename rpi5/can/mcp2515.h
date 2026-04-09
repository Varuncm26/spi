

#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>
#include "spi.h"           
#include "mcp2515_regs.h"   

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t cnf1;   /**< SJW[1:0] | BRP[5:0]                          */
    uint8_t cnf2;   /**< BTLMODE | SAM | PHSEG1[2:0] | PRSEG[2:0]     */
    uint8_t cnf3;   /**< SOF | WAKFIL | --- | PHSEG2[2:0]             */
} mcp2515_bittime_t;

/* Pre-computed presets (defined in mcp2515.c) */
extern const mcp2515_bittime_t MCP2515_BT_8MHz_1000kbps;
extern const mcp2515_bittime_t MCP2515_BT_8MHz_500kbps;
extern const mcp2515_bittime_t MCP2515_BT_8MHz_250kbps;
extern const mcp2515_bittime_t MCP2515_BT_8MHz_125kbps;
extern const mcp2515_bittime_t MCP2515_BT_8MHz_100kbps;
extern const mcp2515_bittime_t MCP2515_BT_8MHz_50kbps;
extern const mcp2515_bittime_t MCP2515_BT_16MHz_1000kbps;
extern const mcp2515_bittime_t MCP2515_BT_16MHz_500kbps;
extern const mcp2515_bittime_t MCP2515_BT_16MHz_250kbps;
extern const mcp2515_bittime_t MCP2515_BT_16MHz_125kbps;
extern const mcp2515_bittime_t MCP2515_BT_16MHz_100kbps;
extern const mcp2515_bittime_t MCP2515_BT_16MHz_50kbps;
extern const mcp2515_bittime_t MCP2515_BT_20MHz_1000kbps;
extern const mcp2515_bittime_t MCP2515_BT_20MHz_500kbps;
extern const mcp2515_bittime_t MCP2515_BT_20MHz_250kbps;
extern const mcp2515_bittime_t MCP2515_BT_20MHz_125kbps;



typedef enum {
    MCP2515_MODE_NORMAL   = 0x00,
    MCP2515_MODE_SLEEP    = 0x20,
    MCP2515_MODE_LOOPBACK = 0x40,
    MCP2515_MODE_LISTEN   = 0x60,
    MCP2515_MODE_CONFIG   = 0x80,
} mcp2515_mode_t;

typedef enum { MCP2515_TXB0=0, MCP2515_TXB1=1, MCP2515_TXB2=2 } mcp2515_txb_t;
typedef enum { MCP2515_RXB0=0, MCP2515_RXB1=1 }                  mcp2515_rxb_t;
typedef enum {
    MCP2515_RXF0=0, MCP2515_RXF1, MCP2515_RXF2,
    MCP2515_RXF3,   MCP2515_RXF4, MCP2515_RXF5
} mcp2515_filter_t;
typedef enum { MCP2515_RXM0=0, MCP2515_RXM1 } mcp2515_mask_t;

/** CAN message */
typedef struct {
    uint32_t id;        /**< 11-bit (standard) or 29-bit (extended) ID */
    bool     extended;  /**< true = extended frame                     */
    bool     rtr;       /**< true = Remote Transmission Request        */
    uint8_t  dlc;       /**< Data Length Code 0-8                      */
    uint8_t  data[8];   /**< Payload bytes                             */
} mcp2515_msg_t;


typedef struct {
    spi_t spi;   /**< Generic SPI handle (owns the file descriptor) */
} mcp2515_t;



#define MCP2515_OK          0
#define MCP2515_ERR_SPI    -1
#define MCP2515_ERR_MODE   -2
#define MCP2515_ERR_NODATA -3
#define MCP2515_ERR_DLC    -4
#define MCP2515_ERR_PARAM  -5



/* Initialisation */
int  mcp2515_init  (mcp2515_t *dev, const char *spi_dev,
                    uint32_t spi_hz, const mcp2515_bittime_t *bt);
void mcp2515_deinit(mcp2515_t *dev);

/* Low-level register access */
int mcp2515_read_reg  (mcp2515_t *dev, uint8_t addr, uint8_t *val);
int mcp2515_write_reg (mcp2515_t *dev, uint8_t addr, uint8_t  val);
int mcp2515_bit_modify(mcp2515_t *dev, uint8_t addr, uint8_t mask, uint8_t data);
int mcp2515_read_regs (mcp2515_t *dev, uint8_t addr, uint8_t *buf,       uint8_t len);
int mcp2515_write_regs(mcp2515_t *dev, uint8_t addr, const uint8_t *buf, uint8_t len);

/* Reset */
int mcp2515_reset(mcp2515_t *dev);

/* Operating mode */
int mcp2515_set_mode(mcp2515_t *dev, mcp2515_mode_t mode);
int mcp2515_get_mode(mcp2515_t *dev, mcp2515_mode_t *mode);

/* Bit timing */
int mcp2515_set_bittime    (mcp2515_t *dev, const mcp2515_bittime_t *bt);
int mcp2515_set_bittime_raw(mcp2515_t *dev, uint8_t brp, uint8_t sjw,
                             uint8_t prop_seg, uint8_t ps1, uint8_t ps2, bool sam3);

/* One-Shot mode */
int mcp2515_set_oneshot(mcp2515_t *dev, bool enable);

/* Transmit */
int mcp2515_transmit    (mcp2515_t *dev, mcp2515_txb_t txb,
                          const mcp2515_msg_t *msg, uint8_t priority);
int mcp2515_tx_pending  (mcp2515_t *dev, mcp2515_txb_t txb, bool *pending);
int mcp2515_tx_status   (mcp2515_t *dev, mcp2515_txb_t txb, uint8_t *ctrl);
int mcp2515_tx_abort    (mcp2515_t *dev, mcp2515_txb_t txb);
int mcp2515_tx_abort_all(mcp2515_t *dev);
int mcp2515_rts         (mcp2515_t *dev, uint8_t txb_mask);

/* Receive */
int mcp2515_rx_available(mcp2515_t *dev, bool *rx0, bool *rx1);
int mcp2515_receive     (mcp2515_t *dev, mcp2515_rxb_t rxb, mcp2515_msg_t *msg);
int mcp2515_receive_any (mcp2515_t *dev, mcp2515_msg_t *msg);
int mcp2515_set_rollover(mcp2515_t *dev, bool enable);

/* Filters / Masks  (device must be in Config mode) */
int mcp2515_set_filter          (mcp2515_t *dev, mcp2515_filter_t f, bool ext, uint32_t id);
int mcp2515_set_mask            (mcp2515_t *dev, mcp2515_mask_t m,   bool ext, uint32_t id);
int mcp2515_set_filter_accept_all(mcp2515_t *dev);

/* Interrupts */
int mcp2515_set_interrupts       (mcp2515_t *dev, uint8_t inte_mask);
int mcp2515_get_interrupt_enable (mcp2515_t *dev, uint8_t *inte);
int mcp2515_get_interrupt_flags  (mcp2515_t *dev, uint8_t *intf);
int mcp2515_clear_interrupt_flags(mcp2515_t *dev, uint8_t flag_mask);

/* Error counters / flags */
int mcp2515_get_tec             (mcp2515_t *dev, uint8_t *tec);
int mcp2515_get_rec             (mcp2515_t *dev, uint8_t *rec);
int mcp2515_get_eflg            (mcp2515_t *dev, uint8_t *eflg);
int mcp2515_clear_overflow_flags(mcp2515_t *dev);

/* CLKOUT */
int mcp2515_set_clkout(mcp2515_t *dev, bool enable, uint8_t prescaler);

/* Pin control */
int mcp2515_set_bfpctrl  (mcp2515_t *dev, uint8_t bfpctrl);
int mcp2515_set_txrtsctrl(mcp2515_t *dev, uint8_t txrtsctrl);

/* Quick-status commands */
int mcp2515_read_status(mcp2515_t *dev, uint8_t *status);
int mcp2515_rx_status  (mcp2515_t *dev, uint8_t *status);

/* Sleep / wake */
int mcp2515_sleep (mcp2515_t *dev);
int mcp2515_wakeup(mcp2515_t *dev);

/* Utility */
int  mcp2515_get_bittime(uint32_t fosc_mhz, uint32_t baud_kbps, mcp2515_bittime_t *bt);
void mcp2515_print_msg  (const mcp2515_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* MCP2515_H */
