/**
 * @file mcp2515.c
 * @brief MCP2515 CAN controller implementation.
 *
 * All SPI communication goes through the generic spi.h layer.
 * This file builds the correct MCP2515 byte sequences and passes
 * raw data buffers to spi_write(), spi_transfer(), and
 * spi_write_then_read() – the SPI layer is completely unaware of CAN.
 *
 * Datasheet: Microchip DS20001801J
 */

#include "mcp2515.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>   /* usleep */

/* ================================================================== */
/*  Bit-timing preset tables                                            */
/*                                                                      */
/*  Encoding (all fields 0-based):                                      */
/*    CNF1 = (SJW-1)<<6 | (BRP-1)                                      */
/*    CNF2 = 0x80 (BTLMODE) | SAM<<6 | (PS1-1)<<3 | (PropSeg-1)       */
/*    CNF3 = PS2-1                                                       */
/* ================================================================== */

#define _CNF1(sjw, brp)          (uint8_t)((((sjw)-1)<<6)|((brp)-1))
#define _CNF2(sam, ps1, prseg)   (uint8_t)(0x80|(((sam)?1:0)<<6)|(((ps1)-1)<<3)|((prseg)-1))
#define _CNF3(ps2)               (uint8_t)((ps2)-1)

/* 8 MHz */
const mcp2515_bittime_t MCP2515_BT_8MHz_1000kbps = { _CNF1(1,1), _CNF2(0,3,2), _CNF3(2) };
const mcp2515_bittime_t MCP2515_BT_8MHz_500kbps  = { _CNF1(1,1), _CNF2(0,8,3), _CNF3(4) };
const mcp2515_bittime_t MCP2515_BT_8MHz_250kbps  = { _CNF1(1,2), _CNF2(0,8,3), _CNF3(4) };
const mcp2515_bittime_t MCP2515_BT_8MHz_125kbps  = { _CNF1(1,4), _CNF2(0,8,3), _CNF3(4) };
const mcp2515_bittime_t MCP2515_BT_8MHz_100kbps  = { _CNF1(1,5), _CNF2(0,8,3), _CNF3(4) };
const mcp2515_bittime_t MCP2515_BT_8MHz_50kbps   = { _CNF1(1,10),_CNF2(0,8,3), _CNF3(4) };

/* 16 MHz */
const mcp2515_bittime_t MCP2515_BT_16MHz_1000kbps = { _CNF1(1,1), _CNF2(0,3,2), _CNF3(2) };
const mcp2515_bittime_t MCP2515_BT_16MHz_500kbps  = { _CNF1(1,2), _CNF2(0,8,3), _CNF3(4) };
const mcp2515_bittime_t MCP2515_BT_16MHz_250kbps  = { _CNF1(1,4), _CNF2(0,8,3), _CNF3(4) };
const mcp2515_bittime_t MCP2515_BT_16MHz_125kbps  = { _CNF1(1,8), _CNF2(0,8,3), _CNF3(4) };
const mcp2515_bittime_t MCP2515_BT_16MHz_100kbps  = { _CNF1(1,10),_CNF2(0,8,3), _CNF3(4) };
const mcp2515_bittime_t MCP2515_BT_16MHz_50kbps   = { _CNF1(1,20),_CNF2(0,8,3), _CNF3(4) };

/* 20 MHz */
const mcp2515_bittime_t MCP2515_BT_20MHz_1000kbps = { _CNF1(1,1), _CNF2(0,5,2), _CNF3(2) };
const mcp2515_bittime_t MCP2515_BT_20MHz_500kbps  = { _CNF1(1,2), _CNF2(0,5,2), _CNF3(2) };
const mcp2515_bittime_t MCP2515_BT_20MHz_250kbps  = { _CNF1(1,4), _CNF2(0,5,2), _CNF3(2) };
const mcp2515_bittime_t MCP2515_BT_20MHz_125kbps  = { _CNF1(1,8), _CNF2(0,5,2), _CNF3(2) };

/* ================================================================== */
/*  Internal address tables                                             */
/* ================================================================== */

static const uint8_t txb_ctrl[3] = { MCP2515_REG_TXB0CTRL, MCP2515_REG_TXB1CTRL, MCP2515_REG_TXB2CTRL };
static const uint8_t txb_sidh[3] = { MCP2515_REG_TXB0SIDH, MCP2515_REG_TXB1SIDH, MCP2515_REG_TXB2SIDH };
static const uint8_t txb_d0  [3] = { MCP2515_REG_TXB0D0,   MCP2515_REG_TXB1D0,   MCP2515_REG_TXB2D0   };
static const uint8_t rxb_sidh[2] = { MCP2515_REG_RXB0SIDH, MCP2515_REG_RXB1SIDH };
static const uint8_t rxb_d0  [2] = { MCP2515_REG_RXB0D0,   MCP2515_REG_RXB1D0   };
static const uint8_t rxb_if  [2] = { MCP2515_CANINTF_RX0IF, MCP2515_CANINTF_RX1IF };
static const uint8_t rxf_sidh[6] = {
    MCP2515_REG_RXF0SIDH, MCP2515_REG_RXF1SIDH, MCP2515_REG_RXF2SIDH,
    MCP2515_REG_RXF3SIDH, MCP2515_REG_RXF4SIDH, MCP2515_REG_RXF5SIDH
};
static const uint8_t rxm_sidh[2] = { MCP2515_REG_RXM0SIDH, MCP2515_REG_RXM1SIDH };

/* ================================================================== */
/*  Internal helpers                                                    */
/* ================================================================== */

static inline void delay_us(unsigned int us) { usleep(us); }

/**
 * Pack a CAN ID into the four MCP2515 identifier register bytes.
 * Layout: SIDH, SIDL, EID8, EID0  (same for TX filters and masks).
 */
static void id_to_regs(bool ext, uint32_t id,
                        uint8_t *sidh, uint8_t *sidl,
                        uint8_t *eid8, uint8_t *eid0)
{
    if (ext) {
        *sidh = (uint8_t)((id >> 21) & 0xFF);
        *sidl = (uint8_t)(((id >> 13) & 0xE0)   /* SID[2:0] in bits 7:5 */
                        | 0x08                   /* EXIDE               */
                        | ((id >> 16) & 0x03));  /* EID[17:16]          */
        *eid8 = (uint8_t)((id >>  8) & 0xFF);
        *eid0 = (uint8_t)( id        & 0xFF);
    } else {
        *sidh = (uint8_t)((id >> 3) & 0xFF);
        *sidl = (uint8_t)((id & 0x07) << 5);
        *eid8 = 0x00;
        *eid0 = 0x00;
    }
}

/**
 * Decode MCP2515 identifier bytes from a received message.
 * Returns true if the message was an extended frame.
 */
static bool regs_to_id(uint8_t sidh, uint8_t sidl,
                        uint8_t eid8, uint8_t eid0,
                        uint32_t *id)
{
    bool ext = (sidl & MCP2515_RXBSIDL_IDE) != 0;
    if (ext) {
        *id = ((uint32_t)(sidh)        << 21)
            | ((uint32_t)(sidl & 0xE0) << 13)
            | ((uint32_t)(sidl & 0x03) << 16)
            | ((uint32_t) eid8         <<  8)
            | ((uint32_t) eid0);
    } else {
        *id = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
    }
    return ext;
}

/* Translate generic SPI return code to MCP2515 error code */
static inline int spi_ret(int r) { return (r == SPI_OK) ? MCP2515_OK : MCP2515_ERR_SPI; }

/* ================================================================== */
/*  Low-level register access                                           */
/*                                                                      */
/*  Every function assembles a byte array and calls the generic SPI     */
/*  layer with that array as the data argument.                         */
/* ================================================================== */

int mcp2515_read_reg(mcp2515_t *dev, uint8_t addr, uint8_t *val)
{
    /*
     * MCP2515 READ: [0x03] [addr] [dummy] → [xx] [xx] [data]
     * We use spi_write_then_read which sends cmd + optional tx bytes
     * then clocks in the response, all in one CS assertion.
     */
    return spi_ret(spi_write_then_read(&dev->spi,
                                        MCP2515_CMD_READ,
                                        &addr, 1,     /* tx payload: address byte */
                                        val,   1));   /* rx: register value       */
}

int mcp2515_write_reg(mcp2515_t *dev, uint8_t addr, uint8_t val)
{
    /*
     * MCP2515 WRITE: [0x02] [addr] [data]
     * Pack all three bytes into one array and pass to spi_write().
     */
    uint8_t frame[3] = { MCP2515_CMD_WRITE, addr, val };
    return spi_ret(spi_write(&dev->spi, frame, sizeof(frame)));
}

int mcp2515_bit_modify(mcp2515_t *dev, uint8_t addr, uint8_t mask, uint8_t data)
{
    /*
     * MCP2515 BIT MODIFY: [0x05] [addr] [mask] [data]
     */
    uint8_t frame[4] = { MCP2515_CMD_BIT_MODIFY, addr, mask, data };
    return spi_ret(spi_write(&dev->spi, frame, sizeof(frame)));
}

int mcp2515_read_regs(mcp2515_t *dev, uint8_t addr, uint8_t *buf, uint8_t len)
{
    /*
     * Sequential READ: [0x03] [start_addr] [dummy * len]
     * MCP2515 auto-increments the address pointer (DS §12.3).
     * spi_write_then_read sends cmd + addr, then clocks in len bytes.
     */
    if (len == 0 || len > 64) return MCP2515_ERR_PARAM;
    return spi_ret(spi_write_then_read(&dev->spi,
                                        MCP2515_CMD_READ,
                                        &addr, 1,
                                        buf,   len));
}

int mcp2515_write_regs(mcp2515_t *dev, uint8_t addr, const uint8_t *buf, uint8_t len)
{
    /*
     * Sequential WRITE: [0x02] [start_addr] [d0] [d1] ... [dn]
     * Build one contiguous frame and hand the whole thing to spi_write().
     */
    if (len == 0 || len > 64 || !buf) return MCP2515_ERR_PARAM;

    uint8_t frame[2 + 64];
    frame[0] = MCP2515_CMD_WRITE;
    frame[1] = addr;
    memcpy(frame + 2, buf, len);

    return spi_ret(spi_write(&dev->spi, frame, (size_t)(2 + len)));
}

/* ================================================================== */
/*  Reset                                                               */
/* ================================================================== */

int mcp2515_reset(mcp2515_t *dev)
{
    /*
     * RESET: single byte [0xC0]  (DS §12.2)
     */
    uint8_t cmd = MCP2515_CMD_RESET;
    int ret = spi_ret(spi_write(&dev->spi, &cmd, 1));
    if (ret != MCP2515_OK) return ret;

    /* Oscillator start-up timer: 128 OSC cycles.
       At 8 MHz = 16 µs; use 2 ms to be safe across all crystal speeds. */
    delay_us(2000);
    return MCP2515_OK;
}

/* ================================================================== */
/*  Operating mode                                                      */
/* ================================================================== */

int mcp2515_set_mode(mcp2515_t *dev, mcp2515_mode_t mode)
{
    int ret = mcp2515_bit_modify(dev,
                                  MCP2515_REG_CANCTRL,
                                  MCP2515_CANCTRL_REQOP_MASK,
                                  (uint8_t)mode);
    if (ret != MCP2515_OK) return ret;

    /* Poll CANSTAT.OPMOD until the transition completes (DS §10) */
    uint8_t stat;
    int tries = 200;
    do {
        delay_us(100);
        if (mcp2515_read_reg(dev, MCP2515_REG_CANSTAT, &stat) != MCP2515_OK)
            return MCP2515_ERR_SPI;
        if ((stat & MCP2515_CANSTAT_OPMOD_MASK) == (uint8_t)mode)
            return MCP2515_OK;
    } while (--tries > 0);

    return MCP2515_ERR_MODE;
}

int mcp2515_get_mode(mcp2515_t *dev, mcp2515_mode_t *mode)
{
    uint8_t val;
    int ret = mcp2515_read_reg(dev, MCP2515_REG_CANSTAT, &val);
    if (ret != MCP2515_OK) return ret;
    *mode = (mcp2515_mode_t)(val & MCP2515_CANSTAT_OPMOD_MASK);
    return MCP2515_OK;
}

/* ================================================================== */
/*  Bit timing                                                          */
/* ================================================================== */

int mcp2515_set_bittime(mcp2515_t *dev, const mcp2515_bittime_t *bt)
{
    if (!bt) return MCP2515_ERR_PARAM;
    /* Write CNF3, CNF2, CNF1 as three consecutive registers starting at 0x28 */
    uint8_t buf[3] = { bt->cnf3, bt->cnf2, bt->cnf1 };
    return mcp2515_write_regs(dev, MCP2515_REG_CNF3, buf, 3);
}

int mcp2515_set_bittime_raw(mcp2515_t *dev,
                             uint8_t brp, uint8_t sjw,
                             uint8_t prop_seg, uint8_t ps1, uint8_t ps2,
                             bool sam3)
{
    if (brp<1||brp>64||sjw<1||sjw>4||prop_seg<1||prop_seg>8||
        ps1<1||ps1>8||ps2<2||ps2>8)
        return MCP2515_ERR_PARAM;

    mcp2515_bittime_t bt;
    bt.cnf1 = _CNF1(sjw, brp);
    bt.cnf2 = _CNF2(sam3, ps1, prop_seg);
    bt.cnf3 = _CNF3(ps2);
    return mcp2515_set_bittime(dev, &bt);
}

/* ================================================================== */
/*  Initialisation                                                      */
/* ================================================================== */

int mcp2515_init(mcp2515_t              *dev,
                 const char             *spi_dev,
                 uint32_t                spi_hz,
                 const mcp2515_bittime_t *bt)
{
    if (!dev || !spi_dev || !bt) return MCP2515_ERR_PARAM;

    /*
     * Open the SPI device.  Mode 0,0 is used; MCP2515 also supports 1,1.
     * 8 bits per word is standard. The speed and path are caller-supplied.
     */
    if (spi_open(&dev->spi, spi_dev, SPI_MODE_0, spi_hz, 8) != SPI_OK)
        return MCP2515_ERR_SPI;

    /* Software reset → Config mode */
    int ret = mcp2515_reset(dev);
    if (ret != MCP2515_OK) goto fail;

    /* Verify Config mode */
    mcp2515_mode_t mode;
    ret = mcp2515_get_mode(dev, &mode);
    if (ret != MCP2515_OK || mode != MCP2515_MODE_CONFIG) {
        ret = MCP2515_ERR_MODE;
        goto fail;
    }

    /* Apply bit timing */
    ret = mcp2515_set_bittime(dev, bt);
    if (ret != MCP2515_OK) goto fail;

    /* Clear all interrupt flags */
    ret = mcp2515_write_reg(dev, MCP2515_REG_CANINTF, 0x00);
    if (ret != MCP2515_OK) goto fail;

    /* Disable all interrupts by default (caller enables what it needs) */
    ret = mcp2515_write_reg(dev, MCP2515_REG_CANINTE, 0x00);
    if (ret != MCP2515_OK) goto fail;

    /* Accept every message (masks = 0 → no bit is checked, all pass) */
    ret = mcp2515_set_filter_accept_all(dev);
    if (ret != MCP2515_OK) goto fail;

    /* RXB0: accept any frame, rollover to RXB1 on overflow */
    ret = mcp2515_write_reg(dev, MCP2515_REG_RXB0CTRL,
                             MCP2515_RXB0CTRL_BUKT | MCP2515_RXB0CTRL_RXM_ANY);
    if (ret != MCP2515_OK) goto fail;

    /* RXB1: accept any frame */
    ret = mcp2515_write_reg(dev, MCP2515_REG_RXB1CTRL,
                             MCP2515_RXB1CTRL_RXM_ANY);
    if (ret != MCP2515_OK) goto fail;

    /* RXnBF pins: disabled (high-impedance) */
    ret = mcp2515_write_reg(dev, MCP2515_REG_BFPCTRL, 0x00);
    if (ret != MCP2515_OK) goto fail;

    /* TXnRTS pins: digital inputs */
    ret = mcp2515_write_reg(dev, MCP2515_REG_TXRTSCTRL, 0x00);
    if (ret != MCP2515_OK) goto fail;

    /* Enter Normal operating mode */
    ret = mcp2515_set_mode(dev, MCP2515_MODE_NORMAL);
    if (ret != MCP2515_OK) goto fail;

    return MCP2515_OK;

fail:
    spi_close(&dev->spi);
    return ret;
}

void mcp2515_deinit(mcp2515_t *dev)
{
    if (dev) {
        mcp2515_sleep(dev);
        spi_close(&dev->spi);
    }
}

/* ================================================================== */
/*  One-Shot mode                                                       */
/* ================================================================== */

int mcp2515_set_oneshot(mcp2515_t *dev, bool enable)
{
    return mcp2515_bit_modify(dev,
                               MCP2515_REG_CANCTRL,
                               MCP2515_CANCTRL_OSM,
                               enable ? MCP2515_CANCTRL_OSM : 0x00);
}

/* ================================================================== */
/*  Transmit                                                            */
/* ================================================================== */

int mcp2515_transmit(mcp2515_t           *dev,
                     mcp2515_txb_t        txb,
                     const mcp2515_msg_t *msg,
                     uint8_t              priority)
{
    if (!dev || !msg || txb > MCP2515_TXB2) return MCP2515_ERR_PARAM;
    if (msg->dlc > 8) return MCP2515_ERR_DLC;

    /* --- Build the 5-byte identifier block: SIDH, SIDL, EID8, EID0, DLC --- */
    uint8_t id_block[5];
    id_to_regs(msg->extended, msg->id,
               &id_block[0], &id_block[1], &id_block[2], &id_block[3]);
    id_block[4] = (msg->dlc & MCP2515_TXBDLC_DLC_MASK)
                | (msg->rtr ? MCP2515_TXBDLC_RTR : 0);

    /* Write SIDH..DLC in one sequential write (5 bytes starting at TXBnSIDH) */
    int ret = mcp2515_write_regs(dev, txb_sidh[txb], id_block, 5);
    if (ret != MCP2515_OK) return ret;

    /* Write data bytes (skip for RTR frames or zero-length messages) */
    if (!msg->rtr && msg->dlc > 0) {
        ret = mcp2515_write_regs(dev, txb_d0[txb], msg->data, msg->dlc);
        if (ret != MCP2515_OK) return ret;
    }

    /* Set TXREQ + priority – ABTF/MLOA/TXERR are cleared automatically */
    uint8_t ctrl = MCP2515_TXBCTRL_TXREQ | (priority & MCP2515_TXBCTRL_TXP_MASK);
    return mcp2515_write_reg(dev, txb_ctrl[txb], ctrl);
}

int mcp2515_tx_pending(mcp2515_t *dev, mcp2515_txb_t txb, bool *pending)
{
    if (txb > MCP2515_TXB2 || !pending) return MCP2515_ERR_PARAM;
    uint8_t ctrl;
    int ret = mcp2515_read_reg(dev, txb_ctrl[txb], &ctrl);
    if (ret != MCP2515_OK) return ret;
    *pending = (ctrl & MCP2515_TXBCTRL_TXREQ) != 0;
    return MCP2515_OK;
}

int mcp2515_tx_status(mcp2515_t *dev, mcp2515_txb_t txb, uint8_t *ctrl)
{
    if (txb > MCP2515_TXB2 || !ctrl) return MCP2515_ERR_PARAM;
    return mcp2515_read_reg(dev, txb_ctrl[txb], ctrl);
}

int mcp2515_tx_abort(mcp2515_t *dev, mcp2515_txb_t txb)
{
    if (txb > MCP2515_TXB2) return MCP2515_ERR_PARAM;
    return mcp2515_bit_modify(dev, txb_ctrl[txb], MCP2515_TXBCTRL_TXREQ, 0x00);
}

int mcp2515_tx_abort_all(mcp2515_t *dev)
{
    /* Set ABAT in CANCTRL */
    int ret = mcp2515_bit_modify(dev, MCP2515_REG_CANCTRL,
                                  MCP2515_CANCTRL_ABAT, MCP2515_CANCTRL_ABAT);
    if (ret != MCP2515_OK) return ret;

    /* Wait for all TXREQ bits to clear */
    for (int i = 0; i < 100; i++) {
        uint8_t c0, c1, c2;
        mcp2515_read_reg(dev, MCP2515_REG_TXB0CTRL, &c0);
        mcp2515_read_reg(dev, MCP2515_REG_TXB1CTRL, &c1);
        mcp2515_read_reg(dev, MCP2515_REG_TXB2CTRL, &c2);
        if (!((c0|c1|c2) & MCP2515_TXBCTRL_TXREQ)) break;
        delay_us(500);
    }

    /* Clear ABAT */
    return mcp2515_bit_modify(dev, MCP2515_REG_CANCTRL,
                               MCP2515_CANCTRL_ABAT, 0x00);
}

int mcp2515_rts(mcp2515_t *dev, uint8_t txb_mask)
{
    /*
     * RTS command: single byte [1000 0nnn] where nnn selects TXB2/1/0.
     * Pass the byte directly to spi_write().
     */
    uint8_t cmd = MCP2515_CMD_RTS(txb_mask & 0x07);
    return spi_ret(spi_write(&dev->spi, &cmd, 1));
}

/* ================================================================== */
/*  Receive                                                             */
/* ================================================================== */

int mcp2515_rx_available(mcp2515_t *dev, bool *rx0, bool *rx1)
{
    uint8_t status;
    int ret = mcp2515_read_status(dev, &status);
    if (ret != MCP2515_OK) return ret;
    /* READ STATUS bit 0 = RX0IF, bit 1 = RX1IF  (DS §12.8) */
    if (rx0) *rx0 = (status & 0x01) != 0;
    if (rx1) *rx1 = (status & 0x02) != 0;
    return MCP2515_OK;
}

int mcp2515_receive(mcp2515_t *dev, mcp2515_rxb_t rxb, mcp2515_msg_t *msg)
{
    if (!msg || rxb > MCP2515_RXB1) return MCP2515_ERR_PARAM;

    /* Is there a message waiting? */
    uint8_t intf;
    int ret = mcp2515_read_reg(dev, MCP2515_REG_CANINTF, &intf);
    if (ret != MCP2515_OK) return ret;
    if (!(intf & rxb_if[rxb])) return MCP2515_ERR_NODATA;

    /* Read SIDH, SIDL, EID8, EID0, DLC (5 bytes) in one transaction */
    uint8_t id_block[5];
    ret = mcp2515_read_regs(dev, rxb_sidh[rxb], id_block, 5);
    if (ret != MCP2515_OK) return ret;

    /* Decode ID and frame type */
    msg->extended = regs_to_id(id_block[0], id_block[1],
                                id_block[2], id_block[3], &msg->id);

    uint8_t dlc_byte = id_block[4];
    msg->dlc = dlc_byte & MCP2515_RXBDLC_DLC_MASK;
    if (msg->dlc > 8) msg->dlc = 8;

    /* RTR flag: extended frames use the DLC register's RTR bit;
       standard frames use the SRR bit in SIDL. */
    msg->rtr = msg->extended
             ? ((dlc_byte  & MCP2515_RXBDLC_RTR) != 0)
             : ((id_block[1] & MCP2515_RXBSIDL_SRR) != 0);

    /* Read data bytes */
    memset(msg->data, 0, 8);
    if (!msg->rtr && msg->dlc > 0) {
        ret = mcp2515_read_regs(dev, rxb_d0[rxb], msg->data, msg->dlc);
        if (ret != MCP2515_OK) return ret;
    }

    /* Clear the RXnIF flag using BIT MODIFY (DS §7 recommended approach) */
    return mcp2515_bit_modify(dev, MCP2515_REG_CANINTF, rxb_if[rxb], 0x00);
}

int mcp2515_receive_any(mcp2515_t *dev, mcp2515_msg_t *msg)
{
    bool r0, r1;
    int ret = mcp2515_rx_available(dev, &r0, &r1);
    if (ret != MCP2515_OK) return ret;
    if (r0) return mcp2515_receive(dev, MCP2515_RXB0, msg);
    if (r1) return mcp2515_receive(dev, MCP2515_RXB1, msg);
    return MCP2515_ERR_NODATA;
}

int mcp2515_set_rollover(mcp2515_t *dev, bool enable)
{
    return mcp2515_bit_modify(dev,
                               MCP2515_REG_RXB0CTRL,
                               MCP2515_RXB0CTRL_BUKT,
                               enable ? MCP2515_RXB0CTRL_BUKT : 0x00);
}

/* ================================================================== */
/*  Filters and Masks                                                   */
/* ================================================================== */

int mcp2515_set_filter(mcp2515_t *dev, mcp2515_filter_t f, bool ext, uint32_t id)
{
    if (f > MCP2515_RXF5) return MCP2515_ERR_PARAM;
    uint8_t regs[4];
    id_to_regs(ext, id, &regs[0], &regs[1], &regs[2], &regs[3]);
    return mcp2515_write_regs(dev, rxf_sidh[f], regs, 4);
}

int mcp2515_set_mask(mcp2515_t *dev, mcp2515_mask_t m, bool ext, uint32_t id)
{
    if (m > MCP2515_RXM1) return MCP2515_ERR_PARAM;
    uint8_t regs[4];
    id_to_regs(ext, id, &regs[0], &regs[1], &regs[2], &regs[3]);
    return mcp2515_write_regs(dev, rxm_sidh[m], regs, 4);
}

int mcp2515_set_filter_accept_all(mcp2515_t *dev)
{
    /* Mask bits = 0 → every identifier bit is ignored → accept all. */
    static const uint8_t zero4[4] = {0, 0, 0, 0};
    int ret = mcp2515_write_regs(dev, MCP2515_REG_RXM0SIDH, zero4, 4);
    if (ret != MCP2515_OK) return ret;
    return mcp2515_write_regs(dev, MCP2515_REG_RXM1SIDH, zero4, 4);
}

/* ================================================================== */
/*  Interrupts                                                          */
/* ================================================================== */

int mcp2515_set_interrupts(mcp2515_t *dev, uint8_t inte_mask)
{
    return mcp2515_write_reg(dev, MCP2515_REG_CANINTE, inte_mask);
}

int mcp2515_get_interrupt_enable(mcp2515_t *dev, uint8_t *inte)
{
    return mcp2515_read_reg(dev, MCP2515_REG_CANINTE, inte);
}

int mcp2515_get_interrupt_flags(mcp2515_t *dev, uint8_t *intf)
{
    return mcp2515_read_reg(dev, MCP2515_REG_CANINTF, intf);
}

int mcp2515_clear_interrupt_flags(mcp2515_t *dev, uint8_t flag_mask)
{
    return mcp2515_bit_modify(dev, MCP2515_REG_CANINTF, flag_mask, 0x00);
}

/* ================================================================== */
/*  Error counters / flags                                              */
/* ================================================================== */

int mcp2515_get_tec(mcp2515_t *dev, uint8_t *tec) { return mcp2515_read_reg(dev, MCP2515_REG_TEC, tec); }
int mcp2515_get_rec(mcp2515_t *dev, uint8_t *rec) { return mcp2515_read_reg(dev, MCP2515_REG_REC, rec); }
int mcp2515_get_eflg(mcp2515_t *dev, uint8_t *e)  { return mcp2515_read_reg(dev, MCP2515_REG_EFLG, e); }

int mcp2515_clear_overflow_flags(mcp2515_t *dev)
{
    return mcp2515_bit_modify(dev, MCP2515_REG_EFLG,
                               MCP2515_EFLG_RX0OVR | MCP2515_EFLG_RX1OVR, 0x00);
}

/* ================================================================== */
/*  CLKOUT                                                              */
/* ================================================================== */

int mcp2515_set_clkout(mcp2515_t *dev, bool enable, uint8_t prescaler)
{
    uint8_t mask = MCP2515_CANCTRL_CLKEN | MCP2515_CANCTRL_CLKPRE_MASK;
    uint8_t data = (prescaler & MCP2515_CANCTRL_CLKPRE_MASK)
                 | (enable ? MCP2515_CANCTRL_CLKEN : 0x00);
    return mcp2515_bit_modify(dev, MCP2515_REG_CANCTRL, mask, data);
}

/* ================================================================== */
/*  Pin control                                                         */
/* ================================================================== */

int mcp2515_set_bfpctrl  (mcp2515_t *dev, uint8_t v) { return mcp2515_write_reg(dev, MCP2515_REG_BFPCTRL,   v); }
int mcp2515_set_txrtsctrl(mcp2515_t *dev, uint8_t v) { return mcp2515_write_reg(dev, MCP2515_REG_TXRTSCTRL, v); }

/* ================================================================== */
/*  Quick-status commands                                               */
/* ================================================================== */

int mcp2515_read_status(mcp2515_t *dev, uint8_t *status)
{
    /*
     * READ STATUS: [0xA0] → [status byte]
     * Pass cmd=READ_STATUS, no tx payload, 1 rx byte.
     */
    return spi_ret(spi_write_then_read(&dev->spi,
                                        MCP2515_CMD_READ_STATUS,
                                        NULL, 0,
                                        status, 1));
}

int mcp2515_rx_status(mcp2515_t *dev, uint8_t *status)
{
    /*
     * RX STATUS: [0xB0] → [status byte]
     */
    return spi_ret(spi_write_then_read(&dev->spi,
                                        MCP2515_CMD_RX_STATUS,
                                        NULL, 0,
                                        status, 1));
}

/* ================================================================== */
/*  Sleep / wake                                                        */
/* ================================================================== */

int mcp2515_sleep(mcp2515_t *dev)
{
    mcp2515_bit_modify(dev, MCP2515_REG_CANINTE,
                        MCP2515_CANINTE_WAKIE, MCP2515_CANINTE_WAKIE);
    return mcp2515_set_mode(dev, MCP2515_MODE_SLEEP);
}

int mcp2515_wakeup(mcp2515_t *dev)
{
    mcp2515_bit_modify(dev, MCP2515_REG_CANINTF, MCP2515_CANINTF_WAKIF, 0x00);
    return mcp2515_set_mode(dev, MCP2515_MODE_NORMAL);
}

/* ================================================================== */
/*  Utility                                                             */
/* ================================================================== */

int mcp2515_get_bittime(uint32_t fosc_mhz, uint32_t baud_kbps, mcp2515_bittime_t *bt)
{
    if (!bt) return MCP2515_ERR_PARAM;

    typedef struct { uint32_t f; uint32_t b; const mcp2515_bittime_t *p; } lut_t;
    static const lut_t lut[] = {
        {  8,1000,&MCP2515_BT_8MHz_1000kbps },
        {  8, 500,&MCP2515_BT_8MHz_500kbps  },
        {  8, 250,&MCP2515_BT_8MHz_250kbps  },
        {  8, 125,&MCP2515_BT_8MHz_125kbps  },
        {  8, 100,&MCP2515_BT_8MHz_100kbps  },
        {  8,  50,&MCP2515_BT_8MHz_50kbps   },
        { 16,1000,&MCP2515_BT_16MHz_1000kbps},
        { 16, 500,&MCP2515_BT_16MHz_500kbps },
        { 16, 250,&MCP2515_BT_16MHz_250kbps },
        { 16, 125,&MCP2515_BT_16MHz_125kbps },
        { 16, 100,&MCP2515_BT_16MHz_100kbps },
        { 16,  50,&MCP2515_BT_16MHz_50kbps  },
        { 20,1000,&MCP2515_BT_20MHz_1000kbps},
        { 20, 500,&MCP2515_BT_20MHz_500kbps },
        { 20, 250,&MCP2515_BT_20MHz_250kbps },
        { 20, 125,&MCP2515_BT_20MHz_125kbps },
    };
    for (size_t i = 0; i < sizeof(lut)/sizeof(lut[0]); i++) {
        if (lut[i].f == fosc_mhz && lut[i].b == baud_kbps) {
            *bt = *lut[i].p;
            return MCP2515_OK;
        }
    }
    return MCP2515_ERR_PARAM;
}

void mcp2515_print_msg(const mcp2515_msg_t *msg)
{
    if (!msg) return;
    printf("[CAN] ID=0x%08X %s %s DLC=%u",
           msg->id,
           msg->extended ? "EXT" : "STD",
           msg->rtr      ? "RTR" : "DAT",
           msg->dlc);
    if (!msg->rtr) {
        printf(" DATA:");
        for (uint8_t i = 0; i < msg->dlc; i++)
            printf(" %02X", msg->data[i]);
    }
    printf("\n");
}
