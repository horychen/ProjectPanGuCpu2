
/**
 * @file   ads8688_min.h
 * @brief  Minimal ADS8688 SPI comm (F28388D CPU2, register-only, no driverlib)
 * @note   - Uses SPIA, 8-bit frames, Mode 1 (CPOL=0, CPHA=1).
 *         - Keeps CS low for a full 32 SCLKs per transaction (16 write + 16 read).
 *         - Default pinmux: GPIO16=SPISIMOA, GPIO17=SPISOMIA, GPIO18=SPICLKA.
 *         - Default CS pin: GPIO60 (change macros if needed).
 */
#ifndef ADS8688_MIN_H
#define ADS8688_MIN_H

#include <stdint.h>
#include <stdbool.h>
#include "hw_types.h"   /* TI device header: provides SpiaRegs, GpioCtrlRegs, etc. */

/* CS control macros (GPIO60 default). If you change CS pin, edit below.) */
#define ADS8688_MIN_CS_LOW()    (GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1)
#define ADS8688_MIN_CS_HIGH()   (GpioDataRegs.GPBSET.bit.GPIO61   = 1)

/* ---------------- ADS8688 command / register map ---------------- */
#define ADS_CMD_CONT      0x00
#define ADS_CMD_STBY      0x82
#define ADS_CMD_PWDN      0x83
#define ADS_CMD_RST       0x85
#define ADS_CMD_AUTO_RST  0xA0
#define ADS_CMD_MAN_0     0xC0
#define ADS_CMD_MAN_1     0xC4
#define ADS_CMD_MAN_2     0xC8
#define ADS_CMD_MAN_3     0xCC
#define ADS_CMD_MAN_4     0xD0
#define ADS_CMD_MAN_5     0xD4
#define ADS_CMD_MAN_6     0xD8
#define ADS_CMD_MAN_7     0xDC

#define ADS_REG_AUTO_SEQ_EN      0x01
#define ADS_REG_CHN_PWRDN        0x02
#define ADS_REG_FEATURE_SELECT   0x03
#define ADS_REG_CH0_RANGE        0x05
#define ADS_REG_CH1_RANGE        0x06
#define ADS_REG_CH2_RANGE        0x07
#define ADS_REG_CH3_RANGE        0x08
#define ADS_REG_CH4_RANGE        0x09
#define ADS_REG_CH5_RANGE        0x0A
#define ADS_REG_CH6_RANGE        0x0B
#define ADS_REG_CH7_RANGE        0x0C
#define ADS_REG_CMD_RD_BCK       0x3F
#define ADS_RANGE_PM2V56   0x02
/* Frame builders for program register access */
static inline uint16_t ADS8688_MakeWrite(uint8_t reg, uint8_t val)
{ return (uint16_t)((0x1u<<12) | ((reg & 0x3Fu) << 8) | (val)); }

static inline uint16_t ADS8688_MakeRead(uint8_t reg)
{ return (uint16_t)((0x2u<<12) | ((reg & 0x3Fu) << 8)); }

/* ---------------- API ---------------- */
//void     ADS8688_MIN_Pinmux_SPIA(void);     /* GPIO16/17/18 -> SPIA */
//void     ADS8688_MIN_CS_Init(void);         /* Init CS (GPIO60 default) */
//void     ADS8688_MIN_SPIA_Init(void);       /* 8-bit, Mode1, baud per macros */

uint16_t ADS8688_Frame32(uint16_t cmd16); /* One full 32-SCLK transaction */

void     ADS8688_SendCmd(uint8_t cmd, uint8_t payload);
void     ADS8688_ProgWrite(uint8_t reg, uint8_t val);
uint8_t  ADS8688_ProgRead(uint8_t reg);

/* Simple smoke test: reset -> align -> MAN_0 -> read (returns 16-bit sample) */
uint16_t ADS8688_SmokeTest(void);
#endif /* ADS8688_MIN_H */
