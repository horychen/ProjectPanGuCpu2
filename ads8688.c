
/**
 * @file   ads8688_min.c
 * @brief  Minimal ADS8688 SPI comm (F28388D CPU2, register-only, no driverlib)
 */
#include "ACMExpr.h"

/* ---------------- Internal helpers ---------------- */
static inline uint8_t spiA_txrx_byte(uint8_t b)
{
    /* C2000 SPI is left-justified: write byte in high 8 bits */
    SpiaRegs.SPITXBUF = ((uint16_t)b) << 8;
    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0) { }     /* wait for shift complete */
    return (uint8_t)(SpiaRegs.SPIRXBUF & 0x00FF);    /* low 8 bits valid */
}


/* ---------------- Public: one 32-SCLK frame ---------------- */
uint16_t ADS8688_Frame32(uint16_t cmd16)
{
    uint8_t hi = (uint8_t)(cmd16 >> 8);
    uint8_t lo = (uint8_t)(cmd16 & 0xFF);
    uint8_t r0, r1;

    ADS8688_MIN_CS_LOW();

    /* First 16 clocks: command (discard returned 16 bits) */
    (void)spiA_txrx_byte(hi);
    (void)spiA_txrx_byte(lo);

    /* Second 16 clocks: dummy to read previous conversion */
    r0 = spiA_txrx_byte(0x00);
    r1 = spiA_txrx_byte(0x00);

    ADS8688_MIN_CS_HIGH();

    return (uint16_t)((r0 << 8) | r1);
}

/* ---------------- Public: simple helpers ---------------- */
void ADS8688_SendCmd(uint8_t cmd, uint8_t payload)
{
    (void)ADS8688_Frame32( ((uint16_t)cmd << 8) | payload );
}

void ADS8688_ProgWrite(uint8_t reg, uint8_t val)
{
    (void)ADS8688_Frame32( ADS8688_MakeWrite(reg, val) );
}

uint8_t ADS8688_ProgRead(uint8_t reg)
{
    (void)ADS8688_Frame32( ADS8688_MakeRead(reg) );
    uint16_t resp = ADS8688_Frame32(0x0000);
    return (uint8_t)(resp & 0xFF);
}

/* ---------------- Public: smoke test ---------------- */
uint16_t ADS8688_SmokeTest(void)
{
    /* Reset device */
    ADS8688_SendCmd(ADS_CMD_RST, 0x00);
    DELAY_US(10000); /* 10 ms */

    /* Align pipeline */
    (void)ADS8688_Frame32(0x0000);

    /* Start CH0 and read it next frame */
    ADS8688_SendCmd(ADS_CMD_MAN_0, 0x00);
    return ADS8688_Frame32(0x0000);
}
