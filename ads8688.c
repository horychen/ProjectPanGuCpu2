#include "ACMExpr.h"

/* ---------------- Internal helpers ---------------- */
static inline uint8_t spiA_txrx_byte(uint8_t b)
{
    /* C2000 SPI is left-justified: write byte in high 8 bits */
    SpiaRegs.SPITXBUF = ((uint16_t)b) << 8;
    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0) { }     /* wait for shift complete */
    return (uint8_t)(SpiaRegs.SPIRXBUF & 0x00FF);    /* low 8 bits valid */
}

/* ��տ��ܲ�����RX����/��־���������������λһ�� SPIA */
static inline void spia_rx_cleanup(void)
{
    /* �������δ�������ֽڣ��� SPIRXBUF ���� INT_FLAG�� */
    while(SpiaRegs.SPISTS.bit.INT_FLAG) (void)(SpiaRegs.SPIRXBUF);

    /* �����ֹ��������һ����С��λ���������λ */
    if (SpiaRegs.SPISTS.bit.OVERRUN_FLAG) {
        EALLOW;
        SpiaRegs.SPICCR.bit.SPISWRESET = 0;
        SpiaRegs.SPICCR.bit.SPISWRESET = 1;
        EDIS;
    }
}

/* ---------------- Public: one 32-SCLK frame����ȫ�棩 ---------------- */
uint16_t ADS8688_Frame32(uint16_t cmd16)
{
    uint8_t hi = (uint8_t)(cmd16 >> 8);
    uint8_t lo = (uint8_t)(cmd16 & 0xFF);
    uint8_t r0, r1;

    /* 0) �������壬���������һ�β��� */
    spia_rx_cleanup();

    /* 1) �ؼ�������֤ 4 �ֽڱ����������� ISR ��ϣ�~6�C8us @5MHz�� */
    DINT;

    ADS8688_MIN_CS_LOW();

    /* First 16 clocks: command (discard returned 16 bits) */
    (void)spiA_txrx_byte(hi);
    (void)spiA_txrx_byte(lo);

    /* Second 16 clocks: dummy to read previous conversion */
    r0 =  spiA_txrx_byte(0x00);
    r1 =  spiA_txrx_byte(0x00);

    ADS8688_MIN_CS_HIGH();
    NOP;
    NOP;
    /* 2) ����״̬���/���������ٴ����� */
    if (SpiaRegs.SPISTS.bit.OVERRUN_FLAG) {
        EALLOW;
        SpiaRegs.SPICCR.bit.SPISWRESET = 0;
        SpiaRegs.SPICCR.bit.SPISWRESET = 1;
        EDIS;
    }

    EINT;  /* �ָ��ж� */

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

    /* ���飺�״ζ��ٶ���һ֡����һ����̬ */
    (void)ADS8688_Frame32(0x0000);  // ����
    return ADS8688_Frame32(0x0000); // ����
}

// 设 8 路为 ±2.56V
void ADS8688_SetAll_PM2V56(void)
{
    ADS8688_ProgWrite(ADS_REG_CH0_RANGE, ADS_RANGE_PM2V56);
    ADS8688_ProgWrite(ADS_REG_CH1_RANGE, ADS_RANGE_PM2V56);
    ADS8688_ProgWrite(ADS_REG_CH2_RANGE, ADS_RANGE_PM2V56);
    ADS8688_ProgWrite(ADS_REG_CH3_RANGE, ADS_RANGE_PM2V56);
    ADS8688_ProgWrite(ADS_REG_CH4_RANGE, ADS_RANGE_PM2V56);
    ADS8688_ProgWrite(ADS_REG_CH5_RANGE, ADS_RANGE_PM2V56);
    ADS8688_ProgWrite(ADS_REG_CH6_RANGE, ADS_RANGE_PM2V56);
    ADS8688_ProgWrite(ADS_REG_CH7_RANGE, ADS_RANGE_PM2V56);
}

// 读 8 路原始码（手动扫描，遵循“本帧下命令、下一帧读上一次结果”）
void ADS8688_ReadAll8(uint16_t *dst8)
{
    if (!dst8) return;

    // 帧0：启动 CH0 转换（本帧回读无效/垃圾）
    (void)ADS8688_Frame32(((uint16_t)ADS_CMD_MAN_0 << 8));

    // 帧1..8：边下下一通道命令，边取上一通道结果
    dst8[0] = ADS8688_Frame32(((uint16_t)ADS_CMD_MAN_1 << 8)); // 读到 CH0
    dst8[1] = ADS8688_Frame32(((uint16_t)ADS_CMD_MAN_2 << 8)); // 读到 CH1
    dst8[2] = ADS8688_Frame32(((uint16_t)ADS_CMD_MAN_3 << 8)); // 读到 CH2
    dst8[3] = ADS8688_Frame32(((uint16_t)ADS_CMD_MAN_4 << 8)); // 读到 CH3
    dst8[4] = ADS8688_Frame32(((uint16_t)ADS_CMD_MAN_5 << 8)); // 读到 CH4
    dst8[5] = ADS8688_Frame32(((uint16_t)ADS_CMD_MAN_6 << 8)); // 读到 CH5
    dst8[6] = ADS8688_Frame32(((uint16_t)ADS_CMD_MAN_7 << 8)); // 读到 CH6
    dst8[7] = ADS8688_Frame32(((uint16_t)ADS_CMD_MAN_0 << 8)); // 读到 CH7，并回到 CH0，便于下一轮
}
