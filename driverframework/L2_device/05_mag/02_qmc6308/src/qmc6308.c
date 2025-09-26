/**
 ******************************************************************************
 * @file    qmcX983.c
 * @author  STMicroelectronics
 * @version V1.0
 * @date    2013-xx-xx
 * @brief    qmcX983����
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:���� ָ���� ������
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

#include "qmc6308.h"

#include <rtdef.h>
#include <rtthread.h>
#include <stdio.h>

#define QMC6308_ABS(X) ((X) < 0.0f ? (-1 * (X)) : (X))
#define QMC6308_ABSF(X) ((X) < 0.0f ? (-1.0 * (X)) : (X))

static unsigned char qmc6308_chipid = 0;
static qmc6308_map g_map;
static int sr_mode = 0;
static int set_only_times = 0;

// static u8 qmc6308_read_block(u8 addr, u8 *data, u8 len)
// {
//     u8 ret = 0;
//     u32 retry = 0;

//     while ((!ret) && (retry++ < 5))
//     {
// #if defined(QST_USE_SW_I2C)
//         ret = qst_sw_readreg(QMC6308_IIC_ADDR, addr, data, len);
// #else
//         ret = I2C_BufferRead(QMC6308_IIC_ADDR, addr, data, len);
// #endif
//     }

//     return ret;
// }

// static u8 qmc6308_write_reg(u8 addr, u8 data)
// {
//     u8 ret = 0;
//     u32 retry = 0;

//     while ((!ret) && (retry++ < 5))
//     {
// #if defined(QST_USE_SW_I2C)
//         ret = qst_sw_writereg(QMC6308_IIC_ADDR, addr, data);
// #else
//         ret = I2C_ByteWrite(QMC6308_IIC_ADDR, addr, data);
// #endif
//     }

//     return ret;
// }

mqc6308_i2c_read_block qmc6308_read_block = NULL;
mqc6308_i2c_write_reg qmc6308_write_reg = NULL;

static void qmc6308_set_layout(int layout)
{
    if (layout == 0)
    {
        g_map.sign[AXIS_X] = 1;
        g_map.sign[AXIS_Y] = 1;
        g_map.sign[AXIS_Z] = 1;
        g_map.map[AXIS_X] = AXIS_X;
        g_map.map[AXIS_Y] = AXIS_Y;
        g_map.map[AXIS_Z] = AXIS_Z;
    }
    else if (layout == 1)
    {
        g_map.sign[AXIS_X] = -1;
        g_map.sign[AXIS_Y] = 1;
        g_map.sign[AXIS_Z] = 1;
        g_map.map[AXIS_X] = AXIS_Y;
        g_map.map[AXIS_Y] = AXIS_X;
        g_map.map[AXIS_Z] = AXIS_Z;
    }
    else if (layout == 2)
    {
        g_map.sign[AXIS_X] = -1;
        g_map.sign[AXIS_Y] = -1;
        g_map.sign[AXIS_Z] = 1;
        g_map.map[AXIS_X] = AXIS_X;
        g_map.map[AXIS_Y] = AXIS_Y;
        g_map.map[AXIS_Z] = AXIS_Z;
    }
    else if (layout == 3)
    {
        g_map.sign[AXIS_X] = 1;
        g_map.sign[AXIS_Y] = -1;
        g_map.sign[AXIS_Z] = 1;
        g_map.map[AXIS_X] = AXIS_Y;
        g_map.map[AXIS_Y] = AXIS_X;
        g_map.map[AXIS_Z] = AXIS_Z;
    }
    else if (layout == 4)
    {
        g_map.sign[AXIS_X] = -1;
        g_map.sign[AXIS_Y] = 1;
        g_map.sign[AXIS_Z] = -1;
        g_map.map[AXIS_X] = AXIS_X;
        g_map.map[AXIS_Y] = AXIS_Y;
        g_map.map[AXIS_Z] = AXIS_Z;
    }
    else if (layout == 5)
    {
        g_map.sign[AXIS_X] = 1;
        g_map.sign[AXIS_Y] = 1;
        g_map.sign[AXIS_Z] = -1;
        g_map.map[AXIS_X] = AXIS_Y;
        g_map.map[AXIS_Y] = AXIS_X;
        g_map.map[AXIS_Z] = AXIS_Z;
    }
    else if (layout == 6)
    {
        g_map.sign[AXIS_X] = 1;
        g_map.sign[AXIS_Y] = -1;
        g_map.sign[AXIS_Z] = -1;
        g_map.map[AXIS_X] = AXIS_X;
        g_map.map[AXIS_Y] = AXIS_Y;
        g_map.map[AXIS_Z] = AXIS_Z;
    }
    else if (layout == 7)
    {
        g_map.sign[AXIS_X] = -1;
        g_map.sign[AXIS_Y] = -1;
        g_map.sign[AXIS_Z] = -1;
        g_map.map[AXIS_X] = AXIS_Y;
        g_map.map[AXIS_Y] = AXIS_X;
        g_map.map[AXIS_Z] = AXIS_Z;
    }
    else
    {
        g_map.sign[AXIS_X] = 1;
        g_map.sign[AXIS_Y] = 1;
        g_map.sign[AXIS_Z] = 1;
        g_map.map[AXIS_X] = AXIS_X;
        g_map.map[AXIS_Y] = AXIS_Y;
        g_map.map[AXIS_Z] = AXIS_Z;
    }
}

static int qmc6308_get_chipid(void)
{
    int ret = 0;
    int i;

    for (i = 0; i < 10; i++)
    {
        ret = qmc6308_read_block(QMC6308_CHIP_ID_REG, &qmc6308_chipid, 1);
        printf("qmc6308_get_chipid chipid = 0x%x\n", qmc6308_chipid);
        if (ret)
        {
            break;
        }
    }
    if (i >= 10)
    {
        return 0;
    }
    if ((qmc6308_chipid & 0xf0) == 0)
    {
        return 0;
    }

    return 1;
}

static void qmc6308_soft_reset(void)
{
    qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x80);
    rt_thread_mdelay(2);
    qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x00);
    rt_thread_mdelay(5);
}

int qmc6308_selftest(void)
{
    int selftest_result = 1;
    int hdata_a[3];
    int hdata_b[3];
    int hdata[3];
    uint8_t rx_buf[8] = {0};
    int t1 = 0;
    int check_result = 0;
    int ret = 0;
    int retry_num = 0;
    // uint8_t reg_bak[8] = {0};
    uint8_t reg_bak_data[8] = {0};

    qmc6308_read_block(QMC6308_CTL_REG_THREE, &reg_bak_data[0], 1);
    qmc6308_read_block(QMC6308_CTL_REG_ONE, &reg_bak_data[1], 1);
    qmc6308_read_block(QMC6308_CTL_REG_TWO, &reg_bak_data[2], 1);

    for (retry_num = 0; retry_num < 3; retry_num++)
    {
        qmc6308_soft_reset();
        qmc6308_write_reg(QMC6308_CTL_REG_THREE, 0x40);
        rt_thread_mdelay(1);
        qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x00);
        rt_thread_mdelay(1);
        qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x03);
        rt_thread_mdelay(1);
        t1 = 0;

        /* Check status register for data availability */
        rx_buf[0] = 0x0;
        while (!(rx_buf[0] & 0x03) && t1 < 20)
        {
            qmc6308_read_block(QMC6308_STATUS_REG, &rx_buf, 1);
            printf("qmc6308 Status Register is (0x%02X)\n", rx_buf[0]);
            t1++;
            rt_thread_mdelay(1);
        }
        if (t1 >= 20)
        {
            continue;
        }
        ret = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, rx_buf, 6);

        hdata_a[0] = (int16_t)((rx_buf[1] << 8) | (rx_buf[0]));
        hdata_a[1] = (int16_t)((rx_buf[3] << 8) | (rx_buf[2]));
        hdata_a[2] = (int16_t)((rx_buf[5] << 8) | (rx_buf[4]));
        printf("qmc6308: selftest data_a,%d %d %d\n", hdata_a[0], hdata_a[1], hdata_a[2]);
        qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x40);
        rt_thread_mdelay(20);
        t1 = 0;
        rx_buf[0] = 0x00;
        while (!(rx_buf[0] & 0x03) && t1 < 20)
        {
            qmc6308_read_block(QMC6308_STATUS_REG, &rx_buf, 1);
            printf("qmc6308 Status Register is (0x%02X)\n", rx_buf[0]);
            t1++;
            rt_thread_mdelay(1);
        }
        if (t1 >= 20)
        {
            continue;
        }
        ret = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, rx_buf, 6);

        hdata_b[0] = (int16_t)((rx_buf[1] << 8) | (rx_buf[0]));
        hdata_b[1] = (int16_t)((rx_buf[3] << 8) | (rx_buf[2]));
        hdata_b[2] = (int16_t)((rx_buf[5] << 8) | (rx_buf[4]));
        printf("qmc6308: selftest data_b,%d %d %d\n", hdata_b[0], hdata_b[1], hdata_b[2]);
        hdata[0] = QMC6308_ABS(hdata_a[0] - hdata_b[0]);
        hdata[1] = QMC6308_ABS(hdata_a[1] - hdata_b[1]);
        hdata[2] = QMC6308_ABS(hdata_a[2] - hdata_b[2]);
        printf("qmc6308: selftest data_abs,%d %d %d\n", hdata[0], hdata[1], hdata[2]);
        check_result = 0;
        if ((hdata[0] < QMC6308_SELFTEST_MAX_X) && (hdata[0] > QMC6308_SELFTEST_MIN_X))
        {
            check_result++;
        }
        if ((hdata[1] < QMC6308_SELFTEST_MAX_Y) && (hdata[1] > QMC6308_SELFTEST_MIN_Y))
        {
            check_result++;
        }
        if ((hdata[2] < QMC6308_SELFTEST_MAX_Z) && (hdata[2] > QMC6308_SELFTEST_MIN_Z))
        {
            check_result++;
        }
        if (check_result >= 3)
        {
            selftest_result = 1;
            break;
        }
        else
        {
            selftest_result = -1;
        }
    }

    qmc6308_write_reg(QMC6308_CTL_REG_THREE, reg_bak_data[0]);
    qmc6308_write_reg(QMC6308_CTL_REG_ONE, reg_bak_data[1]);
    qmc6308_write_reg(QMC6308_CTL_REG_TWO, reg_bak_data[2]);

    return selftest_result;
}

#if defined(QMC6308_MODE_SWITCH)
void qmc6308_setrst_auto_mode(short hw_d[3])
{
    if (sr_mode == 0)
    {
        if ((QMC6308_ABS(hw_d[0]) > 6000) || (QMC6308_ABS(hw_d[1]) > 10000))
        {
            qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);
            qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x01);
            qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0xC3);
            sr_mode = 1;
            set_only_times = 0;
        }
    }
    else
    {
        int force_switch = 0;
        set_only_times++;
        if (set_only_times = 50)
        {
            set_only_times = 0;
            force_switch = 1;
        }
        if (((QMC6308_ABS(hw_d[0]) < 5000) && (QMC6308_ABS(hw_d[1]) < 8000)) || force_switch)
        {
            qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);
            qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x00);
            qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0xC3);
            sr_mode = 0;
        }
    }
}
#endif

int qmc6308_read_mag_xyz(float *data)
{
    int res;
    unsigned char mag_data[6];
    short hw_d[3] = {0};
    float hw_f[3];
    int t1 = 0;
    unsigned char rdy = 0;

    /* Check status register for data availability */
    while (!(rdy & 0x01) && (t1 < 5))
    {
        rdy = QMC6308_STATUS_REG;
        res = qmc6308_read_block(QMC6308_STATUS_REG, &rdy, 1);
        t1++;
    }

    mag_data[0] = QMC6308_DATA_OUT_X_LSB_REG;

    res = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, mag_data, 6);
    if (res == 0)
    {
        return 0;
    }

    hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
    hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
    hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);  // mgas

#if defined(QMC6308_MODE_SWITCH)
    qmc6308_setrst_auto_mode(hw_d);
#endif

    hw_f[0] = (float)((float)hw_d[0] / 10.0f);  // ut
    hw_f[1] = (float)((float)hw_d[1] / 10.0f);  // ut
    hw_f[2] = (float)((float)hw_d[2] / 10.0f);  // ut

    data[AXIS_X] = (float)(g_map.sign[AXIS_X] * hw_f[g_map.map[AXIS_X]]);
    data[AXIS_Y] = (float)(g_map.sign[AXIS_Y] * hw_f[g_map.map[AXIS_Y]]);
    data[AXIS_Z] = (float)(g_map.sign[AXIS_Z] * hw_f[g_map.map[AXIS_Z]]);

    return res;
}

/* Set the sensor mode */
static int qmc6308_set_mode(unsigned char mode)
{
    int err = 0;
    unsigned char ctrl1_value = 0;

    err = qmc6308_read_block(QMC6308_CTL_REG_ONE, &ctrl1_value, 1);
    ctrl1_value = (ctrl1_value & (~0x03)) | mode;
    printf("qmc6308_set_mode, 0x%x = 0x%x \n", QMC6308_CTL_REG_ONE, ctrl1_value);
    err = qmc6308_write_reg(QMC6308_CTL_REG_ONE, ctrl1_value);

    return err;
}

static int qmc6308_set_output_data_rate(unsigned char rate)
{
    int err = 0;
    unsigned char ctrl1_value = 0;

    err = qmc6308_read_block(QMC6308_CTL_REG_ONE, &ctrl1_value, 1);
    ctrl1_value = (ctrl1_value & (~0x0C)) | (rate);
    printf("qmc6308_set_output_data_rate, 0x%x = 0x%2x \n", QMC6308_CTL_REG_ONE, ctrl1_value);
    err = qmc6308_write_reg(QMC6308_CTL_REG_ONE, ctrl1_value);

    return err;
}

static int qmc6308_enable(void)
{
    int ret;

    if (qmc6308_chipid == 0x80)
    {
        ret = qmc6308_write_reg(0x0d, 0x40);
        ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x00);
        ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0xc3);
    }
    else
    {
        ret = qmc6308_write_reg(0x0d, 0x40);
        ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x08);
        ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x63);
    }

    return ret;
}

static int qmc6308_disable(void)
{
    int ret;

    ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);

    return ret;
}

int qmc6308_init(void)
{
    int ret = 0;

    ret = qmc6308_get_chipid();
    if (ret == 0)
    {
        return 0;
    }
    qmc6308_set_layout(1);
    qmc6308_enable();
#if 0
	{
		unsigned char ctrl_value;
		qmc6308_read_block(QMC6308_CTL_REG_ONE, &ctrl_value, 1);
		printf("qmc6308  0x%x=0x%x \n", QMC6308_CTL_REG_ONE, ctrl_value);
		qmc6308_read_block(QMC6308_CTL_REG_TWO, &ctrl_value, 1);
		printf("qmc6308  0x%x=0x%x \n", QMC6308_CTL_REG_TWO, ctrl_value);
		qmc6308_read_block(0x0d, &ctrl_value, 1);
		printf("qmc6308  0x%x=0x%x \n", 0x0d, ctrl_value);
	}
#endif
    return 1;
}

static int qmc6308_config_zerozer(void)
{
    int ret;

    uint8_t CONTROL_REG_ONE_VALUE = QMC6308_CONTINUOUS_MODE | QMC6308_SET_OUTPUT_DATA_RATE_100 |
                                    QMC6308_SET_OVERSAMPLE_RATIO_8 | QMC6308_SET_DOWNSAMPLE_RATIO_1;

    uint8_t CONTROL_REG_TWO_VALUE = QMC6308_SET_SET_RESET_OFF | QMC6308_SET_RANGE_8G;

    uint8_t CONTROL_REG_THREE_VALUE = QMC6308_SET_SRCTRL_ON;

    if (qmc6308_chipid == QMC6308_ID_VALUE)
    {
        ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, CONTROL_REG_ONE_VALUE);
        ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, CONTROL_REG_TWO_VALUE);
        ret = qmc6308_write_reg(QMC6308_CTL_REG_THREE, CONTROL_REG_THREE_VALUE);
    }
    return ret;
}

static int qmc6308_check_id_zerozero(void)
{
    int ret = 0;
    int i;

    for (i = 0; i < 10; i++)
    {
        ret = qmc6308_read_block(QMC6308_CHIP_ID_REG, &qmc6308_chipid, 1);
        printf("qmc6308_get_chipid chipid = 0x%x\n", qmc6308_chipid);
        if (ret)
        {
            break;
        }
    }
    if (i >= 10)
    {
        return 0;
    }
    if ((qmc6308_chipid & 0xf0) == 0)
    {
        return 0;
    }

    return 1;
}

int qmc6308_init_zerozero(void)
{
    int ret = 0;
    if (qmc6308_check_id_zerozero() != 1)
    {
        printf("qmc6308_check_id_zerozero failed\n");
        return 0;
    }
    else
    {
        printf("qmc6308_check_id_zerozero success\n");
    }

    if (qmc6308_config_zerozer() != 1)
    {
        printf("qmc6308_config_zerozer failed\n");
        return 0;
    }
    else
    {
        printf("qmc6308_config_zerozer success\n");
    }
    return 1;
}

int qmc6308_read_mag_xyz_zerozero(int16_t *data)
{
    int res;
    int t1 = 0;
    unsigned char rdy = 0;
    unsigned char mag_data[6];

    /* Check status register for data availability */
    while (!(rdy & 0x01) && (t1 < 5))
    {
        rdy = QMC6308_STATUS_REG;
        res = qmc6308_read_block(QMC6308_STATUS_REG, &rdy, 1);
        t1++;
    }

    res = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, mag_data, 6);
    if (res == 0)
    {
        return 0;
    }

    data[0] = (int16_t)(((mag_data[1]) << 8) | mag_data[0]);
    data[1] = (int16_t)(((mag_data[3]) << 8) | mag_data[2]);
    data[2] = (int16_t)(((mag_data[5]) << 8) | mag_data[4]);  // mgas

    return res;
}
