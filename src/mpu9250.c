#include "mpu9250.h"
#include "mpu9250_registers.h"
#include <math.h>
#include <ch.h>

/* FS_SEL = 1 (max 500 deg / s) */
#define MPU9250_GYRO_SENSITIVITY (M_PI / (180 * 65.5))

/* AFS_SEL = 1 (max 500 deg / s) */
#define MPU9250_ACCEL_SENSITIVITY (9.81 / 8192)

#define MPU9250_TEMP_SENSITIVITY (1 / 333.87)
#define MPU9250_TEMP_OFFSET 21

#define AK8963_MAG_SENSITIVITY (4912.0f / 32760)

static uint8_t mpu9250_reg_read(mpu9250_t *dev, uint8_t reg);
static void mpu9250_reg_write(mpu9250_t *dev, uint8_t reg, uint8_t val);

void mpu9250_init(mpu9250_t *dev, SPIDriver *spi_dev)
{
    dev->spi = spi_dev;
}

bool mpu9250_ping(mpu9250_t *dev)
{
    int id = mpu9250_reg_read(dev, MPU9250_REG_WHO_AM_I);
    return id == 0x71;
}

void mpu9250_configure(mpu9250_t *dev)
{
    /* No FIFO, no external sync, gyroscope sample at 1 kHz, bandwidth 184 Hz */
    mpu9250_reg_write(dev, MPU9250_REG_CONFIG, 0x01);

    /* Divide the sample rate by 3 -> data ready rate is 250 Hz. */
    mpu9250_reg_write(dev, MPU9250_REG_SMPLRT_DIV, 3);

    /* Gyro uses LP filter, 500 deg / s max */
    mpu9250_reg_write(dev, MPU9250_REG_GYRO_CONFIG, (1 << 3));

    /* Accelerometer: 4 g max */
    /* TODO: Correctly calculate max acceleration value. */
    mpu9250_reg_write(dev, MPU9250_REG_ACCEL_CONFIG, (1 << 3));

    /* Accelerometer: 460 Hz bandwidth, 1 kHz sample rate. */
    mpu9250_reg_write(dev, MPU9250_REG_ACCEL_CONFIG_2, 0);

    /* INT pin is active high, configured as push pull, latched, and cleared by
     * reading the INT_STATUS register. */
    mpu9250_reg_write(dev, MPU9250_REG_INT_PIN_CFG,
                      MPU9250_REG_INT_PIN_CFG_LATCHED_INT);

    /* Enable INT on data ready. */
    mpu9250_reg_write(dev, MPU9250_REG_INT_ENABLE, MPU9250_REG_INT_ENABLE_DATA_RDY);

    /* Enable FIFO and clear its content */
    mpu9250_reg_write(dev, MPU9250_REG_USER_CTRL,
                      MPU9250_REG_USER_CTRL_FIFO_EN | MPU9250_REG_USER_CTRL_FIFO_RST);
}

void mpu9250_reset(mpu9250_t *dev)
{
    /* Sets the H_RESET bit. */
    mpu9250_reg_write(dev, MPU9250_REG_PWR_MGMT_1, 0x80);
}

void mpu9250_enable_magnetometer(mpu9250_t *dev)
{
    /* Enable I2C module. */
    uint8_t ctrl = mpu9250_reg_read(dev, MPU9250_REG_USER_CTRL);
    ctrl |= MPU9250_REG_USER_CTRL_I2C_MST_EN;
    ctrl |= MPU9250_REG_USER_CTRL_I2C_IF_DIS;
    mpu9250_reg_write(dev, MPU9250_REG_USER_CTRL, ctrl);

    /* Delay data ready interrupt until data came from the magnetometer. */
    mpu9250_reg_write(dev, MPU9250_REG_I2C_MST_CTRL, (1 << 6) | (1 << 4));

#if 0
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | (1 << 7));
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_REG, AK8963_REG_WHOAMI);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_CTRL, 0x81);
    chThdSleep(1000);
    uint8_t val = mpu9250_reg_read(dev, MPU9250_REG_EXT_SENS_DATA_00);

    if (val != 0x48) {
        chSysHalt("foo");
    }

#else
    /* Enable continous mode @ 100 Hz. */

    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_REG, AK8963_REG_CNTL2);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_DO, 0x01);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_CTRL, 0x81);

    chThdSleepMilliseconds(100);

    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_REG, 0xc);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_DO, (1 << 6));
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_CTRL, 0x81);


    chThdSleepMilliseconds(100);

    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_REG, AK8963_REG_CNTL1);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_DO, 0x16);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_CTRL, 0x81);

    // TODO cleanup those delays
    chThdSleepMilliseconds(1000);

    /* Sets the address with the read bit set. */
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | (1 << 7));

    /* Read 6 bytes from the magnetometer starting from the HXL register and store them. */
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_REG, AK8963_REG_HXL);
    mpu9250_reg_write(dev, MPU9250_REG_I2C_SLV0_CTRL, (1 << 7) + 0x7);
#endif
}

uint8_t mpu9250_interrupt_read_and_clear(mpu9250_t *dev)
{
    /* Reading the interrupt status register clears it automatically. */
    return mpu9250_reg_read(dev, MPU9250_REG_INT_STATUS);
}

void mpu9250_gyro_read(mpu9250_t *dev, float *x, float *y, float *z)
{
    uint8_t xh, xl, yh, yl, zh, zl;
    int16_t mes_x, mes_y, mes_z;
    xh = mpu9250_reg_read(dev, MPU9250_REG_GYRO_XOUT_H);
    xl = mpu9250_reg_read(dev, MPU9250_REG_GYRO_XOUT_L);
    yh = mpu9250_reg_read(dev, MPU9250_REG_GYRO_YOUT_H);
    yl = mpu9250_reg_read(dev, MPU9250_REG_GYRO_YOUT_L);
    zh = mpu9250_reg_read(dev, MPU9250_REG_GYRO_ZOUT_H);
    zl = mpu9250_reg_read(dev, MPU9250_REG_GYRO_ZOUT_L);

    mes_x = (xh << 8) + xl;
    mes_y = (yh << 8) + yl;
    mes_z = (zh << 8) + zl;

    *x = mes_x * MPU9250_GYRO_SENSITIVITY;
    *y = mes_y * MPU9250_GYRO_SENSITIVITY;
    *z = mes_z * MPU9250_GYRO_SENSITIVITY;
}

void mpu9250_acc_read(mpu9250_t *dev, float *x, float *y, float *z)
{
    uint8_t xh, xl, yh, yl, zh, zl;
    int16_t mes_x, mes_y, mes_z;
    xh = mpu9250_reg_read(dev, MPU9250_REG_ACCEL_XOUT_H);
    xl = mpu9250_reg_read(dev, MPU9250_REG_ACCEL_XOUT_L);
    yh = mpu9250_reg_read(dev, MPU9250_REG_ACCEL_YOUT_H);
    yl = mpu9250_reg_read(dev, MPU9250_REG_ACCEL_YOUT_L);
    zh = mpu9250_reg_read(dev, MPU9250_REG_ACCEL_ZOUT_H);
    zl = mpu9250_reg_read(dev, MPU9250_REG_ACCEL_ZOUT_L);

    mes_x = (xh << 8) + xl;
    mes_y = (yh << 8) + yl;
    mes_z = (zh << 8) + zl;

    *x = mes_x * MPU9250_ACCEL_SENSITIVITY;
    *y = mes_y * MPU9250_ACCEL_SENSITIVITY;
    *z = mes_z * MPU9250_ACCEL_SENSITIVITY;
}

float mpu9250_temp_read(mpu9250_t *dev)
{
    uint8_t th, tl;
    int16_t temp;

    th = mpu9250_reg_read(dev, MPU9250_REG_TEMP_OUT_H);
    tl = mpu9250_reg_read(dev, MPU9250_REG_TEMP_OUT_L);

    temp = (th << 8) + tl;

    return MPU9250_TEMP_OFFSET + temp * MPU9250_TEMP_SENSITIVITY;
}

void mpu9250_mag_read(mpu9250_t *dev, float *x, float *y, float *z)
{
    int16_t raw_x=0, raw_y=0, raw_z=0;

    raw_x = mpu9250_reg_read(dev, MPU9250_REG_EXT_SENS_DATA_01);
    raw_x <<= 8;
    raw_x |= mpu9250_reg_read(dev, MPU9250_REG_EXT_SENS_DATA_00);

    raw_y = mpu9250_reg_read(dev, MPU9250_REG_EXT_SENS_DATA_03);
    raw_y <<= 8;
    raw_y |= mpu9250_reg_read(dev, MPU9250_REG_EXT_SENS_DATA_02);

    raw_z = mpu9250_reg_read(dev, MPU9250_REG_EXT_SENS_DATA_05);
    raw_z <<= 8;
    raw_z |= mpu9250_reg_read(dev, MPU9250_REG_EXT_SENS_DATA_04);

    *x = raw_x * AK8963_MAG_SENSITIVITY;
    *y = raw_y * AK8963_MAG_SENSITIVITY;
    *z = raw_z * AK8963_MAG_SENSITIVITY;
}

static uint8_t mpu9250_reg_read(mpu9250_t *dev, uint8_t reg)
{
    uint8_t ret = 0;

    /* 7th bit indicates read (1) or write (0). */
    reg |= 0x80;

    spiSelect(dev->spi);
    spiSend(dev->spi, 1, &reg);
    spiReceive(dev->spi, 1, &ret);
    spiUnselect(dev->spi);

    return ret;
}

static void mpu9250_reg_write(mpu9250_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t cmd[] = {reg, val};

    spiSelect(dev->spi);
    spiSend(dev->spi, sizeof(cmd), cmd);
    spiUnselect(dev->spi);
}
