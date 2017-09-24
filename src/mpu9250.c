#include "mpu9250.h"
#include "mpu9250_registers.h"

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
    /* No FIFO, no external sync, gyroscope sample at 8 kHz (1kHz with prescaler) */
    /* TODO: For some reason it publishes at 8 kHz. */
    mpu9250_reg_write(dev, MPU9250_REG_SMPLRT_DIV, 7);
    mpu9250_reg_write(dev, MPU9250_REG_CONFIG, 0x00);

    /* Gyro uses LP filter, 500 deg / s max */
    mpu9250_reg_write(dev, MPU9250_REG_GYRO_CONFIG, (1 << 3));

    /* Accelerometer: 8 g max */
    /* TODO: Correctly calculate max acceleration value. */
    mpu9250_reg_write(dev, MPU9250_REG_ACCEL_CONFIG, (1 << 3));

    /* Accelerometer: 460 Hz bandwidth, 1 kHz sample rate. */
    mpu9250_reg_write(dev, MPU9250_REG_ACCEL_CONFIG_2, 0);

    /* Enable FIFO for all IMU channels. */
    mpu9250_reg_write(dev, MPU9250_REG_FIFO_EN,
                      MPU9250_REG_FIFO_EN_TEMP_OUT  |
                      MPU9250_REG_FIFO_EN_GYRO_XOUT |
                      MPU9250_REG_FIFO_EN_GYRO_YOUT |
                      MPU9250_REG_FIFO_EN_GYRO_ZOUT |
                      MPU9250_REG_FIFO_EN_ACCEL_OUT);

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

uint8_t mpu9250_interrupt_read_and_clear(mpu9250_t *dev)
{
    /* Reading the interrupt status register clears it automatically. */
    return mpu9250_reg_read(dev, MPU9250_REG_INT_STATUS);
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
