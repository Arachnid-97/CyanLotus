#include "MPU6050.h"
#include "bsp_iic.h"
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"


/**
 * @brief  Writes a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data write from the MPU6050.
 * @param  writeAddr : MPU6050's internal address to write from.
 * @param  NumByteToWrite : number of bytes to write from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */
static void MPU6050_WriteBuffer(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr, uint16_t NumByteToWrite)
{
    uint16_t i;

	IIC_Start();

    Write_IIC_Byte(slaveAddr | 0);
    if(0 == IIC_Wait_Ack())
        goto WriteFail;
    
    Write_IIC_Byte((uint8_t)(writeAddr & 0x00FF));
    if(0 == IIC_Wait_Ack())
        goto WriteFail;
    
    for (i = 0; i < NumByteToWrite; i++)
    {
        Write_IIC_Byte(*(pBuffer + i));
        if(0 == IIC_Wait_Ack())
            goto WriteFail;
    }
	
WriteFail:
	IIC_Stop();
	return;
}

/**
 * @brief  Reads a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */
static void MPU6050_ReadBuffer(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead)
{
    uint16_t i;

    IIC_Start();

    Write_IIC_Byte(slaveAddr | 0);
	if(0 == IIC_Wait_Ack())
		goto ReadFail;

    Write_IIC_Byte((uint8_t)(readAddr & 0x00FF));
	if(0 == IIC_Wait_Ack())
		goto ReadFail;

    IIC_Start();

    Write_IIC_Byte(slaveAddr | 1);
	if(0 == IIC_Wait_Ack())
		goto ReadFail;

    for (i = 0; i < NumByteToRead; i++)
    {
        *(pBuffer + i) = Read_IIC_Byte();

        if (i != NumByteToRead - 1)
        {
            IIC_Ack();
        }
        else
        {
            IIC_UnAck();
        }
    }

ReadFail:
    IIC_Stop();
    return;
}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
static void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t temp;
    MPU6050_ReadBuffer(slaveAddr, &temp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    temp &= ~(mask); // zero all important bits in existing byte
    temp |= data; // combine data with existing byte
    MPU6050_WriteBuffer(slaveAddr, &temp, regAddr, 1);
}

/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
static void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t temp;
    MPU6050_ReadBuffer(slaveAddr, &temp, regAddr, 1);
    temp = (data != 0) ? (temp | (1 << bitNum)) : (temp & ~(1 << bitNum));
    MPU6050_WriteBuffer(slaveAddr, &temp, regAddr, 1);
}

/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
static void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    uint8_t temp;
    MPU6050_ReadBuffer(slaveAddr, &temp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    temp &= mask;
    temp >>= (bitStart - length + 1);
    *data = temp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
static void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t temp;
    MPU6050_ReadBuffer(slaveAddr, &temp, regAddr, 1);
    *data = temp & (1 << bitNum);
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
enum Ascale MPU6050_GetFullScaleAccelRange(void)
{
    uint8_t temp;
    MPU6050_ReadBits(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &temp);
    return (enum Ascale)temp;
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
void MPU6050_SetFullScaleAccelRange(enum Ascale range)
{
    MPU6050_WriteBits(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, (uint8_t)range);
}

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
enum Gscale MPU6050_GetFullScaleGyroRange(void)
{
    uint8_t temp;
    MPU6050_ReadBits(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &temp);
    return (enum Gscale)temp;
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(enum Gscale range)
{
    MPU6050_WriteBits(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, (uint8_t)range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus(void)
{
    uint8_t temp;
    MPU6050_ReadBit(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &temp);
    return temp == 0x00 ? FALSE : TRUE;
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(FunctionalState NewState)
{
    MPU6050_WriteBit(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source)
{
    MPU6050_WriteBits(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050_GetDeviceID(void)
{
    uint8_t temp = 0;
    MPU6050_ReadBits(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &temp);
    return temp;
}

/** Get raw 6-axis motion sensor readings (accel).
 * Retrieves all currently available motion sensor values.
 * @param Accel 16-bit signed integer array of length 3
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050_GetRawAccel(int16_t *Accel)
{
    uint8_t tempBuffer[6];
    MPU6050_ReadBuffer(MPU6050_ADDRESS, tempBuffer, MPU6050_RA_ACCEL_XOUT_H, 6);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        Accel[i] = (int16_t) (((uint16_t) tempBuffer[2 * i] << 8) + tempBuffer[2 * i + 1]);
}

/** Get raw 6-axis motion sensor readings (gyro).
 * Retrieves all currently available motion sensor values.
 * @param Gyro 16-bit signed integer array of length 3
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPU6050_GetRawGyro(int16_t *Gyro)
{
    uint8_t tempBuffer[6];
    MPU6050_ReadBuffer(MPU6050_ADDRESS, tempBuffer, MPU6050_RA_GYRO_XOUT_H, 6);
    /* Get Angular rate */
    for (int i = 0; i < 3; i++)
        Gyro[i - 1] = (int16_t) (((uint16_t) tempBuffer[2 * i] << 8) + tempBuffer[2 * i + 1]);
}

/** Get raw 6-axis motion sensor readings (temperature).
 * @param Temperature 16-bit signed integer data
 * @see MPU6050_RA_TEMP_OUT_H
 */
void MPU6050_GetRawTemp(int16_t *Temperature)
{
    uint8_t tempBuffer[2];
    MPU6050_ReadBuffer(MPU6050_ADDRESS, tempBuffer, MPU6050_RA_TEMP_OUT_H, 2);
    *Temperature = (int16_t) (((uint16_t) tempBuffer[0] << 8) + tempBuffer[1]);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 6
 * @see MPU6050_RA_ACCEL_XOUT_H
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro)
{
    uint8_t tempBuffer[14];
    MPU6050_ReadBuffer(MPU6050_ADDRESS, tempBuffer, MPU6050_RA_ACCEL_XOUT_H, 14);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        AccelGyro[i] = (int16_t) (((uint16_t) tempBuffer[2 * i] << 8) + tempBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (int i = 4; i < 7; i++)
        AccelGyro[i - 1] = (int16_t) (((uint16_t) tempBuffer[2 * i] << 8) + tempBuffer[2 * i + 1]);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection(void)
{
    return MPU6050_GetDeviceID() == 0x34 ? TRUE : FALSE; //0b110100; 8-bit representation in hex = 0x34
}

/** Rset MPU6050 motion sensor readings.
 * Write a one to bit 7 reset bit, toggle reset device.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void MPU6050_Reset(void)
{
    MPU6050_WriteBit(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Config(void)
{
    uint8_t data;

    MPU6050_SetSleepModeStatus(DISABLE);
    MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
    data = 0x03;
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &data, MPU6050_RA_CONFIG, 1);
    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    data = 0x04;
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &data, MPU6050_RA_SMPLRT_DIV, 1); // Use a 200 Hz rate; the same rate set in CONFIG above

    // Set accelerometer full scale range
    MPU6050_SetFullScaleAccelRange(AFS_2G);
    // Set gyroscope full scale range
    MPU6050_SetFullScaleGyroRange(GFS_250DPS);
}

void MPU6050_Init(void)
{
    Simulate_IIC_Config();
    IIC_Stop();

    MPU6050_Config();
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(float *destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[4] = {0, 0, 0, 0};
    uint8_t selfTest[6];
    float factoryTrim[6];

    // Configure the accelerometer for self-test
    MPU6050_ReadBuffer(MPU6050_ADDRESS, rawData, MPU6050_SELF_TEST_X, 4);
    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0; // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0] & 0x1F; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1] & 0x1F; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2] & 0x1F; // ZG_TEST result is a five-bit unsigned integer
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((selfTest[0] - 1.0f) / 30.0f))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((selfTest[1] - 1.0f) / 30.0f))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((selfTest[2] - 1.0f) / 30.0f))); // FT[Za] factory trim calculation
    factoryTrim[3] = (25.0f * 131.0f) * (pow(1.046f, (selfTest[3] - 1.0f)));                     // FT[Xg] factory trim calculation
    factoryTrim[4] = (-25.0f * 131.0f) * (pow(1.046f, (selfTest[4] - 1.0f)));                    // FT[Yg] factory trim calculation
    factoryTrim[5] = (25.0f * 131.0f) * (pow(1.046f, (selfTest[5] - 1.0f)));                     // FT[Zg] factory trim calculation

    // Output self-test results and factory trim calculation if desired
    // Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
    // Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
    // Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
    // Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++)
    {
        destination[i] = 100.0f + 100.0f * (selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
    }
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU6050_Calibrate(float *dest1, float *dest2)
{
    uint8_t temp_data;
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // Reset registers to default in preparation for device calibration
    MPU6050_Reset();
    vTaskDelay(1000 / portTICK_RATE_MS);

    MPU6050_Config();

    // Configure device for bias calculation
    temp_data = 0x00;
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &temp_data, MPU6050_RA_INT_ENABLE, 1);  // Disable all interrupts
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &temp_data, MPU6050_RA_FIFO_EN, 1);     // Disable FIFO
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &temp_data, MPU6050_RA_I2C_MST_CTRL, 1);// Disable I2C master
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &temp_data, MPU6050_RA_USER_CTRL, 1);   // Disable FIFO and I2C master modes
    temp_data = 0x0C;
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &temp_data, MPU6050_RA_USER_CTRL, 1);   // Reset FIFO and DMP

    uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    temp_data = 0x40;
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &temp_data, MPU6050_RA_USER_CTRL, 1);   // Enable FIFO
    temp_data = 0x78;
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &temp_data, MPU6050_RA_FIFO_EN, 1);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    vTaskDelay(800 / portTICK_RATE_MS); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    temp_data = 0x00;
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &temp_data, MPU6050_RA_FIFO_EN, 1);     // Disable gyro and accelerometer sensors for FIFO
    MPU6050_ReadBuffer(MPU6050_ADDRESS, &data[0], MPU6050_RA_FIFO_COUNTH, 2); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        MPU6050_ReadBuffer(MPU6050_ADDRESS, &data[0], MPU6050_RA_FIFO_R_W, 12);           // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }
    accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t)accelsensitivity;
    } // Remove gravity from the z-axis accelerometer bias calculation
    else
    {
        accel_bias[2] += (int32_t)accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    MPU6050_WriteBuffer(MPU6050_ADDRESS, &data[0], MPU6050_RA_XG_OFFS_USRH, 6);

    dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0};                // A place to hold the factory accelerometer trim biases
    MPU6050_ReadBuffer(MPU6050_ADDRESS, &data[0], MPU6050_RA_XA_OFFS_H, 6); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
    accel_bias_reg[1] = (int16_t)((int16_t)data[2] << 8) | data[3];
    accel_bias_reg[2] = (int16_t)((int16_t)data[4] << 8) | data[5];

    uint32_t mask = 1uL;             // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++)
    {
        if (accel_bias_reg[ii] & mask)
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    //  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
    //  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
    //  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
    //  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
    //  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
    //  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
    dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
    dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

const float PI = 3.14159265358979323846f;

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    // parameters for 6 DoF sensor fusion calculations
    // static const float PI = 3.14159265358979323846f;
    static const float GyroMeasError = PI * (60.0f / 180.0f);    // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
    static const float GyroMeasDrift = PI * (1.0f / 180.0f);     // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    // float pitch, yaw, roll;
    static float deltat = 0.0f;                          // integration interval for both filter schemes
    // int lastUpdate = 0, firstUpdate = 0, Now = 0; // used to calculate integration interval                               // used to calculate integration interval
    static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};        // vector to hold quaternion

    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objective funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz; // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    //            float _2q1q3 = 2.0f * q1 * q3;
    //            float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx = 0;
    gbiasy = 0;
    gbiasz = 0;
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 - (beta * hatDot1)) * deltat;
    q2 += (qDot2 - (beta * hatDot2)) * deltat;
    q3 += (qDot3 - (beta * hatDot3)) * deltat;
    q4 += (qDot4 - (beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

#include "bsp_uart.h"
void MPU6050_Test_Start(void)
{
    if(MPU6050_TestConnection())
    {
        DEBUG_PRINTF("MPU6050 communication success!\n");
        float self_test[6] = {0};
        MPU6050SelfTest(self_test);
        DEBUG_PRINTF("x-axis self test: acceleration trim within : %f%% of factory value \n", self_test[0]);
        DEBUG_PRINTF("y-axis self test: acceleration trim within : %f%% of factory value \n", self_test[1]);
        DEBUG_PRINTF("z-axis self test: acceleration trim within : %f%% of factory value \n", self_test[2]);
        DEBUG_PRINTF("x-axis self test: gyration trim within : %f%% of factory value \n", self_test[3]);
        DEBUG_PRINTF("y-axis self test: gyration trim within : %f%% of factory value \n", self_test[4]);
        DEBUG_PRINTF("z-axis self test: gyration trim within : %f%% of factory value \n", self_test[5]);

        if (self_test[0] < 1.0f && self_test[1] < 1.0f && self_test[2] < 1.0f && self_test[3] < 1.0f && self_test[4] < 1.0f && self_test[5] < 1.0f)
        {
            float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer

            MPU6050_Calibrate(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            DEBUG_PRINTF("MPU6050 initialized for active data mode....\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        }
        else
            DEBUG_PRINTF("Device did not the pass self-test!\n");
    }
}

void MPU6050_Test_Poll(void)
{
    int16_t acce_gyro_data[6] = {0};
    static float ax,ay,az,gx,gy,gz;
    static float aRes = 2.0 / 32768.0, gRes = 250.0 / 32768.0;

    MPU6050_GetRawAccelGyro(&acce_gyro_data[0]);
    DEBUG_PRINTF("raw data: %d, %d, %d, %d, %d, %d\n", acce_gyro_data[0], acce_gyro_data[1], acce_gyro_data[2], acce_gyro_data[3], acce_gyro_data[4], acce_gyro_data[5]);
    // Now we'll calculate the accleration value into actual g's
    ax = (float)acce_gyro_data[0] * aRes - acce_gyro_data[0]; // get actual g value, this depends on scale being set
    ay = (float)acce_gyro_data[1] * aRes - acce_gyro_data[1];
    az = (float)acce_gyro_data[2] * aRes - acce_gyro_data[2];

    // Calculate the gyro value into actual degrees per second
    gx = (float)acce_gyro_data[3] * gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)acce_gyro_data[4] * gRes; // - gyroBias[1];
    gz = (float)acce_gyro_data[5] * gRes; // - gyroBias[2];

    DEBUG_PRINTF("data: %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\n", ax,ay,az,gx,gy,gz);
    // DEBUG_PRINTF("raw data: %d, %d, %d, %d, %d, %d\n", ax,ay,az,gx,gy,gz);

    MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f);
    DEBUG_PRINTF("%0.3f, %0.3f, %0.3f mg\n", 1000 * ax, 1000 * ay, 1000 * az);
    DEBUG_PRINTF("%0.3f, %0.3f, %0.3f deg/s\n", gx,gy,gz);
}

