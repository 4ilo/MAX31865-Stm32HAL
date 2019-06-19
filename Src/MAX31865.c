/**
 * Olivier Van den Eede (4ilo) 2019
 */
#include "MAX31865.h"


/*********************** Global variables *************************/
MAX31865_GPIO *gpio;


/*********************** Begin Private functions *************************/
/**
 * Write x bytes using software SPI
 *
 * @param data Pointer to transmit buffer
 * @param len  Amount of bytest to send
 */
void spi_write(uint8_t *data, uint8_t len)
{
    for (uint8_t x = 0; x < len; x++)
    {
        for (int8_t i = 7; i >= 0; i--)
        {
            HAL_GPIO_WritePin(gpio->MOSI_PORT, gpio->MOSI_PIN, (data[x] & (1 << i)));
            DELAY(1);
            HAL_GPIO_WritePin(gpio->CLK_PORT, gpio->CLK_PIN, 1);
            DELAY(1);
            HAL_GPIO_WritePin(gpio->CLK_PORT, gpio->CLK_PIN, 0);
        }
    }

}

/**
 * Read x bytes using software SPI
 *
 * @param buffer Pointer to rx buffer
 * @param len Amount of bytes to receive
 */
void spi_read(uint8_t *buffer, uint8_t len)
{
    for (uint8_t x = 0; x < len; x++)
    {
        buffer[x] = 0;

        for (int8_t i = 7; i >= 0; i--)
        {
            buffer[x] <<= 1;
            HAL_GPIO_WritePin(gpio->CLK_PORT, gpio->CLK_PIN, 1);
            DELAY(1);
            buffer[x] |= HAL_GPIO_ReadPin(gpio->MISO_PORT, gpio->MISO_PIN);
            DELAY(1);
            HAL_GPIO_WritePin(gpio->CLK_PORT, gpio->CLK_PIN, 0);
        }
    }
}

/**
 * Read x bytes from MAX31865 starting from addr
 *
 * @param addr      Register addr to read from
 * @param buffer    Pointer to rx buffer
 * @param len       Amount of bytes to read
 */
void MAX31865_read(uint8_t addr, uint8_t *buffer, uint8_t len)
{
    addr &= ~MAX31865_READ;                                     // Force read bit on address

    HAL_GPIO_WritePin(gpio->CE_PORT, gpio->CE_PIN, 0);          // Enable CE

    spi_write(&addr, 1);                                        // Write addr
    spi_read(buffer, len);                                      // Read data

    HAL_GPIO_WritePin(gpio->CE_PORT, gpio->CE_PIN, 1);          // Disable CE
}

/**
 * Write a byte in a MAX13865 register
 *
 * @param addr      Register addr to write to
 * @param buffer    Tx data
 */
void MAX31865_write(uint8_t addr, uint8_t data)
{
    addr |= MAX31865_WRITE;                                 // Force write bit on address

    HAL_GPIO_WritePin(gpio->CE_PORT, gpio->CE_PIN, 0);      // Enable CE

    spi_write(&addr, 1);                                    // Write addr
    spi_write(&data, 1);                                    // Write data

    HAL_GPIO_WritePin(gpio->CE_PORT, gpio->CE_PIN, 1);      // Disable CE
}

/**
 * Enable of disable MAX831865 bias voltage
 * @param enable Enable of disable
 */
void enableBias(uint8_t enable)
{
    uint8_t status;
    MAX31865_read(MAX31856_CONFIG_REG, &status, 1);

    if (enable)
    {
        status |= MAX31856_CONFIG_BIAS;
    } else
    {
        status &= ~MAX31856_CONFIG_BIAS;
    }

    MAX31865_write(MAX31856_CONFIG_REG, status);
}

/**
 * Enable of disable MAX831865 auto convert
 * @param enable Enable of disable
 */
void autoConvert(uint8_t enable)
{
    uint8_t status;
    MAX31865_read(MAX31856_CONFIG_REG, &status, 1);

    if (enable)
    {
        status |= MAX31856_CONFIG_MODEAUTO;
    } else
    {
        status &= ~MAX31856_CONFIG_MODEAUTO;
    }

    MAX31865_write(MAX31856_CONFIG_REG, status);
}

/**
 * Set the amount of wires the temperature sensor uses
 * @param numwires 2,3 or 4 wires
 */
void setWires(uint8_t numwires)
{
    uint8_t status;
    MAX31865_read(MAX31856_CONFIG_REG, &status, 1);

    if (numwires == 3) // 3-wire
    {
        status |= MAX31856_CONFIG_3WIRE;
    } else // 2-4 wire
    {
        status &= ~MAX31856_CONFIG_3WIRE;
    }

    MAX31865_write(MAX31856_CONFIG_REG, status);
}

/**
 * Perform a single shot conversion
 */
void single_shot(void)
{
    uint8_t status;

    // Read config register
    MAX31865_read(MAX31856_CONFIG_REG, &status, 1);

    // Enable 1shot bit, and write back
    status |= MAX31856_CONFIG_1SHOT;
    MAX31865_write(MAX31856_CONFIG_REG, status);

}
/*********************** End Private functions *************************/


/*********************** Begin Public functions *************************/

/**
 * Initialise MAX31865 for single shot temperature conversion
 *
 * @param max_gpio  MAX31865_GPIO structure with pinout
 * @param wires     Amount of wires on the temperature probe (2,3 or 4)
 */
void MAX31865_init(MAX31865_GPIO *max_gpio, uint8_t wires)
{
    gpio = max_gpio;

    // Datalines in reset state
    HAL_GPIO_WritePin(gpio->CE_PORT, gpio->CE_PIN, 1);
    HAL_GPIO_WritePin(gpio->CLK_PORT, gpio->CLK_PIN, 1);
    HAL_GPIO_WritePin(gpio->MOSI_PORT, gpio->MOSI_PIN, 1);

    setWires(wires);           // Set 2,3 or 4 wire sensor
    enableBias(OFF);           // Disable bias voltage
    autoConvert(OFF);          // Disable auto conversion
}

/**
 * Perform a single temperature conversion, and calculate the value
 *
 * @return  Temperature as float
 */
float MAX31865_readTemp()
{
    // Activate bias voltage to read sensor data, and wait for the capacitors to fill
    enableBias(ON);
    HAL_Delay(10);

    // Perform a single conversion, and wait for the result
    single_shot();
    HAL_Delay(65);

    // Read data from max31865 data registers
    uint8_t buffer[2];
    MAX31865_read(MAX31856_RTDMSB_REG, buffer, 2);

    // Combine 2 bytes into 1 number, and shift 1 down to remove fault bit
    uint16_t data = buffer[0] << 8;
    data |= buffer[1];
    data >>= 1;

    // Calculate the actual resistance of the sensor
    float resistance = ((float) data * RREF) / FACTOR;

    // Calculate the temperature from the measured resistance
    float temp = ((resistance / 100) - 1) / ALPHA;

    // Disable bias voltage to reduce power usage
    enableBias(OFF);

    return temp;
}
/*********************** Begin Public functions *************************/