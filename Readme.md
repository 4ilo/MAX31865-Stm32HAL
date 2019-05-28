# MAX31865 RTD-to-Digital Converter - stm32-HAL


MAX31865 temperature readout for stm32 using stm32-hal library's.

Library is developed and tested with Stm32F411-discovery.

Features:
- Pt100 support
- Software SPI
- 2, 3 and 4 wire temperature probe connection possible

## Example

```c
// Define SPI pinout
MAX31865_GPIO max_gpio;
max_gpio.MISO_PIN = MAX_MISO_Pin;
max_gpio.MISO_PORT = MAX_MISO_GPIO_Port;
max_gpio.MOSI_PIN = MAX_MOSI_Pin;
max_gpio.MOSI_PORT = MAX_MOSI_GPIO_Port;
max_gpio.CLK_PIN = MAX_CLK_Pin;
max_gpio.CLK_PORT = MAX_CLK_GPIO_Port;
max_gpio.CE_PIN = MAX_CE_Pin;
max_gpio.CE_PORT = MAX_CE_GPIO_Port;

// Initialize using the above pinout definition
// Use a 3 wire pt100
MAX31865_init(&max_gpio, 3);

// Perform a single shot conversion, and calculate the temperature
float temp = MAX31865_readTemp();
```