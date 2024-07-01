# VL53L8CX
Arduino library to support the VL53L8CX Time-of-Flight 8x8 multizone ranging sensor with wide field view.

## API

This sensor uses I2C/SPI to communicate. And I2C/SPI instance is required to access to the sensor.
The APIs provide simple distance measure and multizone detection in both polling and interrupt modes.
The APIs derive from VL53L8CX ULD SDK v1.3.0.

## Examples

The examples contained in this library are based on VL53L8CX-SATEL sensor board.

You need to connect the VL53L8CX-SATEL sensor board directly to the Nucleo board with wires as explained below in the case of I2C communication:
 - pin 1 (SPI_I2C_n) of the VL53L8CX satellite connected to pin GND of the Nucleo board
 - pin 2 (LPn) of the VL53L8CX satellite connected to pin A3 of the Nucleo board
 - pin 3 (NCS) not connected
 - pin 4 (MISO) not connected
 - pin 5 (MOSI_SDA) of the VL53L8CX satellite connected to pin D14 (SDA) of the Nucleo board
 - pin 6 (MCLK_SCL) of the VL53L8CX satellite connected to pin D15 (SCL) of the Nucleo board
 - pin 7 (PWREN) of the VL53L8CX satellite connected to pin D11 of the Nucleo board
 - pin 8 (I0VDD) of the VL53L8CX satellite not connected
 - pin 9 (3V3) of the VL53L8CX satellite connected to 3V3 of the Nucleo board
 - pin 10 (1V8) of the VL53L8CX satellite not connected
 - pin 11 (5V) of the VL53L8CX satellite not connected 
 - GPIO1 of VL53L8CX satellite connected to A2 pin of the Nucleo board (not used)
 - GND of the VL53L8CX satellite connected to GND of the Nucleo board

You need to connect the VL53L8CX-SATEL sensor board directly to the Nucleo board with wires as explained below in the case of SPI communication:
 - pin 1 (SPI_I2C_n) of the VL53L8CX satellite connected to 3V3 of the Nucleo board
 - pin 2 (LPn) of the VL53L8CX satellite connected to 3V3 of the Nucleo board
 - pin 3 (NCS) of the VL53L8CX satellite connected to pin D10 of the Nucleo board
 - pin 4 (MISO) of the VL53L8CX satellite connected to pin D5 of the Nucleo board
 - pin 5 (MOSI_SDA) of the VL53L8CX satellite connected to pin D4 (MOSI) of the Nucleo board
 - pin 6 (MCLK_SCL) of the VL53L8CX satellite connected to pin D3 (MCLK_SCL) of the Nucleo board
 - pin 7 (PWREN) of the VL53L8CX satellite connected to pin D11 of the Nucleo board
 - pin 8 (I0VDD) of the VL53L8CX satellite not connected
 - pin 9 (3V3) of the VL53L8CX satellite connected to 3V3 of the Nucleo board
 - pin 10 (1V8) of the VL53L8CX satellite not connected
 - pin 11 (5V) of the VL53L8CX satellite not connected
 - GPIO1 of VL53L8CX satellite connected to A2 pin of the Nucleo board (not used)
 - GND of the VL53L8CX satellite connected to GND of the Nucleo board

There are 3 examples with the VL53L8CX library:

* VL53L8CX_Sat_HelloWorld_I2C: This example code is to show how to get multizone detection and proximity
  values of the VL53L8CX satellite sensor in polling mode using I2C communication.

* VL53L8CX_Sat_HelloWorld_SPI: This example code is to show how to get multizone detection and proximity
  values of the VL53L8CX satellite sensor in polling mode using SPI communication.

* VL53L8CX_ThresholdsDetection: This example code is to show how to configure the thresholds detection of the VL53L8CX satellite sensor.


## Documentation

You can find the source files at
https://github.com/stm32duino/VL53L8CX

The VL53L8CX datasheet is available at
https://www.st.com/en/imaging-and-photonics-solutions/VL53L8CX.html
