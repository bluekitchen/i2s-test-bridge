# I2S Test Bridge for Desktop systems

Bluetooth Controllers historically provide SCO data via PCM/I2S interview. While most Embedded Linux systems have an I2S interface, most desktop systems don't have one.

This project aims to provide a mimimal I2S Interface over a regular UART connection using off-the-shelve hardware,
like the STM32L432KC Nucleo dev kit.

Hardware configuration for Nucleo STM32F091RC board:

  Signal  | GPIO | Header | Pin | Arduino | Logic
  --------|------|--------|-----|---------|------
  GND     |      | CN3    |  4  |  GND    | Gray
  SAI-SCK | PA8  | CN3    | 12  |  D9     | Black
  SAI-FS  | PA9  | CN3    |  1  |  D1     | Brown
  SAI_SDO | PA10 | CN3    |  2  |  D0     | Red 
  SAI-SDI | PB5  | CN3    | 14  | D11     | Orange

I2S Configuration:
- Slave
- Philips I2S
- 8 khz
- 16 bit

UART Configuration:
- 230400 baud (sufficient for 1 Channel, 8 kHz, 16 bit = 128 kbps)
- 8N1 (8 Bit data, 1 Stop Bit, No Parity)

## Usage

Compile the project using Cmake and flash onto the L432. As I2S is configured as slave, it will only start sending data once the clock starts.
The test app will send a 16 bit 266 Hz sine wave over the I2S SDO.

## TX Data

In main.c, you choose choose one of the following I2S output modes either by setting it in the code or via the SEGGER RTT Terminal:

- '1' Forward UART to I2S (default)
- '2' Sine    CVSD  8 kHz/16 bit, 266 Hz (default)
- '3' Sine    mSBC 16 kHz/16 bit, 266 Hz
- '4' Silence CVSD  8 kHz/16 bit
- '5' Silence mSBC 16 kHz/16 bit

## RX Data

Similarly, you can choose one of the following modes for the data sent over UART in big-endian format.

- 'a'  Forward Left I2S Channel data (default)
- 'b'. Sine    CVSD  8 kHz/16 bit, 266 Hz
- 'c'. Sine    mSBC 16 kHz/16 bit, 266 Hz
- 'd'. Silence CVSD  8 kHz/16 bit
- 'e'. Silence mSBC 16 kHz/16 bit
- 'f'. Test Data Counter
