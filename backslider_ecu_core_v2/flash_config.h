// flash_config.h
// Configuration for Adafruit W25Q128 SPI Flash module
// This file defines the flashTransport object needed by Adafruit_SPIFlash

#ifndef FLASH_CONFIG_H
#define FLASH_CONFIG_H

#if defined(ARDUINO) && !defined(TESTING)
#include <SPI.h>
#include "Adafruit_SPIFlash.h"

// External SPI Flash configuration for W25Q128
// Pin configuration matches your hardware setup:
// - CS   = Pin 10
// - MOSI = Pin 11  
// - MISO = Pin 12
// - SCK  = Pin 13
#define EXTERNAL_FLASH_USE_SPI
#define EXTERNAL_FLASH_USE_CS 10

// Create the flash transport object
// This is the object that Adafruit_SPIFlash will use to communicate with the chip
Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, SPI);

#endif // ARDUINO && !TESTING

#endif // FLASH_CONFIG_H 