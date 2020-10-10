/***************************************************************************
  This is a library for the Adafruit MFRC630 Breakout

  Designed specifically to work with the Adafruit MFRC630 Breakout:
  http://www.adafruit.com/products/xxx

  These boards use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "stdint.h"

/* ANTENNA CONFIGURATION SETTINGS (registers 0x28..0x39) */
/* -------------------------------------------------------- */
/* ISO/IEC14443-A 106 */
uint8_t antcfg_iso14443a_106[18] = {0x8E, 0x12, 0x39, 0x0A, 0x18, 0x18,
                                    0x0F, 0x21, 0x00, 0xC0, 0x12, 0xCF,
                                    0x00, 0x04, 0x90, 0x5C, 0x12, 0x0A};
/* ISO/IEC14443-A 212 */
uint8_t antcfg_iso14443a_212[18] = {0x8E, 0xD2, 0x11, 0x0A, 0x18, 0x18,
                                    0x0F, 0x10, 0x00, 0xC0, 0x12, 0xCF,
                                    0x00, 0x05, 0x90, 0x3C, 0x12, 0x0B};
/* ISO/IEC14443-A 424 */
uint8_t antcfg_iso14443a_424[18] = {0x8F, 0xDE, 0x11, 0x0F, 0x18, 0x18,
                                    0x0F, 0x07, 0x00, 0xC0, 0x12, 0xCF,
                                    0x00, 0x06, 0x90, 0x2B, 0x12, 0x0B};
/* ISO/IEC14443-A 848 */
uint8_t antcfg_iso14443a_848[18] = {0x8F, 0xDB, 0x21, 0x0F, 0x18, 0x18,
                                    0x0F, 0x02, 0x00, 0xC0, 0x12, 0xCF,
                                    0x00, 0x07, 0x90, 0x3A, 0x12, 0x0B};

/* PROTOCOL CONFIGURATION SETTINGS */
/* -------------------------------------------------------- */
/* ISO/IEC14443-A 106/ MIFARE */
uint8_t protcfg_iso14443a_106[24] = {
    0x20, 0x00, 0x04, 0x50, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x50, 0x02, 0x00, 0x00, 0x01, 0x00, 0x08, 0x80, 0xB2};
/* ISO/IEC14443-A 212/ MIFARE */
uint8_t protcfg_iso14443a_212[24] = {
    0x20, 0x00, 0x05, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x50, 0x22, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x80, 0xB2};
/* ISO/IEC14443-A 424/ MIFARE */
uint8_t protcfg_iso14443a_424[24] = {
    0x20, 0x00, 0x06, 0x50, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x50, 0x22, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x80, 0xB2};
/* ISO/IEC14443-A 848/ MIFARE */
uint8_t protcfg_iso14443a_848[24] = {
    0x20, 0x00, 0x07, 0x50, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x50, 0x22, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x80, 0xB2};
