/*!
 * @file MFRC630.h
 */
#ifndef __MFRC630_H__
#define __MFRC630_H__

#include "Adafruit_MFRC630_consts.h"
#include "Adafruit_MFRC630_regs.h"
#include "stdint.h"
#include <SPI.h>
#include <stdbool.h>
//#include <Stream.h>
//#include <Wire.h>

/*!
 * @brief MFRC630 I2C Address
 */
#define MFRC630_I2C_ADDR (0x28)

/* Debug output level */
/*
 * NOTE: Setting this macro above RELEASE may require more SRAM than small
 *       MCUs like the Atmel 32u4 can provide!
 */
#define MFRC630_VERBOSITY_RELEASE (0) //!< No debug output
#define MFRC630_VERBOSITY_DEBUG (1)   //!< Debug message output
#define MFRC630_VERBOSITY_TRACE (2)   //!< Full packet trace dumps
#define MFRC630_VERBOSITY                                                      \
  (MFRC630_VERBOSITY_RELEASE) //!< Sets verbosity variable

#define MFRC630_ALWAYS_DISP_ERRORS (1) //!< Sets error output

/* Macro for debug output */
#if MFRC630_VERBOSITY >= MFRC630_VERBOSITY_DEBUG
#define DEBUG_PRINT(...) PRINT(__VA_ARGS__)
#define DEBUG_PRINTLN(...) PRINTln(__VA_ARGS__)
#define DEBUG_TIMESTAMP()                                                      \
  PRINT("\tD [+%lums] ", millis());PRINT("\ [+");
#else
#define DEBUG_PRINT(...)     //!< Disables debug printing
#define DEBUG_PRINTLN(...)   //!< Disables debug println
#define DEBUG_TIMESTAMP(...) //!< Disables debug timestamp
#endif

/* Macro for trace output */
#if MFRC630_VERBOSITY >= MFRC630_VERBOSITY_TRACE
#define TRACE_PRINT(...) PRINT(__VA_ARGS__)
#define TRACE_PRINTLN(...) PRINTln(__VA_ARGS__)
#define TRACE_TIMESTAMP()                                                      \
  PRINT("\t. [+%lums] ", millis());
#else
#define TRACE_PRINT(...)     //!< Disables trace output printing
#define TRACE_PRINTLN(...)   //!< Disables trace output println
#define TRACE_TIMESTAMP(...) //!< Disables trace output timestamp
#endif

/* Macro for error output */
#if MFRC630_ALWAYS_DISP_ERRORS
#define ERROR_PRINT(...) PRINT(__VA_ARGS__) //!< Enables error printing
#define ERROR_PRINTLN(...)                                                     \
  PRINTln(__VA_ARGS__) //!< Enables error println
#define ERROR_TIMESTAMP()                                                      \
  PRINT("\t! [+%lums] ", millis());PRINT("\t! [+");
#else
#define ERROR_PRINT(...) DEBUG_PRINT(__VA_ARGS__)
#define ERROR_PRINTLN(...) DEBUG_PRINTLN(__VA_ARGS__)
#define ERROR_TIMESTAMP()                                                      \
  PRINT("\t! [+%lums] ", millis());DEBUG_PRINT("\t! [+");
#endif

/**
 * Driver for the Adafruit mfrc630 RFID front-end.
 */
  /**
   * HW SPI bus constructor
   *
   * @param transport     The transport to use when communicating with the IC
   * @param cs            The CS/Sel pin for HW SPI access.
   * @param pdown_pin     The power down pin number (required)/
   *
   * @note This instance of the constructor requires the 'transport'
   *       parameter to distinguish is from the default I2C version.
   */
  void mfrc630(void);

  /**
   * Initialises the IC and performs some simple system checks.
   *
   * @return True if init succeeded, otherwise false.
   */
  bool mfrc630_begin(void);

  /* FIFO helpers (see section 7.5) */
  /**
   * Returns the number of bytes current in the FIFO buffer.
   *
   * @return The number of bytes in the FIFO buffer.
   */
  int16_t mfrc630_readFIFOLen(void);

  /**
   * Reads data from the FIFO buffer.
   *
   * @param len       The number of bytes to read
   * @param buffer    The buffer to write data into.
   *
   * @return The actual number of bytes read from the FIFO buffer.
   */
  int16_t mfrc630_readFIFO(uint16_t len, uint8_t *buffer);

  /**
   * Write sdata into the FIFO buffer.
   *
   * @param len       The number of bytes to write.
   * @param buffer    The data to write into the FIFO buffer.
   *
   * @return The actual number of bytes written.
   */
  int16_t mfrc630_writeFIFO(uint16_t len, uint8_t *buffer);

  /**
   * Clears the contents of the FIFO buffer.
   */
  void mfrc630_clearFIFO(void);

  /* Command wrappers */
  /**
   * Sends an unparameterized command to the IC.
   *
   * @param command   The command register to send.
   */
  void mfrc630_writeCommand(uint8_t command);

  /**
   * Sends a parametrized command to the IC.
   *
   * @param command   The command register to send.
   * @param paramlen  The number of parameter bytes.
   * @param params    The paramater values to send.
   */
  void mfrc630_writeCommand_param(uint8_t command, uint8_t paramlen, uint8_t *params);

  /* Radio config. */
  /**
   * Configures the radio for the specified protocol.
   *
   * @param cfg   The radio config setup to use.
   *
   * @return True if succeeded, otherwise false.
   */
  bool mfrc630_configRadio(mfrc630radiocfg cfg);

  /* General helpers */
  /**
   * Returns the current 'comm status' of the IC's internal state machine.
   *
   * @return The 8-bit state ID.
   */
  uint8_t mfrc630_getComStatus(void);

  /**
   * Performs a soft-reset to put the IC into a known state.
   */
  void mfrc630_softReset(void);

  /* Generic ISO14443a commands (common to any supported card variety). */
  /**
   * Sends the REQA command, requesting an ISO14443A-106 tag.
   *
   * @return The ATQA value if a card was detected.
   */
  uint16_t mfrc630_iso14443aRequest(void);

  /**
   * Sends the WUPA wakeup command.
   *
   * @return The ATQA value if a card was detected.
   */
  uint16_t mfrc630_iso14443aWakeup(void);

  /**
   * Selects a detected ISO14443A card, retrieving the UID and SAK.
   *
   * @param uid   Pointer to the buffer where the uid should be written.
   * @param sak   Pointer to the placeholder for the SAK value.
   *
   * @return True if init succeeded, otherwise false.
   */
  uint8_t mfrc630_iso14443aSelect(uint8_t *uid, uint8_t *sak);

  /* Mifare commands. */
  /**
   * Loads the specified authentication keys on the IC.
   *
   * @param key   Pointer to the buffer containing the key values.
   */
  void mfrc630_mifareLoadKey(uint8_t *key);

  /**
   * Authenticates the selected card using the previously supplied key/
   *
   * @param key_type  Whether to use KEYA or KEYB for authentication.
   * @param blocknum  The block number to authenticate.
   * @param uid       The UID of the card to authenticate.
   *
   * @return True if init succeeded, otherwise false.
   */
  bool mfrc630_mifareAuth(uint8_t key_type, uint8_t blocknum, uint8_t *uid);

  /**
   * Reads the contents of the specified (and previously authenticated)
   * memory block.
   *
   * @param blocknum  The block number to read.
   * @param buf       The buffer the data should be written into.
   *
   * @return The number of bytes read.
   */
  uint16_t mfrc630_mifareReadBlock(uint8_t blocknum, uint8_t *buf);

  /**
   * Writes the supplied data to the previously authenticated
   * memory block.
   *
   * @param blocknum  The block number to read.
   * @param buf       The buffer holding the data to write.
   *
   * @return The number of bytes written.
   */
  uint16_t mfrc630_mifareWriteBlock(uint16_t blocknum, uint8_t *buf);


  /* NTAG commands */
  /**
   * Reads the contents of the specified page.
   *
   * @param pagenum   The page number to read.
   * @param buf       The buffer the data should be written into.
   *
   * @return The number of bytes read.
   */
  uint16_t mfrc630_ntagReadPage(uint16_t pagenum, uint8_t *buf);

  /**
   * Writes the supplied content of the specified page.
   *
   * @param pagenum   The page number to write to.
   * @param buf       The data to write to the card.
   *
   * @return The number of bytes written.
   */
  uint16_t mfrc630_ntagWritePage(uint16_t pagenum, uint8_t *buf);

  void mfrc630_write8(uint8_t reg, uint8_t value);
  void mfrc630_writeBuffer(uint8_t reg, uint16_t len, uint8_t *buffer);
  uint8_t mfrc630_read8(uint8_t reg);

  void mfrc630_printHex(uint8_t *buf, size_t len);
  void mfrc630_printError(mfrc630errors err);

  uint16_t mfrc630_iso14443aCommand(iso14443_cmd cmd);

#endif
