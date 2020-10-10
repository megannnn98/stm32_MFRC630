/*!
 * @file Adafruit_MFRC630.cpp
 *
 * @mainpage Adafruit MFRC630 Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the Adafruit MFRC630 Breakout
 *
 * Designed specifically to work with the Adafruit MFRC630 Breakout:
 * http://www.adafruit.com/products/xxx
 *
 * These boards use I2C to communicate, 2 pins are required to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit andopen-source hardware by purchasing products
 * from Adafruit!
 *
 * @section author Author
 *
 * Written by Kevin Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_MFRC630.h"
#include "main.h"


void digitalWrite(uint8_t v8)
{
  HAL_GPIO_WritePin(RFID_CS_GPIO_Port, RFID_CS_Pin, (LOW == v8) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint8_t spi_exchange(uint8_t v8)
{
    LL_SPI_TransmitData8(RFID_SPI, v8);
    while (SPI_SR_RXNE != (RFID_SPI->SR & SPI_SR_RXNE))
        ;
    return LL_SPI_ReceiveData8(RFID_SPI);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

static unsigned char rev8_lookup[16] = {0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
                                        0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf};

/*!
 * @brief Uses the lookup table above to reverse a single uint8_t.
 * @param n Input uint8_t
 * @return uint8_t. A uint8_t
 */
uint8_t reverse8(uint8_t n) {
  return (rev8_lookup[n & 0b1111] << 4) | rev8_lookup[n >> 4];
}

/**************************************************************************/
/*!
    @brief  Write a uint8_t to the specified register
*/
/**************************************************************************/
void Adafruit_MFRC630::write8(uint8_t reg, uint8_t value) {
  TRACE_TIMESTAMP();
  TRACE_PRINT("Writing 0x");
  TRACE_PRINT(value, HEX);
  TRACE_PRINT(" to 0x");
  TRACE_PRINTLN(reg, HEX);

  /* SPI */
  digitalWrite(LOW);
  spi_exchange((reg << 1) | 0x00);
  spi_exchange(value);
  digitalWrite(HIGH);

}

/**************************************************************************/
/*!
    @brief  Write a buffer to the specified register
*/
/**************************************************************************/
void Adafruit_MFRC630::writeBuffer(uint8_t reg, uint16_t len, uint8_t *buffer) {
  TRACE_TIMESTAMP();
  TRACE_PRINT("Writing ");
  TRACE_PRINT(len);
  TRACE_PRINT(" uint8_t(s) to 0x");
  TRACE_PRINTLN(reg, HEX);

  TRACE_TIMESTAMP();

  /* SPI */
  digitalWrite(LOW);
  spi_exchange((reg << 1) | 0x00);
  for (uint8_t i = 0; i < len; i++) {
    spi_exchange(buffer[i]);
    TRACE_PRINT("0x");
    TRACE_PRINT(buffer[i], HEX);
    TRACE_PRINT(" ");
  }
  digitalWrite(HIGH);

  TRACE_PRINTLN("");
}

/**************************************************************************/
/*!
    @brief  Read a uint8_t from the specified register
*/
/**************************************************************************/
uint8_t Adafruit_MFRC630::read8(uint8_t reg) {
  uint8_t resp = 0;
  uint8_t tx[2] = {0};
  uint8_t rx[2] = {0};

  TRACE_TIMESTAMP();
  TRACE_PRINT("Requesting 1 uint8_t from 0x");
  TRACE_PRINTLN(reg, HEX);

  /* SPI */
  digitalWrite(LOW);
  tx[0] = (reg << 1) | 0x01;
  tx[1] = 0;
  rx[0] = spi_exchange(tx[0]);
  rx[1] = spi_exchange(tx[1]);
  digitalWrite(HIGH);
  resp = rx[1];


  TRACE_TIMESTAMP();
  TRACE_PRINT("Response = ");
  TRACE_PRINT(" 0x");
  if (resp <= 0xF) {
    TRACE_PRINT("0");
  }
  TRACE_PRINT(resp, HEX);
  TRACE_PRINTLN("");

  return resp;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new instance of the Adafruit_MFRC630 class
            using the default I2C bus.
*/
/**************************************************************************/
/*!
    @brief  Instantiates a new instance of the Adafruit_MFRC630 class
            using the HW SPI bus.
*/
/**************************************************************************/
Adafruit_MFRC630::Adafruit_MFRC630() {

}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Sets up the HW
*/
/**************************************************************************/
bool Adafruit_MFRC630::begin() {
  /* Display alert for DEBUG and TRACE output. */
  DEBUG_PRINTLN("\tDebug output enabled: D [+ms] Message");
  TRACE_PRINTLN("\tTrace output enabled: . [+ms] Message");
  DEBUG_PRINTLN("");

  /* Enable I2C, SPI or SW serial */
  DEBUG_TIMESTAMP();

  /* Check device ID for bus response */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Checking transport layer");

  /* Read the VERSION register */
  uint8_t ver = read8(MFRC630_REG_VERSION);

  /* If ver == 0xFF or 0x0 likely a bus failure */
  if ((ver == 0xFF) || (ver == 0)) {
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("Transport failure!");
    return false;
  }

  /* If !1.8, there was a problem */
  if (ver != 0x18) {
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("FAILED!");
    return false;
  }

  DEBUG_TIMESTAMP();
  DEBUG_PRINT("IC Version = ");
  DEBUG_PRINT((ver & 0xF0) >> 4, HEX);
  DEBUG_PRINT(".");
  DEBUG_PRINTLN(ver & 0x0F, HEX);

  return true;
}

/**************************************************************************/
/*!
    @brief  Determines the number of uint8_ts in the HW FIFO buffer (max 512)

    @returns The number of uint8_ts available in the HW FIFO buffer
*/
/**************************************************************************/
int16_t Adafruit_MFRC630::readFIFOLen(void) {
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Checking FIFO length");

  /* Give FIFO a chance to fill up. */
  /* TODO: Why do we need a delay between reads?!? */
  delay(10);

  /* Read the MFRC630_REG_FIFO_LENGTH register */
  /* In 512 uint8_t mode, the upper two bits are stored in FIFO_CONTROL */
  uint8_t hi = read8(MFRC630_REG_FIFO_CONTROL);
  uint8_t lo = read8(MFRC630_REG_FIFO_LENGTH);

  /* Determine len based on FIFO size (255 uint8_t or 512 uint8_t mode) */
  int16_t l = (hi & 0x80) ? lo : (((hi & 0x3) << 8) | lo);

  DEBUG_TIMESTAMP();
  DEBUG_PRINT("FIFO contains ");
  DEBUG_PRINT(l);
  DEBUG_PRINTLN(" uint8_t(s)");

  return l;
}

/**************************************************************************/
/*!
    @brief  Read 'len' uint8_ts from the HW FIFO buffer (max 512 uint8_ts)

    @returns The number of uint8_ts read from the FIFO, or -1 if an error occured.
*/
/**************************************************************************/
int16_t Adafruit_MFRC630::readFIFO(uint16_t len, uint8_t *buffer) {
  int16_t counter = 0;

  /* Check for 512 uint8_t overflow */
  if (len > 512) {
    return -1;
  }

  DEBUG_TIMESTAMP();
  DEBUG_PRINT("Reading ");
  DEBUG_PRINT(len);
  DEBUG_PRINTLN(" uint8_t(s) from FIFO");

  /* Read len uint8_ts from the FIFO */
  for (uint16_t i = 0; i < len; i++) {
    buffer[i] = read8(MFRC630_REG_FIFO_DATA);
    counter++;
  }

  return counter;
}

/**************************************************************************/
/*!
    @brief  Writes the specified number of uint8_ts to the HW FIFO

    @returns The number of uint8_ts written to the FIFO, -1 if an error occured.
*/
/**************************************************************************/
int16_t Adafruit_MFRC630::writeFIFO(uint16_t len, uint8_t *buffer) {
  int counter = 0;

  /* Check for 512 uint8_t overflow */
  if (len > 512) {
    return -1;
  }

  DEBUG_TIMESTAMP();
  DEBUG_PRINT("Writing ");
  DEBUG_PRINT(len);
  DEBUG_PRINTLN(" uint8_t(s) to FIFO");

  /* Write len uint8_ts to the FIFO */
  for (uint16_t i = 0; i < len; i++) {
    write8(MFRC630_REG_FIFO_DATA, buffer[i]);
    counter++;
  }

  return counter;
}

/**************************************************************************/
/*!
    @brief  Flushes the contents of the FIFo buffer
*/
/**************************************************************************/
void Adafruit_MFRC630::clearFIFO(void) {
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Clearing FIFO buffer ");

  uint8_t ctrl = read8(MFRC630_REG_FIFO_CONTROL);
  write8(MFRC630_REG_FIFO_CONTROL, ctrl | (1 << 4));
}

/**************************************************************************/
/*!
    @brief  Writes a parameter-less command to the internal state machine
*/
/**************************************************************************/
void Adafruit_MFRC630::writeCommand(uint8_t command) {
  uint8_t buff[1] = {command};

  DEBUG_TIMESTAMP();
  DEBUG_PRINT("Sending CMD 0x");
  DEBUG_PRINTLN(command, HEX);

  writeBuffer(MFRC630_REG_COMMAND, 1, buff);
}

/**************************************************************************/
/*!
    @brief  Writes a parameterised command to the internal state machine
*/
/**************************************************************************/
void Adafruit_MFRC630::writeCommand(uint8_t command, uint8_t paramlen,
                                    uint8_t *params) {
  /* Arguments and/or data necessary to process a command are exchanged via
     the FIFO buffer:

     - Each command that needs a certain number of arguments will start
       processing only when it has received the correct number of arguments
       via the FIFO buffer.
     - The FIFO buffer is not cleared automatically at command start. It is
       recommended to write the command arguments and/or the data uint8_ts into
       the FIFO buffer and start the command afterwards.
     - Each command may be stopped by the host by writing a new command code
       into the command register e.g.: the Idle-Command. */

  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Sending Command");

  /* Cancel any current command. */
  write8(MFRC630_REG_COMMAND, MFRC630_CMD_IDLE);

  /* Flush the FIFO */
  clearFIFO();

  /* Write data to the FIFO */
  writeFIFO(paramlen, params);

  /* Send the command */
  write8(MFRC630_REG_COMMAND, command);
}

/**************************************************************************/
/*!
    @brief  Gets the three bit COM status for the IC
*/
/**************************************************************************/
uint8_t Adafruit_MFRC630::getComStatus(void) {
  return (read8(MFRC630_REG_STATUS) & 0b111);
}

/**************************************************************************/
/*!
    @brief  Performs a soft reset of the IC
*/
/**************************************************************************/
void Adafruit_MFRC630::softReset(void) {
  writeCommand(MFRC630_CMD_SOFTRESET);
  delay(100);
}

/**************************************************************************/
/*!
    @brief  Prints out n uint8_ts of hex data.
*/
/**************************************************************************/
void Adafruit_MFRC630::printHex(uint8_t *buf, size_t len) {
  if (len) {
    DEBUG_PRINT("[ ");
  }
  for (uint8_t i = 0; i < len; i++) {
    // DEBUG_PRINT("0x");
    if (buf[i] < 16) {
      DEBUG_PRINT("0");
    }
    DEBUG_PRINT(buf[i], HEX);
    DEBUG_PRINT(" ");
  }
  if (len) {
    DEBUG_PRINT("]");
  }
}

/**************************************************************************/
/*!
    @brief  Prints a human readable error code.
*/
/**************************************************************************/
void Adafruit_MFRC630::printError(enum mfrc630errors err) {
//  ERROR_TIMESTAMP();
//  ERROR_PRINT("ERROR! Danger, Will Robinson: ");
//
//  switch (err) {
//  case MFRC630_ERROR_INTEG:
//    ERROR_PRINTLN("Data integrity!");
//    break;
//  case MFRC630_ERROR_PROT:
//    ERROR_PRINTLN("Protocol error!");
//    break;
//  case MFRC630_ERROR_COLLDET:
//    ERROR_PRINTLN("Collision detected!");
//    break;
//  case MFRC630_ERROR_NODATA:
//    ERROR_PRINTLN("No data!");
//    break;
//  case MFRC630_ERROR_MINFRAME:
//    ERROR_PRINTLN("Frame data too small!");
//    break;
//  case MFRC630_ERROR_FIFOOVL:
//    ERROR_PRINTLN("FIFO full!");
//    break;
//  case MFRC630_ERROR_FIFOWR:
//    ERROR_PRINTLN("Couldn't write to FIFO!");
//    break;
//  case MFRC630_ERROR_EEPROM:
//    ERROR_PRINTLN("EEPROM access!");
//    break;
//  default:
//    ERROR_PRINT("Unhandled error code: ");
//    ERROR_PRINTLN(err, HEX);
//    break;
//  }
//
//  /* Halt execution here if we're not in release mode! */
//#if MFRC630_VERBOSITY > MFRC630_VERBOSITY_RELEASE
//  while (1) {
//    delay(1);
//  }
//#endif
}

/**************************************************************************/
/*!
    @brief  Configures the radio for the specified protocol
*/
/**************************************************************************/
bool Adafruit_MFRC630::configRadio(mfrc630radiocfg cfg) {
  DEBUG_TIMESTAMP();
  DEBUG_PRINT("Configuring the radio for ");

  switch (cfg) {
  case MFRC630_RADIOCFG_ISO1443A_106:
    DEBUG_PRINTLN("ISO1443A-106");
    writeBuffer(MFRC630_REG_DRV_MOD, sizeof(antcfg_iso14443a_106),
                antcfg_iso14443a_106);

    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("Setting driver mode");
    write8(MFRC630_REG_DRV_MOD, 0x8E); /* Driver mode register */

    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("Setting transmitter amplifier (residual carrier %)");
    write8(MFRC630_REG_TX_AMP, 0x12); /* Transmiter amplifier register */

    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("Setting driver configuration");
    write8(MFRC630_REG_DRV_CON, 0x39); /* Driver configuration register */

    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("Configuring transmitter (overshoot/TX load)");
    write8(MFRC630_REG_TXL, 0x06); /* Transmitter register */
    break;
  default:
    DEBUG_PRINTLN("[UNKNOWN!]");
    return false;
    break;
  }

  return true;
}

uint16_t Adafruit_MFRC630::iso14443aRequest(void) {
  return iso14443aCommand(ISO14443_CMD_REQA);
}

uint16_t Adafruit_MFRC630::iso14443aWakeup(void) {
  return iso14443aCommand(ISO14443_CMD_WUPA);
}

uint16_t Adafruit_MFRC630::iso14443aCommand(enum iso14443_cmd cmd) {
  uint16_t atqa = 0; /* Answer to request (2 uint8_ts). */

  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Checking for an ISO14443A tag");

  /* Cancel any current command */
  writeCommand(MFRC630_CMD_IDLE);

  /* Flush the FIFO */
  clearFIFO();

  /*
   * Define the number of bits from the last uint8_t should be sent. 000 means
   * that all bits of the last data uint8_t are sent, 1..7 causes the specified
   * number of bits to be sent. Also set the DataEn bit to enable data xfer.
   */
  write8(MFRC630_REG_TX_DATA_NUM, 0x07 | (1 << 3));

  /* Disable CRC. */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("A. Disabling CRC checks.");
  write8(MFRC630_REG_TX_CRC_PRESET, 0x18);
  write8(MFRC630_REG_RX_CRC_CON, 0x18);

  /* Clear the receiver control register. */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("B. Clearing the receiver control register.");
  write8(MFRC630_REG_RX_BIT_CTRL, 0);

  /* Clear the interrupts. */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("C. Clearing and configuring interrupts.");
  write8(MFRC630_REG_IRQ0, 0b01111111);
  write8(MFRC630_REG_IRQ1, 0b00111111);
  /* Allow the receiver and Error IRQs to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQOEN, MFRC630IRQ0_RXIRQ | MFRC630IRQ0_ERRIRQ);
  /* Allow Timer0 IRQ to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ1EN, MFRC630IRQ1_TIMER0IRQ);

  /* Configure the frame wait timeout using T0 (5ms max). */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("D. Configuring Timer0 @ 211.875kHz, post TX, 5ms timeout.");
  write8(MFRC630_REG_T0_CONTROL, 0b10001);
  write8(MFRC630_REG_T0_RELOAD_HI, 1100 >> 8);
  write8(MFRC630_REG_TO_RELOAD_LO, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_HI, 1100 >> 8);
  write8(MFRC630_REG_T0_COUNTER_VAL_LO, 0xFF);

  /* Send the ISO14443 command. */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("E. Sending ISO14443 command.");
  uint8_t send_req[] = {(uint8_t)cmd};
  writeCommand(MFRC630_CMD_TRANSCEIVE, 1, send_req);

  /* Wait here until we're done reading, get an error, or timeout. */
  /* TODO: Update to use timeout parameter! */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("F. Waiting for a response or timeout.");
  uint8_t irqval = 0;
  while (!(irqval & MFRC630IRQ1_TIMER0IRQ)) {
    irqval = read8(MFRC630_REG_IRQ1);
    /* Check for a global interrrupt, which can only be ERR or RX. */
    if (irqval & MFRC630IRQ1_GLOBALIRQ) {
      break;
    }
  }

  /* Cancel the current command (in case we timed out or error occurred). */
  writeCommand(MFRC630_CMD_IDLE);

  /* Check the RX IRQ, and exit appropriately if it has fired (error). */
  irqval = read8(MFRC630_REG_IRQ0);
  if ((!(irqval & MFRC630IRQ0_RXIRQ) || (irqval & MFRC630IRQ0_ERRIRQ))) {
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("ERROR: No RX flag set, transceive failed or timed out.");
    /* Display the error message if ERROR IRQ is set. */
    if (irqval && MFRC630IRQ0_ERRIRQ) {
      uint8_t error = read8(MFRC630_REG_ERROR);
      /* Only display the error if it isn't a timeout. */
      if (error) {
        printError((enum mfrc630errors)error);
      }
    }
    return 0;
  }

  /* Read the response */
  uint16_t rxlen = readFIFOLen();
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("G. Reading response from FIFO buffer.");
  if (rxlen == 2) {
    /*
     * If we have 2 uint8_ts for the response, it's the ATQA.
     *
     * See ISO14443-3 6.3.2 for help in interpretting the ATQA value.
     *
     * "After a REQA Command is transmitted by the PCD, all
     * PICCs in the IDLE State shall respond synchronously with ATQA."
     *
     * 0x44 = 4 bit frame anticollision
     *        UID size = double
     */
    readFIFO(rxlen, (uint8_t *)&atqa);
    DEBUG_TIMESTAMP();
    DEBUG_PRINT("Received response (ATQA): 0x");
    DEBUG_PRINTLN(atqa, HEX);
    return atqa;
  }

  return 0;
}

/*
 * For high level details on the selection and anti-collision protocols see
 * "Chip Type Identification Procedure" in
 * https://www.nxp.com/docs/en/application-note/AN10833.pdf
 */
uint8_t Adafruit_MFRC630::iso14443aSelect(uint8_t *uid, uint8_t *sak) {
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Selecting an ISO14443A tag");

  /* Cancel any current command */
  writeCommand(MFRC630_CMD_IDLE);

  /* Flush the FIFO */
  clearFIFO();

  /* Allow the receiver and Error IRQs to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQOEN, MFRC630IRQ0_RXIRQ | MFRC630IRQ0_ERRIRQ);

  /* Allow Timer0 IRQ to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ1EN, MFRC630IRQ1_TIMER0IRQ);

  /* Configure the frame wait timeout using T0 (5ms max). */
  /* 1 'tick' 4.72us, so 1100 = 5.2ms */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("A. Configuring Timer0 @ 211.875kHz, post TX, 5ms timeout.");
  write8(MFRC630_REG_T0_CONTROL, 0b10001);
  write8(MFRC630_REG_T0_RELOAD_HI, 1100 >> 8);
  write8(MFRC630_REG_TO_RELOAD_LO, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_HI, 1100 >> 8);
  write8(MFRC630_REG_T0_COUNTER_VAL_LO, 0xFF);

  /* Set the cascade level (collision detection loop) */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("B. Checking cascade level (collision detection).");
  uint8_t cascadelvl;
  for (cascadelvl = 1; cascadelvl <= 3; cascadelvl++) {
    uint8_t cmd;
    uint8_t kbits = 0;                        /* Bits known in UID so far. */
    uint8_t send_req[7] = {0};                /* TX buffer */
    uint8_t *uid_this_level = &(send_req[2]); /* UID pointer */
    uint8_t message_length;

    DEBUG_TIMESTAMP();
    DEBUG_PRINT("Cascade level ");
    DEBUG_PRINTLN(cascadelvl);

    switch (cascadelvl) {
    case 1:
      cmd = ISO14443_CAS_LEVEL_1;
      break;
    case 2:
      cmd = ISO14443_CAS_LEVEL_2;
      break;
    case 3:
      cmd = ISO14443_CAS_LEVEL_3;
      break;
    }

    /* Disable CRC. */
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("a. Disabling CRC checks.");
    write8(MFRC630_REG_TX_CRC_PRESET, 0x18);
    write8(MFRC630_REG_RX_CRC_CON, 0x18);

    /* As per ISO14443-3, limit coliision checks to 32 attempts. */
    uint8_t cnum;
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("b. Collision detection (max 32 attempts).");
    for (cnum = 0; cnum < 32; cnum++) {
      DEBUG_TIMESTAMP();
      DEBUG_PRINT("Attempt = ");
      DEBUG_PRINT(cnum);
      DEBUG_PRINT(", known bits = ");
      DEBUG_PRINT(kbits);
      DEBUG_PRINT(" ");
      printHex(uid_this_level, (kbits + 8 - 1) / 8);
      DEBUG_PRINTLN("");

      /* Clear the interrupts. */
      write8(MFRC630_REG_IRQ0, 0b01111111);
      write8(MFRC630_REG_IRQ1, 0b00111111);

      /* Send the current collision level command */
      send_req[0] = cmd;
      send_req[1] = 0x20 + kbits;

      /* Limit MFRC630_REG_TX_DATA_NUM to the correct number of bits. */
      write8(MFRC630_REG_TX_DATA_NUM, (kbits % 8) | (1 << 3));

      // ValuesAfterColl: If cleared, every received bit after a collision is
      // replaced by a zero. This function is needed for ISO/IEC14443
      // anticollision (0<<7). We want to shift the bits with RxAlign
      uint8_t rxalign = kbits % 8;
      write8(MFRC630_REG_RX_BIT_CTRL, (0 << 7) | (rxalign << 4));

      /* Determine the message length */
      if ((kbits % 8) == 0) {
        message_length = ((kbits / 8)) + 2;
      } else {
        message_length = ((kbits / 8) + 1) + 2;
      }

      /* Send the command. */
      writeCommand(MFRC630_CMD_TRANSCEIVE, message_length, send_req);

      /* Wait until the command execution is complete. */
      uint8_t irq1_value = 0;
      while (!(irq1_value & MFRC630IRQ1_TIMER0IRQ)) {
        irq1_value = read8(MFRC630_REG_IRQ1);
        /* Check for a global interrrupt, which can only be ERR or RX. */
        if (irq1_value & MFRC630IRQ1_GLOBALIRQ) {
          break;
        }
      }

      /* Cancel any current command */
      writeCommand(MFRC630_CMD_IDLE);

      /* Parse results */
      uint8_t irq0_value = read8(MFRC630_REG_IRQ0);
      uint8_t error = read8(MFRC630_REG_ERROR);
      uint8_t coll = read8(MFRC630_REG_RX_COLL);
      uint8_t coll_p = 0;

      /* Check if an error occured */
      if (irq0_value & MFRC630IRQ0_ERRIRQ) {
        /* Display the error code in human-readable format. */
        printError((enum mfrc630errors)error);
        if (error & MFRC630_ERROR_COLLDET) {
          /* Collision error, check if the collision position is valid */
          if (coll & (1 << 7)) {
            /* Valid, so check the collision position (bottom 7 bits). */
            coll_p = coll & (~(1 << 7));
            DEBUG_TIMESTAMP();
            DEBUG_PRINT("Bit collision detected at bit ");
            DEBUG_PRINTLN(coll_p);

            uint8_t choice_pos = kbits + coll_p;
            uint8_t selection =
                (uid[((choice_pos + (cascadelvl - 1) * 3) / 8)] >>
                 ((choice_pos) % 8)) &
                1;
            uid_this_level[((choice_pos) / 8)] |= selection
                                                  << ((choice_pos) % 8);
            kbits++;

            DEBUG_TIMESTAMP();
            DEBUG_PRINT("'uid_this_level' is now ");
            DEBUG_PRINT(kbits);
            DEBUG_PRINT(": ");
            printHex(uid_this_level, 10);
            DEBUG_PRINTLN("");
          } else {
            /* Invalid collision position (bit 7 = 0) */
            DEBUG_TIMESTAMP();
            DEBUG_PRINTLN("Bit collision detected, but no valid position.");
            coll_p = 0x20 - kbits;
          } /* End: if (coll & (1 << 7)) */
        } else {
          DEBUG_TIMESTAMP();
          DEBUG_PRINTLN("Unhandled error.");
          coll_p = 0x20 - kbits;
        } /* End: if (error & MFRC630_ERROR_COLLDET) */
      } else if (irq0_value & MFRC630IRQ0_RXIRQ) {
        /* We have data and no collision, all is well in the world! */
        coll_p = 0x20 - kbits;
        DEBUG_TIMESTAMP();
        DEBUG_PRINTLN("Received data, no bit collision!");
      } else {
        /* Probably no card */
        DEBUG_TIMESTAMP();
        DEBUG_PRINTLN("No error and no data = No card");
        return 0;
      } /* End: if (irq0_value & (1 << 1)) */

      /* Read the UID so far */
      uint16_t rxlen = readFIFOLen();
      uint8_t buf[5]; /* UID = 4 uint8_ts + BCC */
      readFIFO(rxlen < 5 ? rxlen : 5, buf);

      /*
       * Move current buffer contents into the UID placeholder, OR'ing the
       * results so that we don't lose the bit we set if you have a collision.
       */
      uint8_t rbx;
      for (rbx = 0; (rbx < rxlen); rbx++) {
        uid_this_level[(kbits / 8) + rbx] |= buf[rbx];
      }
      kbits += coll_p;

      if ((kbits >= 32)) {
        DEBUG_TIMESTAMP();
        DEBUG_PRINT("Leaving collision loop: uid ");
        DEBUG_PRINT(kbits);
        DEBUG_PRINTLN(" bits long");
        DEBUG_TIMESTAMP();
        printHex(uid_this_level, kbits / 8);
        DEBUG_PRINTLN("");
        break; /* Exit the collision loop */
      }
    } /* End: for (cnum = 0; cnum < 32; cnum++) */

    /* Check if the BCC matches ... */
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("C. Checking BCC for data integrity.");
    uint8_t bcc_val = uid_this_level[4];
    uint8_t bcc_calc = uid_this_level[0] ^ uid_this_level[1] ^
                       uid_this_level[2] ^ uid_this_level[3];
    if (bcc_val != bcc_calc) {
      DEBUG_PRINTLN("ERROR: BCC mistmatch!\n");
      return 0;
    }

    /* Clear the interrupts. */
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("D. Clearing and configuring interrupts.");
    write8(MFRC630_REG_IRQ0, 0b01111111);
    write8(MFRC630_REG_IRQ1, 0b00111111);

    send_req[0] = cmd;
    send_req[1] = 0x70;
    send_req[6] = bcc_calc;
    message_length = 7;

    /* Re-enable CRCs. */
    write8(MFRC630_REG_TX_CRC_PRESET, 0x18 | 1);
    write8(MFRC630_REG_RX_CRC_CON, 0x18 | 1);

    /* Reset the TX and RX registers (disable alignment, transmit full uint8_ts) */
    write8(MFRC630_REG_TX_DATA_NUM, (kbits % 8) | (1 << 3));
    uint8_t rxalign = 0;
    write8(MFRC630_REG_RX_BIT_CTRL, (0 << 7) | (rxalign << 4));

    /* Send the command. */
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("E. Sending collision command");
    writeCommand(MFRC630_CMD_TRANSCEIVE, message_length, send_req);

    /* Wait until the command execution is complete. */
    uint8_t irq1_value = 0;
    while (!(irq1_value & MFRC630IRQ1_TIMER0IRQ)) {
      irq1_value = read8(MFRC630_REG_IRQ1);
      /* Check for a global interrrupt, which can only be ERR or RX. */
      if (irq1_value & MFRC630IRQ1_GLOBALIRQ) {
        break;
      }
    }
    writeCommand(MFRC630_CMD_IDLE);

    /* Check the source of exiting the loop. */
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("F. Command complete, verifying proper exit.");
    uint8_t irq0_value = read8(MFRC630_REG_IRQ0);
    /* Check the ERROR IRQ */
    if (irq0_value & MFRC630IRQ0_ERRIRQ) {
      /* Check what kind of error. */
      uint8_t error = read8(MFRC630_REG_ERROR);
      if (error & MFRC630_ERROR_COLLDET) {
        /* Collision detecttion. */
        printError(MFRC630_ERROR_COLLDET);
        return 0;
      }
    }

    /* Read SAK answer from fifo. */
    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("G. Checking SAK in response payload.");
    uint8_t sak_len = readFIFOLen();
    if (sak_len != 1) {
      DEBUG_TIMESTAMP();
      DEBUG_PRINTLN("ERROR: NO SAK in response!\n");
      return 0;
    }
    uint8_t sak_value;
    readFIFO(sak_len, &sak_value);

    DEBUG_TIMESTAMP();
    DEBUG_PRINT("SAK answer: ");
    DEBUG_PRINTLN(sak_value);

    /* Check if there is more data to read. */
    if (sak_value & (1 << 2)) {
      /* UID not yet complete, continue to next cascade. */
      DEBUG_TIMESTAMP();
      DEBUG_PRINTLN("UID not complete ... looping to next cascade level.");
      uint8_t UIDn;
      for (UIDn = 0; UIDn < 3; UIDn++) {
        // uid_this_level[UIDn] = uid_this_level[UIDn + 1];
        uid[(cascadelvl - 1) * 3 + UIDn] = uid_this_level[UIDn + 1];
      }
    } else {
      DEBUG_TIMESTAMP();
      DEBUG_PRINTLN("DONE! UID fully parsed, exiting.");
      /* Done! */
      /* Add current uint8_ts at this level to the UID. */
      uint8_t UIDn;
      for (UIDn = 0; UIDn < 4; UIDn++) {
        uid[(cascadelvl - 1) * 3 + UIDn] = uid_this_level[UIDn];
      }

      /* Finally, return the length of the UID that's now at the uid pointer. */
      return cascadelvl * 3 + 1;
    }

    DEBUG_TIMESTAMP();
    DEBUG_PRINTLN("Exiting cascade loop");

  } /* End: for (cascadelvl = 1; cascadelvl <= 3; cascadelvl++) */

  /* Return 0 for UUID length if nothing was found. */
  return 0;
}

void Adafruit_MFRC630::mifareLoadKey(uint8_t *key) {
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Loading Mifare key into crypto unit.");

  writeCommand(MFRC630_CMD_IDLE);
  clearFIFO();
  writeFIFO(6, key);
  writeCommand(MFRC630_CMD_LOADKEY);
}

bool Adafruit_MFRC630::mifareAuth(uint8_t key_type, uint8_t blocknum,
                                  uint8_t *uid) {
  DEBUG_TIMESTAMP();
  DEBUG_PRINT("Authenticating Mifare block ");
  DEBUG_PRINTLN(blocknum);

  writeCommand(MFRC630_CMD_IDLE);
  clearFIFO();

  /* Allow the IDLE and Error IRQs to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQOEN, MFRC630IRQ0_IDLEIRQ | MFRC630IRQ0_ERRIRQ);
  /* Allow Timer0 IRQ to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ1EN, MFRC630IRQ1_TIMER0IRQ);

  /* Configure the frame wait timeout using T0 (10ms max). */
  /* 1 'tick' 4.72us, so 2000 = ~10ms */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Configuring Timer0 @ 211.875kHz, post TX, 10ms timeout.");
  write8(MFRC630_REG_T0_CONTROL, 0b10001);
  write8(MFRC630_REG_T0_RELOAD_HI, 2000 >> 8);
  write8(MFRC630_REG_TO_RELOAD_LO, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_HI, 2000 >> 8);
  write8(MFRC630_REG_T0_COUNTER_VAL_LO, 0xFF);

  /* Clear interrupts. */
  write8(MFRC630_REG_IRQ0, 0b01111111);
  write8(MFRC630_REG_IRQ1, 0b00111111);

  /* Start of AUTH procedure. */
  writeCommand(MFRC630_CMD_IDLE);
  clearFIFO();

  /*
   * MFAUTHENT command has the following parameters:
   * [0]    Key type (0x60 = KEYA, 0x61 = KEYB)
   * [1]    Block address
   * [2]    UID uint8_t 0
   * [3]    UID uint8_t 1
   * [4]    UID uint8_t 2
   * [5]    UID uint8_t 3
   *
   * NOTE: When the MFAuthent command is active, any FIFO access is blocked!
   */
  uint8_t params[6] = {key_type, blocknum, uid[0], uid[1], uid[2], uid[3]};
  writeFIFO(6, params);
  writeCommand(MFRC630_CMD_MFAUTHENT);

  /*
   * This command terminates automatically when the MIFARE Classic card is
   * authenticated and the bit MFCrypto1On is set to logic 1.
   *
   * This command does not terminate automatically when the card does not
   * answer, therefore the timer should be initialized to automatic mode. In
   * this case, beside the bit IdleIRQ the bit TimerIRQ can be used as
   * termination criteria. During authentication processing the bits RxIRQ
   * and TxIRQ are blocked. The Crypto1On shows if the authentication was
   * successful. The Crypto1On is always valid.
   *
   * In case there is an error during authentication, the bit ProtocolErr in
   * the Error register is set to logic 1 and the bit Crypto1On in register
   * Status2Reg is set to logic 0.
   */

  /* Wait until the command execution is complete. */
  uint8_t irq1_value = 0;
  while (!(irq1_value & MFRC630IRQ1_TIMER0IRQ)) {
    irq1_value = read8(MFRC630_REG_IRQ1);
    /* Check for a global interrrupt, which can only be ERR or RX. */
    if (irq1_value & MFRC630IRQ1_GLOBALIRQ) {
      break;
    }
  }

#if 0
  uint8_t irq0_value = read8(MFRC630_REG_IRQ0);
  uint8_t error = read8(MFRC630_REG_ERROR);
  uint8_t status = read8(MFRC630_REG_STATUS);
  PRINT("ERROR : 0x%02x", error); 
  PRINT("IRQ0  : 0x%02x", irq0_value);
  PRINT("IRQ1  : 0x%02x", irq1_value);
  PRINT("STATUS: 0x%02x", status);
#endif

  /* Check the error flag (MFRC630_ERROR_PROT, etc.) */
  uint8_t error = read8(MFRC630_REG_ERROR);
  if (error) {
    printError((enum mfrc630errors)error);
    return false;
  }

  /* Check if we timed out or got a response. */
  if (irq1_value & MFRC630IRQ1_TIMER0IRQ) {
    /* Timed out, no auth! :( */
    return false;
  }

  /* Check the status register for CRYPTO1 flag (Mifare AUTH). */
  uint8_t status = read8(MFRC630_REG_STATUS);
  return (status & MFRC630STATUS_CRYPTO1ON) ? true : false;
}

uint16_t Adafruit_MFRC630::mifareReadBlock(uint8_t blocknum, uint8_t *buf) {
  clearFIFO();

  /* Enable CRC. */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("A. Disabling CRC checks.");
  write8(MFRC630_REG_TX_CRC_PRESET, 0x18 | 1);
  write8(MFRC630_REG_RX_CRC_CON, 0x18 | 1);

  /* Allow the IDLE and Error IRQs to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQOEN, MFRC630IRQ0_IDLEIRQ | MFRC630IRQ0_ERRIRQ);
  /* Allow Timer0 IRQ to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ1EN, MFRC630IRQ1_TIMER0IRQ);

  /* Configure the frame wait timeout using T0 (10ms max). */
  /* 1 'tick' 4.72us, so 2000 = ~10ms */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Configuring Timer0 @ 211.875kHz, post TX, 10ms timeout.");
  write8(MFRC630_REG_T0_CONTROL, 0b10001); /* Start at end of TX, 211kHz */
  write8(MFRC630_REG_T0_RELOAD_HI, 0xFF);
  write8(MFRC630_REG_TO_RELOAD_LO, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_HI, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_LO, 0xFF);

  /* Clear interrupts. */
  write8(MFRC630_REG_IRQ0, 0b01111111);
  write8(MFRC630_REG_IRQ1, 0b00111111);

  /* Transceive the command. */
  uint8_t req[2] = {MIFARE_CMD_READ, blocknum};
  writeCommand(MFRC630_CMD_TRANSCEIVE, 2, req);

  /* Wait until the command execution is complete. */
  uint8_t irq1_value = 0;
  while (!(irq1_value & MFRC630IRQ1_TIMER0IRQ)) {
    irq1_value = read8(MFRC630_REG_IRQ1);
    /* Check for a global interrrupt, which can only be ERR or RX. */
    if (irq1_value & MFRC630IRQ1_GLOBALIRQ) {
      break;
    }
  }
  writeCommand(MFRC630_CMD_IDLE);

  /* Check if we timed out or got a response. */
  if (irq1_value & MFRC630IRQ1_TIMER0IRQ) {
    /* Timed out, no auth :( */
    DEBUG_PRINTLN("TIMED OUT!");
    return 0;
  }

  /* Read the size and contents of the FIFO, and return the results. */
  uint16_t buffer_length = readFIFOLen();
  uint16_t rx_len = (buffer_length <= 16) ? buffer_length : 16;
  readFIFO(rx_len, buf);

  return rx_len;
}

uint16_t Adafruit_MFRC630::ntagReadPage(uint16_t pagenum, uint8_t *buf) {
  clearFIFO();

  /* Enable CRC. */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("A. Disabling CRC checks.");
  write8(MFRC630_REG_TX_CRC_PRESET, 0x18 | 1);
  write8(MFRC630_REG_RX_CRC_CON, 0x18 | 1);

  /* Allow the IDLE and Error IRQs to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQOEN, MFRC630IRQ0_IDLEIRQ | MFRC630IRQ0_ERRIRQ);
  /* Allow Timer0 IRQ to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ1EN, MFRC630IRQ1_TIMER0IRQ);

  /* Configure the frame wait timeout using T0 (10ms max). */
  /* 1 'tick' 4.72us, so 2000 = ~10ms */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Configuring Timer0 @ 211.875kHz, post TX, 10ms timeout.");
  write8(MFRC630_REG_T0_CONTROL, 0b10001); /* Start at end of TX, 211kHz */
  write8(MFRC630_REG_T0_RELOAD_HI, 0xFF);
  write8(MFRC630_REG_TO_RELOAD_LO, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_HI, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_LO, 0xFF);

  /* Clear interrupts. */
  write8(MFRC630_REG_IRQ0, 0b01111111);
  write8(MFRC630_REG_IRQ1, 0b00111111);

  /* Transceive the command. */
  uint8_t req[2] = {(uint8_t)NTAG_CMD_READ, (uint8_t)pagenum};
  writeCommand(MFRC630_CMD_TRANSCEIVE, 2, req);

  /* Wait until the command execution is complete. */
  uint8_t irq1_value = 0;
  while (!(irq1_value & MFRC630IRQ1_TIMER0IRQ)) {
    irq1_value = read8(MFRC630_REG_IRQ1);
    /* Check for a global interrrupt, which can only be ERR or RX. */
    if (irq1_value & MFRC630IRQ1_GLOBALIRQ) {
      break;
    }
  }
  writeCommand(MFRC630_CMD_IDLE);

  /* Check if we timed out or got a response. */
  if (irq1_value & MFRC630IRQ1_TIMER0IRQ) {
    /* Timed out, no auth :( */
    DEBUG_PRINTLN("TIMED OUT!");
    return 0;
  }

  /* Read the size and contents of the FIFO, and return the results. */
  uint16_t buffer_length = readFIFOLen();
  uint16_t rx_len = (buffer_length <= 4) ? buffer_length : 4;
  readFIFO(rx_len, buf);

  return rx_len;
}

uint16_t Adafruit_MFRC630::mifareWriteBlock(uint16_t blocknum, uint8_t *buf) {
  clearFIFO();

  DEBUG_TIMESTAMP();
  DEBUG_PRINT("Writing data to card @ 0x");
  DEBUG_PRINTLN(blocknum);

  /* Enable CRC for TX (RX off!). */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("A. Disabling CRC checks.");
  write8(MFRC630_REG_TX_CRC_PRESET, 0x18 | 1);
  write8(MFRC630_REG_RX_CRC_CON, 0x18 | 0);

  /* Allow the IDLE and Error IRQs to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQOEN, MFRC630IRQ0_IDLEIRQ | MFRC630IRQ0_ERRIRQ);
  /* Allow Timer0 IRQ to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ1EN, MFRC630IRQ1_TIMER0IRQ);

  /* Configure the frame wait timeout using T0 (10ms max). */
  /* 1 'tick' 4.72us, so 2000 = ~10ms */
  DEBUG_TIMESTAMP();
  DEBUG_PRINTLN("Configuring Timer0 @ 211.875kHz, post TX, 10ms timeout.");
  write8(MFRC630_REG_T0_CONTROL, 0b10001); /* Start at end of TX, 211kHz */
  write8(MFRC630_REG_T0_RELOAD_HI, 0xFF);
  write8(MFRC630_REG_TO_RELOAD_LO, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_HI, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_LO, 0xFF);

  /* Clear interrupts. */
  write8(MFRC630_REG_IRQ0, 0b01111111);
  write8(MFRC630_REG_IRQ1, 0b00111111);

  /* Transceive the WRITE command. */
  uint8_t req1[2] = {(uint8_t)MIFARE_CMD_WRITE, (uint8_t)blocknum};
  writeCommand(MFRC630_CMD_TRANSCEIVE, sizeof(req1), req1);

  /* Wait until the command execution is complete. */
  uint8_t irq1_value = 0;
  while (!(irq1_value & MFRC630IRQ1_TIMER0IRQ)) {
    irq1_value = read8(MFRC630_REG_IRQ1);
    /* Check for a global interrrupt, which can only be ERR or RX. */
    if (irq1_value & MFRC630IRQ1_GLOBALIRQ) {
      break;
    }
  }
  writeCommand(MFRC630_CMD_IDLE);

  /* Check if we timed out or got a response. */
  if (irq1_value & MFRC630IRQ1_TIMER0IRQ) {
    /* Timed out, no auth :( */
    DEBUG_PRINTLN("TIMED OUT!");
    return 0;
  }

  /* Check if an error occured */
  uint8_t error = read8(MFRC630_REG_ERROR);
  uint8_t irq0_value = read8(MFRC630_REG_IRQ0);
  if (irq0_value & MFRC630IRQ0_ERRIRQ) {
    printError((enum mfrc630errors)error);
    return 0;
  }

  /* We should have a single ACK uint8_t in buffer at this point. */
  uint16_t buffer_length = readFIFOLen();
  if (buffer_length != 1) {
    DEBUG_TIMESTAMP();
    DEBUG_PRINT("Unexpected response buffer len: ");
    DEBUG_PRINTLN(buffer_length);
    return 0;
  }

  uint8_t ack = 0;
  readFIFO(1, &ack);
  if (ack != 0x0A) {
    /* Missing valid ACK response! */
    DEBUG_TIMESTAMP();
    DEBUG_PRINT("Invalid ACK response: ");
    DEBUG_PRINTLN(ack, HEX);
    return 0;
  }

  /* TODO: Verift values! */
  /* Clear the interrupts. */
  write8(MFRC630_REG_IRQ0, 0b01111111);
  write8(MFRC630_REG_IRQ1, 0b00111111);

  /* Transfer the page data. */
  writeCommand(MFRC630_CMD_TRANSCEIVE, 16, buf);

  /* Wait until the command execution is complete. */
  irq1_value = 0;
  while (!(irq1_value & MFRC630IRQ1_TIMER0IRQ)) {
    irq1_value = read8(MFRC630_REG_IRQ1);
    /* Check for a global interrrupt, which can only be ERR or RX. */
    if (irq1_value & MFRC630IRQ1_GLOBALIRQ) {
      break;
    }
  }
  writeCommand(MFRC630_CMD_IDLE);

  /* Check if we timed out or got a response. */
  if (irq1_value & MFRC630IRQ1_TIMER0IRQ) {
    /* Timed out, no auth :( */
    DEBUG_PRINTLN("TIMED OUT!");
    return 0;
  }

  /* Check if an error occured */
  error = read8(MFRC630_REG_ERROR);
  irq0_value = read8(MFRC630_REG_IRQ0);
  if (irq0_value & MFRC630IRQ0_ERRIRQ) {
    printError((enum mfrc630errors)error);
    return 0;
  }

  /* We should have a single ACK uint8_t in buffer at this point. */
  buffer_length = readFIFOLen();
  if (buffer_length != 1) {
    DEBUG_TIMESTAMP();
    DEBUG_PRINT("Unexpected response buffer len: ");
    DEBUG_PRINTLN(buffer_length);
    return 0;
  }

  ack = 0;
  readFIFO(1, &ack);
  if (ack != 0x0A) {
    /* Missing valid ACK response! */
    DEBUG_TIMESTAMP();
    DEBUG_PRINT("Invalid ACK response: ");
    DEBUG_PRINTLN(ack, HEX);
    return 0;
  }

  return 16;
}

uint16_t Adafruit_MFRC630::ntagWritePage(uint16_t pagenum, uint8_t *buf) {
  /*
   * For now, protect pages 0..3 and 40..44, and restrict writes to the safe
   * 'user memory' range (see docs/NTAG.md for further details).
   */
  if ((pagenum < 4) || (pagenum > 44)) {
    DEBUG_TIMESTAMP();
    DEBUG_PRINT("Page number out of range for NTAG213: ");
    DEBUG_PRINTLN(pagenum);
    return 0;
  }

  /* Use the Mifare write, which is compatible with the NTAG cards. */
  return mifareWriteBlock(pagenum, buf) == 16 ? 4 : 0;
}
