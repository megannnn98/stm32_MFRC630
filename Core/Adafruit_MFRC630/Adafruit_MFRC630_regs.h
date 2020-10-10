/*!
 * @file mfrc630_regs.h
 */
#ifndef __mfrc630_REGS_H__
#define __mfrc630_REGS_H__

/*!
 * @brief MFRC630 command set
 */
typedef enum  {
  mfrc630_REG_COMMAND = 0x00,
  mfrc630_REG_HOST_CTRL = 0x01,
  mfrc630_REG_FIFO_CONTROL = 0x02,
  mfrc630_REG_WATER_LEVEL = 0x03,
  mfrc630_REG_FIFO_LENGTH = 0x04,
  mfrc630_REG_FIFO_DATA = 0x05,
  mfrc630_REG_IRQ0 = 0x06,
  mfrc630_REG_IRQ1 = 0x07,
  mfrc630_REG_IRQOEN = 0x08,
  mfrc630_REG_IRQ1EN = 0x09,
  mfrc630_REG_ERROR = 0x0A,
  mfrc630_REG_STATUS = 0x0B,
  mfrc630_REG_RX_BIT_CTRL = 0x0C,
  mfrc630_REG_RX_COLL = 0x0D,
  mfrc630_REG_T_CONTROL = 0x0E,
  mfrc630_REG_T0_CONTROL = 0x0F,
  mfrc630_REG_T0_RELOAD_HI = 0x10,
  mfrc630_REG_TO_RELOAD_LO = 0x11,
  mfrc630_REG_T0_COUNTER_VAL_HI = 0x12,
  mfrc630_REG_T0_COUNTER_VAL_LO = 0x13,
  mfrc630_REG_T1_CONTROL = 0x14,
  mfrc630_REG_T1_RELOAD_HI = 0x15,
  mfrc630_REG_T1_RELOAD_LO = 0x16,
  mfrc630_REG_T1_COUNTER_VAL_HI = 0x17,
  mfrc630_REG_T1_COUNTER_VAL_LO = 0x18,
  mfrc630_REG_T2_CONTROL = 0x19,
  mfrc630_REG_T2_RELOAD_HI = 0x1A,
  mfrc630_REG_T2_RELOAD_LO = 0x1B,
  mfrc630_REG_T2_COUNTER_VAL_HI = 0x1C,
  mfrc630_REG_T2_COUNTER_VAL_LO = 0x1D,
  mfrc630_REG_T3_CONTROL = 0x1E,
  mfrc630_REG_T3_RELOAD_HI = 0x1F,
  mfrc630_REG_T3_RELOAD_LO = 0x20,
  mfrc630_REG_T3_COUNTER_VAL_HI = 0x21,
  mfrc630_REG_T3_COUNTER_VAL_LO = 0x22,
  mfrc630_REG_T4_CONTROL = 0x23,
  mfrc630_REG_T4_RELOAD_HI = 0x24,
  mfrc630_REG_T4_RELOAD_LO = 0x25,
  mfrc630_REG_T4_COUNTER_VAL_HI = 0x26,
  mfrc630_REG_T4_COUNTER_VAL_LO = 0x27,

  /* 0x28..0x39 = Antenna Configuration */
  mfrc630_REG_DRV_MOD = 0x28,             /**<  (ISO/IEC14443-A 106 = 0x8E) */
  mfrc630_REG_TX_AMP = 0x29,              /**<  (ISO/IEC14443-A 106 = 0x12) */
  mfrc630_REG_DRV_CON = 0x2A,             /**<  (ISO/IEC14443-A 106 = 0x39) */
  mfrc630_REG_TXL = 0x2B,                 /**<  (ISO/IEC14443-A 106 = 0x0A) */
  mfrc630_REG_TX_CRC_PRESET = 0x2C,       /**<  (ISO/IEC14443-A 106 = 0x18) */
  mfrc630_REG_RX_CRC_CON = 0x2D,          /**<  (ISO/IEC14443-A 106 = 0x18) */
  mfrc630_REG_TX_DATA_NUM = 0x2E,         /**<  (ISO/IEC14443-A 106 = 0x0F) */
  mfrc630_REG_TX_MOD_WIDTH = 0x2F,        /**<  (ISO/IEC14443-A 106 = 0x21) */
  mfrc630_REG_TX_SYM_10_BURST_LEN = 0x30, /**<  (ISO/IEC14443-A 106 = 0x00) */
  mfrc630_REG_TX_WAIT_CTRL = 0x31,        /**<  (ISO/IEC14443-A 106 = 0xC0) */
  mfrc630_REG_TX_WAIT_LO = 0x32,          /**<  (ISO/IEC14443-A 106 = 0x12) */
  mfrc630_REG_FRAME_CON = 0x33,           /**<  (ISO/IEC14443-A 106 = 0xCF) */
  mfrc630_REG_RX_SOFD = 0x34,             /**<  (ISO/IEC14443-A 106 = 0x00) */
  mfrc630_REG_RX_CTRL = 0x35,             /**<  (ISO/IEC14443-A 106 = 0x04) */
  mfrc630_REG_RX_WAIT = 0x36,             /**<  (ISO/IEC14443-A 106 = 0x90) */
  mfrc630_REG_RX_THRESHOLD = 0x37,        /**<  (ISO/IEC14443-A 106 = 0x5C) */
  mfrc630_REG_RCV = 0x38,                 /**<  (ISO/IEC14443-A 106 = 0x12) */
  mfrc630_REG_RX_ANA = 0x39,              /**<  (ISO/IEC14443-A 106 = 0x0A) */

  mfrc630_REG_RFU_LPCD = 0x3A,
  mfrc630_REG_SERIAL_SPEED = 0x3B,
  mfrc630_REG_LFO_TRIMM = 0x3C,
  mfrc630_REG_PLL_CTRL = 0x3D,
  mfrc630_REG_PLL_DIVOUT = 0x3E,
  mfrc630_REG_LPCD_QMIN = 0x3F,
  mfrc630_REG_LPCD_QMAX = 0x40,
  mfrc630_REG_LPCD_IMIN = 0x41,
  mfrc630_REG_LPCD_I_RESULT = 0x42,
  mfrc630_REG_LPCD_Q_RESULT = 0x43,
  mfrc630_REG_PADEN = 0x44,
  mfrc630_REG_PADOUT = 0x45,
  mfrc630_REG_PADIN = 0x46,
  mfrc630_REG_SIGOUT = 0x47,
  mfrc630_REG_VERSION = 0x7F
} mfrc630reg;

/*! See Table 7.10.2: Command Set */
typedef enum  {
  mfrc630_CMD_IDLE = 0x00, /**< Cancels current command */
  mfrc630_CMD_LPCD = 0x01, /**< Low power card detection */
  mfrc630_CMD_LOADKEY =
      0x02, /**< Reads a 6 uint8_t MIFARE key and puts it into KEY BUFFER */
  mfrc630_CMD_MFAUTHENT = 0x03, /**< Performs Mifare Classic authentication */
  mfrc630_CMD_RECEIVE = 0x05,   /**< Activates the receive circuit */
  mfrc630_CMD_TRANSMIT = 0x06,  /**< Transmits data from the FIFO buffer */
  mfrc630_CMD_TRANSCEIVE =
      0x07, /**< Transmits data from the FIFO buffer and automatically activates
               the receive buffer when finished */
  mfrc630_CMD_WRITEE2 = 0x08, /**< Gets 1 uint8_t from FIFO and writes to EEPROM */
  mfrc630_CMD_WRITEE2PAGE =
      0x09, /**< Gets up to 64 bytes from FIFO and writes to EEPROM */
  mfrc630_CMD_READE2 =
      0x0A, /**< Reads data from EEPROM and copies it into the FIFO buffer */
  mfrc630_CMD_LOADREG =
      0x0C, /**< Reads data from the internal EEPROM and initializes the MFRC630
               registers. EEPROM address needs to be within EEPROM sector 2 */
  mfrc630_CMD_LOADPROTOCOL =
      0x0D, /**< Reads data from the internal EEPROM and initializes the MFRC630
               registers needed for a protocol change. */
  mfrc630_CMD_LOADKEYE2 =
      0x0E, /**< Copies a key from EEPROM into the key buffer */
  mfrc630_CMD_STOREKEYE2 =
      0x0F,                   /**< Stores a MIFARE key (6 bytes) into EEPROM */
  mfrc630_CMD_READRNR = 0x1C, /**< Copies bytes from the random number generator
                                 into the FIFO buffer until the FIFO is full */
  mfrc630_CMD_SOFTRESET = 0x1F /**< SW resets the MFRC630 */
} mfrc630cmd;

/*! ISO14443 Commands (see ISO-14443-3) */
typedef enum  {
  ISO14443_CMD_REQA = 0x26,    /**< Request command. */
  ISO14443_CMD_WUPA = 0x52,    /**< Wakeup command. */
  ISO14443_CAS_LEVEL_1 = 0x93, /**< Anticollision cascade level 1. */
  ISO14443_CAS_LEVEL_2 = 0x95, /**< Anticollision cascade level 2. */
  ISO14443_CAS_LEVEL_3 = 0x97  /**< Anticollision cascade level 3. */
} iso14443_cmd;

/*! Mifare Commands */
typedef enum  {
  MIFARE_CMD_AUTH_A = 0x60,
  MIFARE_CMD_AUTH_B = 0x61,
  MIFARE_CMD_READ = 0x30,
  MIFARE_CMD_WRITE = 0xA0,
  MIFARE_CMD_TRANSFER = 0xB0,
  MIFARE_CMD_DECREMENT = 0xC0,
  MIFARE_CMD_INCREMENT = 0xC1,
  MIFARE_CMD_STORE = 0xC2,
  MIFARE_ULTRALIGHT_CMD_WRITE = 0xA2
} mifare_cmd;

/*! NTAG Commands */
typedef enum  {
  NTAG_CMD_READ = 0x30,      /**> NTAG page read. */
  NTAG_CMD_WRITE = 0xA2,     /**< NTAG-specfiic 4 uint8_t write. */
  NTAG_CMD_COMP_WRITE = 0xA0 /**< Mifare Classic 16-uint8_t compat. write. */
} ntag_cmd;

/*! 'ComState' values for for the mfrc630_REG_STATUS register (0x0B) */
typedef enum  {
  mfrc630_COMSTAT_IDLE = 0,         /**< IDLE */
  mfrc630_COMSTAT_TXWAIT = 1,       /**< TX Wait */
  mfrc630_COMSTAT_TRANSMITTING = 3, /**< Transmitting */
  mfrc630_COMSTAT_RXWAIT = 5,       /**< RX Wait */
  mfrc630_COMSTAT_WAITFORDATA = 6,  /**< Waiting for DATA */
  mfrc630_COMSTAT_RECEIVING = 7     /**< Receiving */
} mfrc630comstat;

/*! Radio config modes */
typedef enum  {
  mfrc630_RADIOCFG_ISO1443A_106 = 1, /**< ISO1443A 106 Mode */
  mfrc630_LAST
} mfrc630radiocfg;

/*! MFRC360 errors */
typedef enum  {
  mfrc630_ERROR_EEPROM = (1 << 7),   /**< EEPROM error. */
  mfrc630_ERROR_FIFOWR = (1 << 6),   /**< FIFO write error. */
  mfrc630_ERROR_FIFOOVL = (1 << 5),  /**< FIFO already full! */
  mfrc630_ERROR_MINFRAME = (1 << 4), /**< Not enough data in frame. */
  mfrc630_ERROR_NODATA = (1 << 3),   /**< FIFO empty! */
  mfrc630_ERROR_COLLDET = (1 << 2),  /**< Collision detection, see RxColl. */
  mfrc630_ERROR_PROT = (1 << 1),     /**< Protocol error. */
  mfrc630_ERROR_INTEG = (1 << 0)     /**< Data integrity error. */
} mfrc630errors;

/*! MFRC630 interrupt requests 0 */
typedef enum  {
  MFRC630IRQ0_SET = (1 << 7),        /**< Sets/Clears interrupt. */
  MFRC630IRQ0_HIALERTIRQ = (1 << 6), /**< FIFO has reached top level. */
  MFRC630IRQ0_LOALERTIRQ = (1 << 5), /**< FIFO has reached bottom level. */
  MFRC630IRQ0_IDLEIRQ = (1 << 4),    /**< Command terminated by itself. */
  MFRC630IRQ0_TXIRQ = (1 << 3),      /**< Data transmission complete */
  MFRC630IRQ0_RXIRQ = (1 << 2),      /**< Receiver detected end of stream */
  MFRC630IRQ0_ERRIRQ =
      (1 << 1), /**< FifoWrErr, FiFoOvl, ProtErr, NoDataErr, IntegErr. */
  MFRC630IRQ0_RXSOF = (1 << 0) /**< RX start of frame detected. */
} mfrc630irq0;

/*! MFRC630 interrupt requests 1 */
typedef enum  {
  MFRC630IRQ1_SET = (1 << 7),       /**< Sets/Clears interrupt. */
  MFRC630IRQ1_GLOBALIRQ = (1 << 6), /**< Set if an enabled IRQ occured */
  MFRC630IRQ1_LPCDIRQ = (1 << 5),   /**< Card detected in low power mode */
  MFRC630IRQ1_TIMER4IRQ = (1 << 4), /**< Timer 4 underflow */
  MFRC630IRQ1_TIMER3IRQ = (1 << 3), /**< Timer 3 underflow */
  MFRC630IRQ1_TIMER2IRQ = (1 << 2), /**< Timer 2 underflow */
  MFRC630IRQ1_TIMER1IRQ = (1 << 1), /**< Timer 1 underflow */
  MFRC630IRQ1_TIMER0IRQ = (1 << 0), /**< Timer 0 underflow */
} mfrc630irq1;

/*! MFRC630 crypto engine status */
typedef enum  {
  MFRC630STATUS_CRYPTO1ON = (1 << 5) /**< Mifare Classic Crypto engine on */
} mfrc630status;

#endif
