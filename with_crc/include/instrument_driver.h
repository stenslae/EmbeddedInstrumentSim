/* instrument_driver.h
Author:  Nevin Leh, Emma Stensland
Date:    April 2020 */

/********************
Includes
*********************/
#include "instrument.h"

/********************
Constants
*********************/
// Sizes
const uint16_t K_MAX_PACKET_SIZE = 512;
const uint8_t K_MIN_PACKET_SIZE = 10;
const uint16_t K_MAX_CMD_SIZE = 246;
const uint8_t K_MIN_CMD_SIZE = 12;
const uint8_t K_MAX_CMDS = 10;
const uint8_t K_TIME_SIZE = 33;
const uint16_t K_MAX_TLM_SIZE = 8196;

// Offsets
const uint8_t K_INS_DATA_LEN_OFFSET = 6;
const uint8_t K_INS_TIME_LENGTH_OFFSET = 14;
const uint8_t K_INS_TIME_OFFSET = 18;
const uint8_t K_INS_HEADER_OFFSET = 10;

// Sync
const uint32_t SYNC = 0xFEFA30C8;

// CRC
const uint16_t CRC_SEED = 0xFFFF;
const uint16_t CRC_SEED_TABLE = 0x0000;
const uint16_t CRC_POLY = 0x1021;

/********************
Enums
*********************/
typedef enum E_REC_STATE {
   E_REC_IDLE = 0,
   E_REC_LENGTH = 1,
   E_REC_TIME_START = 2,
   E_REC_TIME = 3,
   E_REC_CMD_START = 4,
   E_REC_CMD = 5,
} REC_STATE;

typedef enum UPDATE_STATE {
   UPDATE_TIME = 0,
   UPDATE_SEQUENCE = 1,
   RESET_SEQUENCE = 2,
   TOGGLE_HEART = 3,
   TOGGLE_POWER = 4,
} UPDATE_STATE;

typedef enum ALARM_STATE {
   ITF_LENGTH = 0,
   ITF_CHECKSUM = 1,
   CCSDS_FORMAT = 2,
   CCSDS_APID = 3,
   CCSDS_LENGTH = 4,
} ALARM_STATE;

/********************
Functions
*********************/
void getData(void);
void buildCRC(void);
uint16_t crc(uint16_t checksum, uint8_t data);
void reset();
void instrumentUpdate(UPDATE_STATE updade_arg);
void processCommands(void);
void sendData(int pack_size);
void echo(uint16_t arg_count, uint16_t cmd_location_head, uint8_t command_result);
void status();
void alarm(ALARM_STATE alarm_type);