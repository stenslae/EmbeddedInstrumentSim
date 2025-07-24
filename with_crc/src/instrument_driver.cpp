/* instrument_driver.cpp
Author: Tyler Holliday, Connor Parrott, Andrew Johnson, Nevin Leh, Emma Stensland
Date:   23 December 2024
-----------
Description
-----------
Functions for operating instrument simulator on a Teensy 4.1 
NOTES: intended to send 1pps status and echo any commands sent, all in individual frames */

/********************
Includes
*********************/
#include "instrument_driver.h"

/********************
Global Variables
*********************/
// Instrument Values
uint8_t i_heartbeat = 0x80;
uint16_t i_sequence_count = 0;
uint8_t i_power = 0x00;
uint32_t i_time = 0;
uint16_t i_status_send = 1;

// CRC-CCITT16 Lookup
uint16_t CRC_LOOKUP[256];

// Flags
uint8_t flag_time_recieved = 0;                    // Successful time packet recieved
uint8_t flag_packet_error = 0;                     // Error during recieving CCSDS
uint8_t flag_sync_found = 0;                       // ITF frame found
uint8_t flag_end_reached = 0;                      // ITF frame done

// Counters
REC_STATE state = E_REC_IDLE;                      // FSM for getData()
REC_STATE next_state = E_REC_IDLE;                 // FSM for getData()
uint16_t g_read_count = 0;                         // Total reads of ITF
uint8_t g_idle_count = 0;                          // Bytes idled in CMD_START
uint8_t g_command_num = 0;                         // Command packets recieved in ITF * 2
uint16_t g_cmd_read_count = 0;                     // Reads of command CCSDS
uint16_t g_cmd_read_total = 0;                     // Reads since first command header
uint16_t status_send_counter = 0;                  // 1pps packets read after status() called

// Reads
uint8_t new_byte = 0x00;                           // Most recent byte read
uint32_t g_four_bytes = 0x00000000;                // Last 4 bytes read
uint16_t g_two_bytes = 0x0000;                     // Last 2 bytes read
uint16_t g_data_len = 0;                           // ITF frame length
uint16_t crc_total = CRC_SEED;                     // CRC of ITF
uint32_t g_time_next = 0;                          // The time of the next 1pps
uint16_t g_cmd_length = 0;                         // Length of command
uint8_t cmd_packets[K_MAX_CMD_SIZE * K_MAX_CMDS];  // All data from commands
uint16_t cmd_location_info[K_MAX_CMDS * 2];        // Start and Arg Number of each command

// Output
uint8_t tlm_packet[K_MAX_TLM_SIZE];

/**********************************************************************************************************************
* Function      : void getData(void)
* Description   : Reads spacecraft data frame and loads commands for instrument
* Arguments     : none
* Returns       : none
* Remarks       : State machine
*                 E_REC_IDLE - wait for sync
*                 E_REC_LENGTH - reads length of frame 
*                 E_REC_TIME_START - waits for timestamp packet
*                 E_REC_TIME - saves current time
*                 E_REC_CMD_START - waits for command packet start
*                 E_REC_CMD - saves command packet and runs command
*                 E_REC_RESET - clears data saved from frame
**********************************************************************************************************************/
void getData(void) {
    // Reset frame values
    reset();

    while (Serial2.available() > 0) {  
        // Read a byte from UART
        new_byte = Serial2.read();

        // Load most recent 4 bytes read
        g_four_bytes = (g_four_bytes << 8) & 0xFFFFFF00;
        g_four_bytes = g_four_bytes | new_byte;

        // Mask for recent 2 bytes read
        g_two_bytes = g_four_bytes & 0xFFFF;

        // If frame is being read, calculate crc
        if(flag_sync_found == 1) {
            // Count reads since synced
            g_read_count++;

            // Add up crc of each byte in itf
            crc_total = crc(crc_total, new_byte);

            // Redundant overflow protection
            if(g_read_count > K_MAX_PACKET_SIZE) {
                // ITF bad length, send an alarm
                flag_time_recieved = 0;
                alarm(ITF_LENGTH);
                // Force a reset
                reset();
            }
        }

        // Flag if end of frame has been read
        if((g_read_count == g_data_len) && (g_read_count >= K_INS_DATA_LEN_OFFSET)) {
            flag_end_reached = 1;
        }

        // Add a delay to allow for almost half fill of serial buff
        if(Serial2.available() == 0 & flag_end_reached == 0){
            delay(0.25);
        }

        switch (state) {
        case E_REC_IDLE:
            // Look for start of ITF
            if (g_four_bytes == SYNC) {
                // Trigger frame read
                flag_sync_found = 1;
                next_state = E_REC_LENGTH;

                // Set read count
                g_read_count = 4;

                // Update instrument MET (also will send status pack depending on interval)
                instrumentUpdate(UPDATE_TIME);
            }
            break;

        case E_REC_LENGTH:
            // Data length done reading in, saves and run checks then transition states
            if (g_read_count == K_INS_DATA_LEN_OFFSET) {
                // Save data length
                g_data_len = g_two_bytes & ~0xE000; // Turn off first three bits so just length
                g_data_len += K_INS_DATA_LEN_OFFSET; // Get total frame length

                // First three bits of g_data_len, should be 000 for commands
                if(0 != (g_two_bytes & 0xE000)) {
                    // CCSDS bad format, go to reset state
                    alarm(CCSDS_FORMAT);
                    reset();
                }

                // Check if packet is too big or too small
                if (g_data_len < K_MAX_PACKET_SIZE && g_data_len > K_MIN_PACKET_SIZE) {
                    // Good, go to next state
                    next_state = E_REC_TIME_START;
                } else {
                    // ITF bad length, go to reset state
                    flag_time_recieved = 0;
                    alarm(ITF_LENGTH);
                    reset();
                }
            }
            break;

        case E_REC_TIME_START:
            // Look for next timestamp packet header
            if (g_read_count == K_INS_HEADER_OFFSET) {
                // Timestamp header found
                if(g_two_bytes == 0x1900) {
                    next_state = E_REC_TIME;
                }else {
                    // No timestamp recieved
                    flag_time_recieved = 0;
                    // Check if command header
                    if(g_two_bytes == 0x1B00) {
                        next_state = E_REC_CMD;
                        g_command_num ++;
                        g_cmd_length = 0;
                        g_cmd_read_count = 0;
                        g_idle_count = 0;
                    }else {
                        // If correct APID in the header
                        if((g_two_bytes & 0x7FF) == 0x100 || (g_two_bytes & 0x7FF) == 0x300) {
                            // CCSDS bad format
                            alarm(CCSDS_FORMAT);
                        }else {
                            // CCSDS bad APID
                            alarm(CCSDS_APID);
                        }
                        // Trash packet and keep looking for commands
                        next_state = E_REC_CMD_START;
                   }
                }
            }
            break;
     
        case E_REC_TIME:

            // Verify spacecraft time packet is correct size
            if((g_read_count == K_INS_TIME_LENGTH_OFFSET) && (g_two_bytes != K_TIME_SIZE)) {
                // CCSDS bad length, stop reading time packet
                alarm(CCSDS_LENGTH);
                next_state = E_REC_CMD_START;
            }

            // Save last four read bytes as the time
            if(g_read_count == K_INS_TIME_OFFSET) {
                flag_time_recieved = 1;
                g_time_next = g_four_bytes;
            }

            // Verify bytes are reserved
            if((g_read_count < K_INS_TIME_OFFSET + 30) && (g_read_count > K_INS_TIME_OFFSET + 2) && (g_two_bytes != 0x00)) {
                // CCSDS bad format flag
                flag_packet_error = 1;
            }

            // Done processing spacecraft time packet
            if(g_read_count == K_INS_TIME_OFFSET + 30) {
                if(flag_packet_error == 1) {
                    // CCSDS bad format, send alarm trash time packet
                    alarm(CCSDS_FORMAT);
                    flag_time_recieved = 0;
                    flag_packet_error = 0;
                }
                next_state = E_REC_CMD_START;
            }
            break;
           
        case E_REC_CMD_START:

            g_idle_count ++;
            // Look for command packet header
            if(g_two_bytes == 0x1B00) {
                // Start a new command read
                next_state = E_REC_CMD;
                // Save the index where the command starts
                cmd_location_info[g_command_num] = g_cmd_read_total;
                g_command_num ++;
                g_cmd_length = 0;
                g_cmd_read_count = 0;
                g_idle_count = 0;
            }
            
            // Idling in CMD_START for too long looking for header
            if(g_idle_count > 2) {
                // Exclude CRC
                if(g_read_count <= g_data_len-2){
                    // Bad packet
                    if((g_two_bytes & 0x7FF) == 0x300) {
                        // CCSDS bad format
                        alarm(CCSDS_FORMAT);
                    }else {
                        // CCSDS bad APID
                        alarm(CCSDS_APID);
                    }
                }

                // Restart idling to look for new packet
                g_idle_count = 0;
            }
            break;

        case E_REC_CMD:
            // Every byte since command packet header found
            g_cmd_read_total ++;
            g_cmd_read_count ++; // Doesn't roll over

            // Save command packet length
            if(g_cmd_read_count == K_INS_DATA_LEN_OFFSET) {
                // Save command Length
                g_cmd_length = g_two_bytes + K_INS_HEADER_OFFSET + 1;
                //Save number of arguments 
                cmd_location_info[g_command_num] = g_two_bytes - 3;
                // Verify command packet length
                if(g_cmd_length < K_MIN_CMD_SIZE || g_cmd_length > (K_MAX_CMD_SIZE + 10)) {
                    // CCSDS bad length, look for new command (or frame end will be hit)
                    alarm(CCSDS_LENGTH);
                    next_state = E_REC_CMD_START;
                    // Remove current save info for command
                    cmd_location_info[g_command_num] = 0x00;
                    cmd_location_info[g_command_num-1] = 0x00;
                    g_cmd_read_total -= K_INS_DATA_LEN_OFFSET;
                    g_cmd_read_count -= K_INS_DATA_LEN_OFFSET;
                    g_command_num --;
                }
            }
            // Save command
            if(g_cmd_read_count > K_INS_HEADER_OFFSET){
                // Redundant overflow protection
                if(g_cmd_read_total > (K_MAX_CMD_SIZE * K_MAX_CMDS)) {
                    // ITF bad length, send an alarm
                    flag_time_recieved = 0;
                    alarm(ITF_LENGTH);
                    // Force a reset
                    reset();
                }else {
                    // Remove padding if present
                    if((new_byte == 0x00) && (g_cmd_read_count == g_cmd_length+1)) {
                        cmd_location_info[g_command_num]--;
                        g_cmd_read_total--;
                    }else {
                        cmd_packets[g_cmd_read_total - K_INS_HEADER_OFFSET - 1] = new_byte;
                    }
                }
            }
            // Command read success
            if(g_cmd_read_count == (g_cmd_length + K_INS_DATA_LEN_OFFSET + 1)) {
                // Find new command (or frame end will be hit)
                next_state = E_REC_CMD_START;
                g_command_num ++;
            }
            break;
        }

        if(flag_end_reached == 1) {
            flag_end_reached = 0;

            // Conduct CRC
            if(crc_total == 0x0000) {
                // All commands have been loaded and verified, execute them
                processCommands();
            }else {
                // ITF bad checksum, send an alarm
                flag_time_recieved = 0;
                alarm(ITF_CHECKSUM);
            }

            // Reset all values changed from reading frame
            reset();
        }

        state = next_state;
    }
}

/**********************************************************************************************************************
* Function      : void buildCRC(uint_16_t)
* Description   : Calculates the crc for a given byte for lookup table
* Arguments     : none
* Returns       : none
**********************************************************************************************************************/
void buildCRC() {
    for(int i = 0; i<256; i++){
        // Initialize lookup table seed
        CRC_LOOKUP[i] = CRC_SEED_TABLE;

        // Append data into the top part of the checksum so it is the most significant
        CRC_LOOKUP[i] ^= ((i << 8) & 0xFFFF);

        // Go through all of the new 8 bits and divide the polynomial
        for (uint8_t j = 0; j < 8; j++) {
            // If uppermost bit on
            if (CRC_LOOKUP[i] & 0x8000) {
                // Shift 1 out, subtract polynomial from checksum
                CRC_LOOKUP[i] = (CRC_LOOKUP[i] << 1) ^ CRC_POLY;
            }else {
                // Shift 0 out, polynomial doesn't fit
                CRC_LOOKUP[i] = (CRC_LOOKUP[i] << 1);
            }
        }
    }
}

/**********************************************************************************************************************
* Function      : uint16_t crc(uint_16_t)
* Description   : Calculates the checksum for a given byte
* Arguments     : uint816_t checksum, uint8_t data
* Returns       : uint16_t
* Remarks       : none
**********************************************************************************************************************/
uint16_t crc(uint16_t checksum, uint8_t data) {
    // Shift new data into current checksum
    uint8_t lookup = ((checksum >> 8) ^ data) & 0xFF;

    // XOR the crc value with the shifted checksum
    checksum = (checksum << 8) ^ CRC_LOOKUP[lookup];

   // Return the remainder of dividing data with polynomial
   return checksum;
}

/**********************************************************************************************************************
* Function      : void reset()
* Description   : Sets all saved values in the frame to zero
* Arguments     : none
* Returns       : none
**********************************************************************************************************************/
void reset(void){
    state = E_REC_IDLE;     
    next_state = E_REC_IDLE;
    flag_sync_found = 0;
    g_read_count = 0;
    g_command_num = 0;
    g_cmd_read_count = 0;
    g_cmd_read_total = 0;
    g_data_len = 0;
    flag_end_reached = 0;
    crc_total = CRC_SEED;
    memset(cmd_packets, 0x00, K_MAX_CMD_SIZE * 10);
    memset(cmd_location_info, 0x0000, 20);
}

/**********************************************************************************************************************
* Function      : void instrumentUpdate()
* Description   : Updates instruments heartbeat, MET, and sequence count based on argument
* Arguments     : UPDATE_STATE update_arg
* Returns       : none
* Remarks       : none
**********************************************************************************************************************/
void instrumentUpdate(UPDATE_STATE update_arg) {
   switch(update_arg){
       // Update if time was recieved, otherwise increment by 1
       case UPDATE_TIME:
           if(flag_time_recieved == 1) {
               i_time = g_time_next;
               flag_time_recieved = 0;
           }else {
               i_time ++;
           }

           // At every time interval update, a status packet may be sent 
            status_send_counter ++;
            if(status_send_counter == i_status_send && i_status_send != 0){
                status();
                status_send_counter = 0;
            }
            break;

       // Increment sequence count
       case UPDATE_SEQUENCE:
            i_sequence_count ++;
            // Mask to 14 bits
            i_sequence_count = i_sequence_count & 0x3FFF;
            break;

       // Reset sequence count
       case RESET_SEQUENCE:
            i_sequence_count = 0;
            break;

       // Toggle instrument heartbeat
       case TOGGLE_HEART:
            i_heartbeat ^= 0x80;
            break;

       // Toggle instrument power
       case TOGGLE_POWER:
            i_power ^= 0x40;
            break;
   }
}

/**********************************************************************************************************************
* Function      : void executeCommand()
* Description   : Pretends to execute commands then echo back a sucessful execution
* Arguments     : none
* Returns       : none
**********************************************************************************************************************/
void processCommands(void) {
    for(int i=0; i < g_command_num-1; i+=2) {
        // Pretend command executed successfully
        // Execute command, OPCODE: cmd_packets[cmd_location_info[i]] MACRO: cmd_packets[cmd_location_info[i +1]], ARGS after
        uint8_t command_result = 0x00;

        // Echo command if one was saved
        echo(cmd_location_info[i+1], cmd_location_info[i], command_result);
    }
}

/**********************************************************************************************************************
* Function      : void sendData(unint8_t* tlm_packet, int pack_size)
* Description   : Sends the TLM packet out via the Teensy's Serial 2 port
* Arguments     : int pack_size - size of the packet to be sent
* Returns      : none
**********************************************************************************************************************/
void sendData(int pack_size) {
    Serial2.write(tlm_packet, pack_size);
    // Tiktok after every frame
    instrumentUpdate(TOGGLE_HEART);
}

/**********************************************************************************************************************
* Function      : void status()
* Description   : Builds a status packet on TLM frame after designated interval (i_status_send)
* Arguments     : none
* Returns      : none
**********************************************************************************************************************/
void status() {
    // Verifies old tlm has sent before replacing tlm packet
    Serial2.flush();
    
    // Increase sequence count
    instrumentUpdate(UPDATE_SEQUENCE);

    // Set pack_size (args 124 + header of 16)
    int pack_size = 140;

    // Initialize packet
    memset(tlm_packet, 0x00, K_MAX_TLM_SIZE);

    // Telemetry ITF Header
    // Sync
    tlm_packet[0] = 0xFE;
    tlm_packet[1] = 0xFA;
    tlm_packet[2] = 0x30;
    tlm_packet[3] = 0xC8;
    // Alive, Power Down, Spare, Length (Aliveness toggled in sim)
    int e_data_len = pack_size - K_INS_DATA_LEN_OFFSET;
    tlm_packet[4] = i_heartbeat | i_power | ((e_data_len  >> 8) & 0xFF);
    // Length
    tlm_packet[5] = e_data_len  & 0xFF;
    // Status Header
    // Version, Type, Secondary, APID
    int s_apid = 0x305;
    tlm_packet[6] = 0x08 | ((s_apid >> 8) & 0x07);
    // APID
    tlm_packet[7] = (s_apid & 0xFF);
    // Grouping, Sequence Count: 11XXXXXX
    tlm_packet[8] = 0xC0 | ((i_sequence_count >> 8) & 0xFF);
    // Sequence Count: XXXXXXXX
    tlm_packet[9] = (i_sequence_count) & 0xFF;
    // Length of packet after this byte:
    e_data_len = 123;
    tlm_packet[10] = (e_data_len >> 8) & 0xFF;
    tlm_packet[11] = e_data_len & 0xFF;
 
    // Time tag (4 bytes) when command was executed
    tlm_packet[12] = (i_time >> 24) & 0xFF;
    tlm_packet[13] = (i_time >> 16) & 0xFF;
    tlm_packet[14] = (i_time >> 8) & 0xFF;
    tlm_packet[15] = i_time & 0xFF;

    // FIXME: Arguments filled with dummy values
    // ANALOG: 16-47
    // DIGITAL: 48-102
    // SOFTWARE: 102-137

    // Checksum at end
    int temp_check = CRC_SEED;
    for (size_t i = 4; i < pack_size-2; i++) {
        temp_check = crc(temp_check, tlm_packet[i]);
    }
  
    tlm_packet[138] = (temp_check >> 8) & 0xFF;
    tlm_packet[139] = temp_check;

    // Send status packet
    sendData(pack_size);
}

/**********************************************************************************************************************
* Function     : uint16_t* buildEcho(int pack_size)
* Description  : Builds the simulated echo packet into TLM frame
* Arguments    : uint16_t arg_count, int cmd_location_head, uint8_t command_result
* Returns      : none
**********************************************************************************************************************/
void echo(uint16_t arg_count, uint16_t cmd_location_head, uint8_t command_result) {
    // Verifies old tlm has sent before replacing tlm packet
    Serial2.flush();
    
    // Increase sequence count
    instrumentUpdate(UPDATE_SEQUENCE);

    // Maxmimum aruments that can be sent
    if(arg_count>10){
        arg_count = 10;
    }

    // Set pack_size
    int pack_size = arg_count + 20;

    // Check if padding is needed
    if(pack_size % 2 == 1) {
        // Make the size even
        pack_size ++;
    }
    // Initialize packet
    memset(tlm_packet, 0x00, K_MAX_TLM_SIZE);

    // Telemetry ITF Header
    // Sync
    tlm_packet[0] = 0xFE;
    tlm_packet[1] = 0xFA;
    tlm_packet[2] = 0x30;
    tlm_packet[3] = 0xC8;
    // Alive, Power Down, Spare, Length (Aliveness toggled in sim)
    int e_data_len = pack_size - K_INS_DATA_LEN_OFFSET;
    tlm_packet[4] = i_heartbeat | i_power | ((e_data_len  >> 8) & 0xFF);
    // Length
    tlm_packet[5] = e_data_len  & 0xFF;

    // Echo Header
    // Version, Type, Secondary, APID
    int e_apid = 0x301;
    tlm_packet[6] =  0x08 | ((e_apid >> 8) & 0x07);;
    // APID
    tlm_packet[7] = (e_apid & 0xFF);
    // Grouping, Sequence Count: 11XXXXXX
    tlm_packet[8] = 0xC0 | ((i_sequence_count >> 8) & 0xFF);
    // Sequence Count: XXXXXXXX
    tlm_packet[9] = (i_sequence_count) & 0xFF;
    // Length of packet after this byte:
    e_data_len = pack_size - 12;
    tlm_packet[10] = (e_data_len >> 8) & 0xFF;
    tlm_packet[11] = e_data_len & 0xFF;
 
    // Time tag (4 bytes) when command was executed
    tlm_packet[12] = (i_time >> 24) & 0xFF;
    tlm_packet[13] = (i_time >> 16) & 0xFF;
    tlm_packet[14] = (i_time >> 8) & 0xFF;
    tlm_packet[15] = i_time & 0xFF;

    // Macro, Result
    tlm_packet[16] = ((cmd_packets[cmd_location_head+1] & 0x01) << 7) | (command_result & 0x7F);
    // Opcode
    tlm_packet[17] = cmd_packets[cmd_location_head];
    // Load Arguments
    for(int i = 1; i <= arg_count; i ++){
        tlm_packet[17 + i] = cmd_packets[cmd_location_head + 1 + i];
    }

    // Checksum at end
    int temp_check = CRC_SEED;
    for (size_t i = 4; i < pack_size-2; i++) {
        temp_check = crc(temp_check, tlm_packet[i]);
    }

    tlm_packet[18 + arg_count] = (temp_check >> 8) & 0xFF;
    tlm_packet[19 + arg_count] = temp_check;

    // Send echo packet
    sendData(pack_size);
}

/**********************************************************************************************************************
* Function      : void alarm()
* Description   : Builds alarm packet and sends into TLM frame
* Arguments     : ALARM_STATE alarm_type
* Returns       : none
**********************************************************************************************************************/
void alarm(ALARM_STATE alarm_type) {
    // Verifies old tlm has sent before replacing tlm packet
    Serial2.flush();
    
    // Increase sequence count
    instrumentUpdate(UPDATE_SEQUENCE);
    
    // Set pack_size
    int pack_size = 22;

    // Initialize packet
    memset(tlm_packet, 0x00, K_MAX_TLM_SIZE);

    // Telemetry ITF Header
    // Sync
    tlm_packet[0] = 0xFE;
    tlm_packet[1] = 0xFA;
    tlm_packet[2] = 0x30;
    tlm_packet[3] = 0xC8;
    // Alive, Power Down, Spare, Length (Aliveness toggled in sim)
    int a_data_len = pack_size - K_INS_DATA_LEN_OFFSET;
    tlm_packet[4] = i_heartbeat | i_power | ((a_data_len  >> 8) & 0xFF);
    // Length
    tlm_packet[5] = a_data_len  & 0xFF;

    // Alarm Header
    // Version, Type, Secondary, APID
    int a_apid = 0x302;
    tlm_packet[6] =  0x08 | ((a_apid >> 8) & 0x07);;
    // APID
    tlm_packet[7] = (a_apid & 0xFF);
    // Grouping, Sequence Count: 11XXXXXX
    tlm_packet[8] = 0xC0 | ((i_sequence_count >> 8) & 0xFF);
    // Sequence Count: XXXXXXXX
    tlm_packet[9] = (i_sequence_count) & 0xFF;
    // Length of packet after this byte - 1:
    a_data_len = 7;
    tlm_packet[10] = (a_data_len >> 8) & 0xFF;
    tlm_packet[11] = a_data_len & 0xFF;
 
    // Time tag (4 bytes) when command was executed
    tlm_packet[12] = (i_time >> 24) & 0xFF;
    tlm_packet[13] = (i_time >> 16) & 0xFF;
    tlm_packet[14] = (i_time >> 8) & 0xFF;
    tlm_packet[15] = i_time & 0xFF;
   
    // Alarm ID
    tlm_packet[16] = 0x01;
    // Type
    tlm_packet[17] = 0x01;
   
    switch(alarm_type){
        case ITF_LENGTH:
            // Value
            tlm_packet[18] = 0x01;
        break;

        case ITF_CHECKSUM:
            // Value
            tlm_packet[18] = 0x02;
        break;

        case CCSDS_FORMAT:
            // Value
            tlm_packet[18] = 0x03;
        break;

        case CCSDS_APID:
            // Value
            tlm_packet[18] = 0x04;
        break;

        case CCSDS_LENGTH:
            // Value
            tlm_packet[18] = 0x05;
        break;
   }

    // Auxillary
    tlm_packet[19] = 0x00;

    // Checksum at end
    int temp_check = CRC_SEED;
    for (size_t i = 4; i < pack_size-2; i++) {
        temp_check = crc(temp_check, tlm_packet[i]);
    }

    tlm_packet[20] = (temp_check >> 8) & 0xFF;
    tlm_packet[21] = temp_check;

    // Send alarm packet
    sendData(pack_size);
}
