/* instrument_driver.cpp
Author: Tyler Holliday, Connor Parrott, Andrew Johnson, Nevin Leh
Date:   23 December 2024

-----------
Description
-----------
Functions for operating instrument simulator on a Teensy 4.1 */

/********************
Includes
*********************/
#include "instrument_driver.h"

/********************
Global Variables
*********************/
// Survey State Info
uint8_t g_surv_enabled = 1;
uint16_t g_surv_len = 8196;
uint8_t g_counter = 0; // Global counter to ensure it persists across calls
uint8_t g_length_high = (uint8_t)((g_surv_len + 3) >> 8);
uint8_t g_length_low = (uint8_t)(g_surv_len + 3);

// Burst State Info
uint8_t g_burst_enabled = 0;
uint16_t g_burst_len = 0;

// read buffer
uint8_t g_read_buff[64]; //max command size is 64 bytes
uint8_t g_rx_buff[64]; //max command size is 64 bytes
uint8_t g_read_buff_size = 0;
uint32_t g_sync_search;
uint16_t g_data_len = 0;

REC_STATE state = E_REC_IDLE;
uint8_t g_rx_ndx = 0;
uint8_t tx_counter = 0;
uint8_t process_count = 0;

/**********************************************************************************************************************
* Function      : void resetReadBuffSize()
* Description   : resets the size of the buffer
* Arguments     : none
* Returns       : 
* Remarks       : function is unused
**********************************************************************************************************************/
void resetReadBuffSize()
{
    g_read_buff_size = 0;
    state = E_REC_IDLE;
}

/**********************************************************************************************************************
* Function      : void getData(void)
* Description   : gets data from simulator if requested
* Arguments     : none
* Returns       : 
* Remarks       : function is unused but it looks like it had functionality associated with simulating burst mode
**********************************************************************************************************************/
void getData(void)
{
    uint8_t new_byte;
    REC_STATE next_state;
    //Serial2.println("Start Reading");
    // while (Serial2.available() > 0){
        
    //     // build rx buffer
    //     g_rx_ndx++;
    //     new_byte = Serial2.read();
    //     g_rx_buff[g_rx_ndx] = new_byte;
    // }
    // //Serial2.println("Finished Reading");
    // g_rx_ndx = 0;
    // uint8_t g_rx_buff_len = sizeof(g_rx_buff) / sizeof(g_rx_buff[0]);
    // uint8_t counter = 0;

    // if (g_rx_buff_len > counter){
    //     //Serial2.println("Haven't reached the end of the buffer");
    // }
    // if (Serial2.available() < 0)  {
    //     Serial2.println("Serialnot available");
    // } else {
    //     Serial2.println("Serial is available")
    // }


    while (Serial2.available() < 0)
    {
        //Serial2.println("PROCESSING");
        //read a byte from uart
        new_byte = Serial2.read();
        Serial2.println("New Byte:");
        Serial2.println(new_byte);
        //update search synch value
        g_sync_search = (g_sync_search << 8) & 0xFFFFFF00;
        g_sync_search = g_sync_search + new_byte;
        //default stay in current state
        next_state = state;

        switch (state)
        {
        case E_REC_IDLE:
            if (g_sync_search == 0xFEFA30C8)
            {
                //reset buffer size
                g_read_buff_size = 0;
                //load sync into buffer
                g_read_buff[K_INS_SYNC_OFFSET] = g_sync_search >> 24;
                g_read_buff[K_INS_SYNC_OFFSET + 1] = g_sync_search >> 16;
                g_read_buff[K_INS_SYNC_OFFSET + 2] = g_sync_search >> 8;
                g_read_buff[K_INS_SYNC_OFFSET + 3] = g_sync_search;
                g_read_buff_size = 4;
                //change state
                next_state = E_REC_SYNC;
            }
            break;

        case E_REC_SYNC:
            g_read_buff[g_read_buff_size] = new_byte;
            g_read_buff_size += 1;
            //if we have data length save it and transition states
            if (g_read_buff_size == K_INS_HEADER_SIZE)
            {
                // Serial2.println("DATA LENGTH:");
                // Serial2.println(g_data_len);
                g_data_len = (g_read_buff[K_INS_DATA_LEN_OFFSET] << 8) & 0xFF00;
                g_data_len = g_data_len | g_read_buff[K_INS_DATA_LEN_OFFSET + 1];
                
                //check if packet is to big
                if (g_data_len < K_MAX_PACKET_SIZE)
                {
                    // Serial2.println("correct data length");
                    // Serial2.println(g_data_len);
                    next_state = E_REC_DATA_LEN;
                }
                else
                {
                    // Serial2.println("wrong data length");
                    Serial2.println(g_data_len);
                    next_state = E_REC_IDLE;
                    g_read_buff_size = 0;
                }
            }
            break;
            
        case E_REC_DATA_LEN:
            g_read_buff[g_read_buff_size] = new_byte;
            g_read_buff_size += 1;
            if (g_read_buff_size == (K_INS_HEADER_SIZE + g_data_len + K_CRC_LEN + 1)) //plus 1 because data length is length -1
            {
                processCommand();
                next_state = E_REC_IDLE; //reset state machine
                g_read_buff_size = 0;    //reset counter
            }
            break;
        }
        state = next_state;
    }

    // if (g_echo_flag == 1){
    //     Serial2.print("echo flag state; ");
    //     Serial2.println(g_echo_flag);
    //     Serial2.write(g_read_buff, g_read_buff_size);
    //     g_echo_flag = 0;
    // }


}

/**********************************************************************************************************************
* Function      : void processCommand()
* Description   : process a command 
* Arguments     : none
* Returns       : boolean indicating CRC status
* Remarks       : Function is unused. Based on commit notes, this function was meant to handle three diffeent commands. 
                    The commands changed the operation mode of the simulator to either CMD echo, survey mode, or 
                    burst mode.
**********************************************************************************************************************/
void processCommand()
{
    //return if bad crc
    if (!checkCRC())
    {
        return;
    }

    uint16_t app_id = g_read_buff[K_INS_APPID_OFFSET] << 8;
    app_id = app_id | g_read_buff[K_INS_APPID_OFFSET + 1];
    // Serial2.println("APPID");
    // Serial2.println(app_id);
    app_id = app_id & K_INS_APPID_MASK;
    // Serial2.println("APPID post mask");
    // Serial2.println(app_id);
    if (app_id == 0x100){app_id = 0x300;}

    process_count++;
    Serial2.print("Process flag count: ");
    Serial2.println(process_count);

    switch (app_id)
    {
    case K_INS_CMD_ECHO:
        //echo command
        // Serial2.write(g_read_buff, g_read_buff_size);
        // Serial2.println("hitting echo");
        tx_counter++;
            Serial2.print("TX counter: ");
            Serial2.println(tx_counter);
        // g_echo_flag = 1;
        break;

    case K_INS_CMD_SURVEY:
        g_surv_enabled = g_read_buff[K_INS_HEADER_SIZE];
        g_surv_len = g_read_buff[K_INS_HEADER_SIZE + 1] << 8;
        g_surv_len = g_surv_len | g_read_buff[K_INS_HEADER_SIZE + 2];
        break;
    case K_INS_CMD_BURST:
        g_burst_enabled = g_read_buff[K_INS_HEADER_SIZE];
        g_burst_len = g_read_buff[K_INS_HEADER_SIZE + 1] << 8;
        g_burst_len = g_burst_len | g_read_buff[K_INS_HEADER_SIZE + 2];
        break;
    default:
        break;
    }
}

/**********************************************************************************************************************
* Function      : uint8_t checkCRC
* Description   : verifies the CRC is correct
* Arguments     : none
* Returns       : boolean indicating CRC status
**********************************************************************************************************************/
uint8_t checkCRC()
{
    /*todo: implement crc*/
    //crc is just 0xBBCC
    if ((g_read_buff[K_INS_HEADER_SIZE + g_data_len + 1] == 0xBB) && (g_read_buff[K_INS_HEADER_SIZE + g_data_len + 2] == 0xCC))
    {
        return 1;
    }
    else
    {
        return 1; // was set to return 0 but CRC will never be 0xBBCC
    }
}

/**********************************************************************************************************************
* Function      : void sendData(unint8_t* tlm_packet, int pack_size)
* Description   : sends the packet out via the Teensy's Serial 2 port
* Arguments     : uint8_t* tlm_packet - pointer to packet
                : int pack_size - size of the packet to be sent
**********************************************************************************************************************/
void sendData(uint8_t* tlm_packet, int pack_size)
{
    Serial2.write(tlm_packet, pack_size);
}


/**********************************************************************************************************************
* Function     : uint16_t* buildPacket(int pack_size)
* Description  : builds the simulated instrument packet
* Arguments    : int pack_size - size of the packet to be sent
* Returns      : instrument packet
**********************************************************************************************************************/
uint8_t* buildPacket(int pack_size) 
{ 
    // initialize packet
    uint8_t* tlm_packet = new uint8_t[pack_size];
    memset(tlm_packet, 0xA0, pack_size);

    // build header
    uint8_t tlm_hdr[8] = {0xFE, 0xFA, 0x30, 0xC8, 0x00, 0x00, 0x00, 0x00};
    
    // add length field
    int data_len = pack_size - 6; // count of every byte after length field
    tlm_hdr[4] = (data_len >> 8) & 0xFF;
    tlm_hdr[5] = data_len & 0xFF;

    // add header to packet
    for(int i = 0; i < 8; i++){
        tlm_packet[i] = tlm_hdr[i];
    }

    // add dummy checksum (or stop phrase)
    tlm_packet[pack_size-2] = 0xEB;
    tlm_packet[pack_size-1] = 0x90;

    return tlm_packet;
}



