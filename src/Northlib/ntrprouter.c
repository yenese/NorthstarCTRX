/**
 *    __  __ ____ _  __ ____ ___ __  __
 * 	  \ \/ // __// |/ //  _// _ |\ \/ /
 *     \  // _/ /    /_/ / / __ | \  /
 *     /_//___//_/|_//___//_/ |_| /_/
 *
 *         Yeniay System Firmware
 *
 *       Copyright (C) 2024 Yeniay
 *
 * This  program  is  free software:   you
 * can  redistribute it  and/or  modify it
 * under  the  terms of  the  GNU  General
 * Public  License as  published  by   the
 * Free Software Foundation, in version 3.
 *
 * You  should  have  received  a  copy of
 * the  GNU  General  Public License along
 * with this program. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "systime.h"
#include "ntrprouter.h"
#include "ntrp.h"
#include "my_usb.h"
#include "rf52.h"
#include "led.h"

#define SYNC_TIMEOUT     (500)     /* 50 seconds timeout */
#define RX_LED			 (0)
#define TX_LED 		     (1)

static int8_t       ready;   /* Syncronisation ? OK : ERROR */
static NTRPR_Pipe_t nrfPipe;  /* One Main PIPE */

static uint8_t      rxBuffer[NTRP_MAX_MSG_SIZE];   /* Master RX buffer */
static uint8_t      txBuffer[NTRP_MAX_MSG_SIZE];   /* Slave Transmit buffer */

static NTRPR_Mode_e mode; /* RxTx , Rx, Tx modes. Default : RxTx */
void   NTRPR_UsbTask(void* argv);
void   NTRPR_RF52Callback(uint8_t* pRxData, uint16_t len);

void NTRPR_Init(void){
    ready = 0;
    mode = R_MODE_TRX;
	
    /* RF52 INIT NEEDED */
    rf52Init(NTRPR_RF52Callback);
    ledWrite(TX_LED, 1);

	if(NTRPR_Sync(SYNC_TIMEOUT) == 0) return;

    while (1){
        NTRPR_UsbTask();
        delay(1);
    }   
}

uint8_t NTRPR_Sync(uint32_t timeout){
    char syncbuffer[3];
    uint32_t timer = millis() + (timeout * 1000);
    while(millis() < timer){
    	while (usbAvailable() < 3){
            usbPrint(NTRP_SYNC_DATA);
            ledToggle(TX_LED);
            delay(100);
        }
    	usbReceive((uint8_t *)syncbuffer, 3);
    	if(strncmp(syncbuffer, NTRP_PAIR_DATA, 3) == 0){
    		ready = 1;
    		return  1;
    	}
        delay(1);
    }
    return 0;
}

void NTRPR_UsbTask(void* argv){
    NTRP_Message_t msgbuffer = NTRP_NewMessage();
    /* Waits usb port for catch a success ntrp_message */
    if(NTRPR_ReceiveMaster(&msgbuffer)){
        NTRPR_Route(&msgbuffer);
    }
}

void NTRPR_RF52Callback(uint8_t* pRxData, uint16_t len){
    NTRP_Message_t msg = NTRP_NewMessage();
    NTRP_PackParse(&msg.packet, pRxData);
    msg.receiverID = NTRP_MASTER_ID;
    msg.talkerID   = nrfPipe.id;
    msg.packetsize = NTRP_MAX_PACKET_SIZE;
    NTRPR_Route(&msg);
    ledToggle(RX_LED);
}

void NTRPR_Debug(const char* msg){
    NTRP_Message_t temp = NTRP_NewMessage();

    temp.talkerID   = NTRP_ROUTER_ID;
    temp.receiverID = NTRP_MASTER_ID;

    uint8_t i = 0;
    temp.packet.header = NTRP_MSG;

    while(i <= (NTRP_MAX_PACKET_SIZE - 2) && msg[i] != 0x00){
        temp.packet.data.bytes[i] = msg[i];
        i++;
    }

    temp.packet.dataID = i;
    temp.packetsize    = i + 2;
    NTRPR_TransmitMaster(&temp);
}

uint8_t NTRPR_ReceiveMaster(NTRP_Message_t* msg){
    uint16_t len = usbAvailable(); 
    if(len <= 0) return 0;
    if(len > NTRP_MAX_MSG_SIZE) len = NTRP_MAX_MSG_SIZE;
    usbRead(rxBuffer, len);
    return NTRP_Parse(msg, rxBuffer);
}

void NTRPR_TransmitMaster(const NTRP_Message_t* msg){
    if(NTRP_Unite(txBuffer, msg)){
        usbTransmit(txBuffer, msg->packetsize + 5);
    }
}

uint8_t NTRPR_TransmitPipe(uint8_t pipeid, const NTRP_Packet_t* packet, uint8_t size){
    NTRP_PackUnite(txBuffer, size, packet);
    rf52Transmit(txBuffer, size);
    ledToggle(TX_LED);
    return 1;
}

void NTRPR_Route(NTRP_Message_t* msg){
	switch (msg->receiverID){
	    case NTRP_MASTER_ID: NTRPR_TransmitMaster(msg);break;                       /* ReceiverID Master */
	    case NTRP_ROUTER_ID: NTRPR_RouterCOM(&msg->packet, msg->packetsize);break;  /* ReceiverID Router */
	    default: NTRPR_TransmitPipe(msg->receiverID, &msg->packet, msg->packetsize); break;
    }
}

void NTRPR_RouterCOM(NTRP_Packet_t* cmd, uint8_t size){

    /* SWITCH TO ROUTER COMMAND */
    switch (cmd->header){
    case NTRP_MSG: NTRPR_Debug("Router Message ACK"); break;
    case R_OPENPIPE:
        NTRPR_Pipe_t pipe;
        pipe.id =        cmd->dataID; 			  /* Reference PIPE id char : '1','2'... */
        pipe.channel =   cmd->data.bytes[0];
        pipe.bandwidth = cmd->data.bytes[1];

        for(uint8_t i = 0; i < 5 ; i++){
            pipe.txaddress[i] = cmd->data.bytes[i + 2]; /*300*/
            pipe.rxaddress[i] = cmd->data.bytes[i + 7]; /*301*/
        }

        pipe.txaddress[4]--; /*300*/
        if  (NTRPR_OpenPipe(pipe)) NTRPR_Debug("NRF Pipe Opened");
        else NTRPR_Debug("NRF Pipe Error");
    break;
    case R_TRX: break;
    case R_FULLRX: break;
    case R_FULLTX: break;
    case R_CLOSEPIPE: break;
    case R_EXIT: break;
    default:break;
    }
}

uint8_t NTRPR_OpenPipe(NTRPR_Pipe_t cmd){
    nrfPipe = cmd;
    rf52SetChannel(cmd.channel % 100);
    return 1;
}

void NTRPR_ClosePipe(char id){
	/* Not Implemented */
}
