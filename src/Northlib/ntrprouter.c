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

#include "led.h"

#define SYNC_TIMEOUT     (50)     /* 50 seconds timeout */
#define USB_LED			 (0)
#define RADIO_LED 		 (1)

static int8_t ready; /* Syncronisation ? OK : ERROR */

static NTRPR_Pipe_t nrfPipe[NRF_MAX_PIPE_SIZE];  /* 5 PIPE. Do not use pipe 0 for multiceiver applications */
static uint8_t 	 	nrfPipeIndex;
static int8_t  	 	nrfLastTransmitIndex;      /* Last Transmit Pipe Index */

static uint8_t rxBuffer[NTRP_MAX_MSG_SIZE];   /* Master RX buffer */
static uint8_t txBuffer[NTRP_MAX_MSG_SIZE];   /* Slave Transmit buffer */

static NTRPR_Mode_e mode; /* RxTx , Rx, Tx modes. Default : RxTx */

static SemaphoreHandle_t radioMutex;

void NTRPR_UsbTask(void* argv);
void NTRPR_RadioTask(void* argv);
STATIC_MEM_TASK_ALLOC(NTRPR_USB, NTRPR_TASK_STACK,NTRPR_TASK_PRI);
STATIC_MEM_TASK_ALLOC(NTRPR_RADIO, NTRPR_TASK_STACK,NTRPR_TASK_PRI);

void NTRPR_Init(void){
	nrfPipeIndex = 1;
	nrfLastTransmitIndex = -1;
    ready = 0;
    mode = R_MODE_TRX;
	
    /* ESB INIT NEEDED */
    // RF24_Init(&RF24_SPI, RF24_CE_GPIO, RF24_CE, RF24_CS_GPIO, RF24_CS);
	// if(RF24_begin())serialPrint("[+] NTRPR Radio Init OK\n");
	// else {serialPrint("[-] NTRPR Radio Init ERROR!\n"); return;}
	// RF24_setDataRate(RF24_2MBPS);
	// RF24_setPALevel(RF24_PA_MAX, 0);

    ledWrite(RADIO_LED, 1);

	if(NTRPR_Sync(SYNC_TIMEOUT) == 0) return;

	radioMutex = xSemaphoreCreateMutex();

	STATIC_MEM_TASK_CREATE(NTRPR_USB, NTRPR_UsbTask, NULL);
	STATIC_MEM_TASK_CREATE(NTRPR_RADIO, NTRPR_RadioTask, NULL);
}

uint8_t NTRPR_Sync(uint32_t timeout){

    char syncbuffer[3];
    uint32_t timer = micros() + (timeout * 1000);
    while(micros() < timer){
    	while (usbAvailable() < 3){
            usbPrint(NTRP_SYNC_DATA);
            ledToggle(USB_LED);
            delay(100);
        }
    	usbReceive((uint8_t *)syncbuffer, 3);
    	if(strncmp(syncbuffer, NTRP_PAIR_DATA, 3) == 0){
    		ready = 1;
    		return  1;
    	}
    }
    return 0;
}

void NTRPR_UsbTask(void* argv){

	while(1){
	    NTRP_Message_t msgbuffer = NTRP_NewMessage();
	    /* Waits usb port for catch a success ntrp_message */
		usbWaitDataReady();
	    ledToggle(USB_LED);
	    if(NTRPR_ReceiveMaster(&msgbuffer)) NTRPR_Route(&msgbuffer);
	}
}

void NTRPR_RadioTask(void* argv){

	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1){
	    NTRP_Message_t msgbuffer = NTRP_NewMessage();
		/* Retries for catch a success ntrp_message */
		if(NTRPR_ReceivePipe(&msgbuffer)){
		    ledToggle(RADIO_LED);
			NTRPR_Route(&msgbuffer);
		}
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
	}
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

uint8_t NTRPR_ReceivePipe(NTRP_Message_t* msg){
	/* NRF Receive */
    if(mode == R_MODE_FULLTX) return 0;

    uint8_t pipe = 0;
	if (xSemaphoreTake(radioMutex, portMAX_DELAY) != pdTRUE) return 0;
    if(RF24_pipeAvailable(&pipe)){
        RF24_read(rxBuffer, NTRP_MAX_MSG_SIZE);

        NTRP_PackParse(&msg->packet, rxBuffer);

        msg->receiverID = NTRP_MASTER_ID;
        msg->talkerID   = nrfPipe[pipe].id;
        msg->packetsize = NTRP_MAX_PACKET_SIZE;
        xSemaphoreGive(radioMutex);
        return 1;
    }
    xSemaphoreGive(radioMutex);
    return 0;
}

uint8_t NTRPR_TransmitPipe(uint8_t pipeid, const NTRP_Packet_t* packet, uint8_t size){
    if(mode == R_MODE_FULLRX) return 0;
    if (pipeid == 0) return 0; /*Pipe ID needs to be an ascii char*/

	if (xSemaphoreTake(radioMutex, portMAX_DELAY) != pdTRUE) return 0;

    for (uint8_t i = 0; i < nrfPipeIndex; i++) /* Pipe index start @1 */
    {
        if(nrfPipe[i].id != pipeid) continue;

        NTRP_PackUnite(txBuffer, size, packet);

        if(nrfLastTransmitIndex != i){
        	NTRPR_Debug("OpenWritingPipe");
        	RF24_openWritingPipe(nrfPipe[i].txaddress); /* RF24 -> TX Settling (!!!CHANGES THE RX0_ADDRESS TOO!!!)*/
            nrfLastTransmitIndex = i;
        }

        if(mode == R_MODE_FULLTX){
        	RF24_write(txBuffer, size);
            return 1;
        }

        RF24_stopListening();
        RF24_write(txBuffer, size);
        RF24_startListening();
        xSemaphoreGive(radioMutex);
        return 1;
    }
    xSemaphoreGive(radioMutex);
    return 0;
}

void NTRPR_TransmitPipeFast( uint8_t pipeid, const uint8_t* raw_sentence, uint8_t size){
    if(mode == R_MODE_FULLRX) return;

	if (xSemaphoreTake(radioMutex, portMAX_DELAY) != pdTRUE) return;

    for (uint8_t i = 0; i < nrfPipeIndex; i++)
    {
        if(nrfPipe[i].id != pipeid) continue;

        if(mode == R_MODE_TRX) RF24_stopListening(); // Set to TX Mode for transaction

        if(nrfLastTransmitIndex != i){
        	RF24_openWritingPipe(nrfPipe[i].txaddress);
        	nrfLastTransmitIndex = i;
        }

        RF24_write(raw_sentence, size);
        if(mode == R_MODE_TRX) RF24_startListening(); // Set to RX Mode again
    }
    xSemaphoreGive(radioMutex);
}

void 	NTRPR_Route(NTRP_Message_t* msg){
	switch (msg->receiverID)
	{
	    case NTRP_MASTER_ID: NTRPR_TransmitMaster(msg);break;                       /* ReceiverID Master */
	    case NTRP_ROUTER_ID: NTRPR_RouterCOM(&msg->packet, msg->packetsize);break;  /* ReceiverID Router */
	    default:{
	        if(!NTRPR_TransmitPipe(msg->receiverID, &msg->packet, msg->packetsize)){ /* Search NRF pipes for Receiver Hit*/
	        	NTRPR_Debug("Packet Lost");
	            char payloadmsg[26];
	            sprintf(payloadmsg,"Talker %c: Receiver %c", msg->talkerID, msg->receiverID);
	            NTRPR_Debug(payloadmsg);
	        }
	        break;
	    }
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
            pipe.rxaddress[i] = cmd->data.bytes[i + 2]; /*301*/
        }

        pipe.txaddress[4]--; /*300*/

        if (NTRPR_OpenPipe(pipe)) NTRPR_Debug("NRF Pipe Opened");
        else NTRPR_Debug("NRF Pipe Error");
    break;
    case R_TRX:
    	NTRPR_Debug("NRF TRX");
        RF24_startListening();
        mode = R_MODE_TRX; break;
    case R_FULLRX:
    	NTRPR_Debug("NRF FULLRX");
        RF24_startListening();
        mode = R_MODE_FULLRX; break;
    case R_FULLTX:
    	NTRPR_Debug("NRF FULLTX");
        RF24_stopListening();
        mode = R_MODE_FULLTX; break;
    case R_CLOSEPIPE:
        /* Not Implemented */
    break;
    case R_EXIT:
        /* Not Implemented */
    break;
    default:break;
    }
}

uint8_t NTRPR_OpenPipe(NTRPR_Pipe_t cmd){
    if(nrfPipeIndex >= NRF_MAX_PIPE_SIZE) return 0; /* Reached Maximum Pipe Number */

    //TODO: nrf->setChannel(cmd.channel);
    //TODO: nrf->setSpeed(speeds[cmd.speedbyte])
    RF24_openReadingPipe(nrfPipeIndex, cmd.rxaddress); /*301*/
    RF24_startListening();

    nrfPipe[nrfPipeIndex] = cmd;  /* Pipe index is need to be same with nrf rx pipe index*/
    nrfPipeIndex++;               /* Incremented for next openpipe & reptresenting the pipe size */
    return 1;
}

void NTRPR_ClosePipe(char id){
	/* Not Implemented */
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if (GPIO_Pin == GPIO_PIN_15) {
//		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//
//		xSemaphoreGiveFromISR(radioReady, &xHigherPriorityTaskWoken);
//
//		if(xHigherPriorityTaskWoken){
//			portYIELD();
//		}
//	}
//}

