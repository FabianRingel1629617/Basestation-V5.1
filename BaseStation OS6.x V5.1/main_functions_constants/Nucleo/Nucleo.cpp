#include "Nucleo.h"
#include "BNO055.h"
#include "Constants.h"
#include <cstdint>

//User button with interrupt + Timer for debounce
InterruptIn button(BUTTON1);
Timer buttonTimer; 

//Objects of needed classes
Nrf24l01p nrf(D11, D12, D13, D4, D5); 	// Create an object for the NRF24L01Plus class
BufferedSerial pc(USBTX, USBRX);		// Serial connection

PwmOut buzzer(PB_3);

uint8_t nrf_data_address[NRF_ADDR_LEN]      = {0xBA,0x5E,0xDA,0x7A,0xFF};       // change 0xFF to actual node
uint8_t nrf_broadcast_address[NRF_ADDR_LEN] = {0xBA,0x5E,0xCA,0x57,0x3d};       //

/**
 * 0x00  = Mode 0 (Quaternion Only)
 * 0x01  = Mode 1 (Quaternion and Linear Acceleration)
 * 0x02  = (For the recovery of the Glove-Configuration) Gloves go into "Idle" Mode, some commands can only be executed in idle mode, like collection of the calibration data
 * 0x03  = Getting IMUData from BaseStation (Only for Testing)
 * 0x04  = Sending of a Command from BS to Glove
 * 0x05  = 
 * 0x06  = 
 * 0x07  = 
 * 0x08  = 
 * 0x09  = 
 */
uint8_t BS_payload_TX[TX_PAYLOAD_LEN] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
uint8_t BS_payload_command[2] = {0xFF, 0xFF};

uint8_t RX_buffer_1[PAYLOAD_MAX_LEN] = {0};
uint8_t RX_buffer_2[PAYLOAD_MAX_LEN] = {0};
uint8_t RX_buffer_3[PAYLOAD_MAX_LEN] = {0};
uint8_t RX_buffer_broadcast[PAYLOAD_MAX_LEN] = {0};

uint8_t baseStationMode{0};
uint8_t baseStationCmd{0}; 
uint8_t baseStationBNOMode{0};                   
uint8_t cmdTypeAndCmd[MAX_NUM_NODES][2]{{0x01, 0x00}, {0x01, 0x00}, {0x01, 0x00}, {0x01, 0x00}, 
                                        {0x01, 0x00}, {0x01, 0x00}, {0x01, 0x00}, {0x01, 0x00}, 
                                        {0x01, 0x00}, {0x01, 0x00}, {0x01, 0x00}, {0x01, 0x00}, 
                                        {0x01, 0x00}, {0x01, 0x00}, {0x01, 0x00}, {0x01, 0x00}};
uint8_t cmdTypeCmdAndData[MAX_NUM_NODES][3]{0};
uint8_t gloveUID[MAX_NUM_NODES]{0};

/*****------*****/
uint16_t confNodes = 0;
uint16_t activNodes = 0;
uint8_t curNode = 0;
uint8_t freeNodeID = 255;

/*****-----*****/
uint8_t sessionID = 0x00;

bool dataChanged = false;   //Indicates that the SessionID should be updated and send

/*****---Serial buffers---*****/
char serialRXBuffer[BUFFER_LENGTH] = {0};
uint8_t commandBuffer[BUFFER_LENGTH]  = {0};


/************************************************************************************
** Nucleo_Init function:
** - Start-up delay
** - Initializes the I/O peripherals
** - Needed for the BNO055 else not
*************************************************************************************/
void Nucleo_Init(void)
{
    ThisThread::sleep_for(750ms);
    //wait(0.75);         //750ms mandatory delay for BNO055 POR
}

void serial_write(uint8_t* buffer, uint8_t rxLen)
{
    pc.write(buffer, rxLen);
}

/*
void Button_Interrupt() 
{
    //ThisThread::sleep_for(50ms); //nicht thread safe!!! 
    buttonTimer.start(); // start timer counting
    while(chrono::duration_cast<chrono::milliseconds>(buttonTimer.elapsed_time()).count() <50){}
    //while(buttonTimer.read_ms() < 50) {} //deprecated
    //buttonTimer.stop();
    //wait_ms(50);    //Delay for switch debounce
    if (mode == 0)
    {
    	mode = 1;
    }
    else if (mode == 1)
    {
    	mode = 2;
    } else if (mode ==2) { //only for Testing
        mode = 3;
    } else {
        mode = 0;
    }
    dataChanged = true;
}
*/

void updateBitset(uint16_t &bitSet, uint8_t nodeID, bool setBit) {   //Setzen/Löschen des Bits an der angegebenen stelle
    if (nodeID < 16) {
        if (setBit) {
            bitSet |= bitmasksSet[nodeID];
        } else {
            bitSet &= bitmasksClear[nodeID];
        }   
    }
}

bool getBitState(uint16_t bitSet, uint8_t id){
    if (id > 15) {
        return false;
    }
    return bitSet & checkMask[id];
}

uint8_t getNextFreeNodeID(uint16_t bitSet){                                 //Suchen der kleinsten freien NodeID
    for (int ID = 0; ID < 16; ID++) {
        if (! (bitSet & bitmasksSet[ID])) {
            return ID;
        }
    }
    return 255;
}

uint8_t getNextNodeID(uint16_t bitSet, uint8_t currentNode) {               //
    if(bitSet == 0) {
        return 255;
    }
    do {
        currentNode++;
        if (currentNode > 15) {
            currentNode = 0;
        }
    } while (!(bitSet & bitmasksSet[currentNode]) );
    return currentNode;
}

uint8_t getFirstNodeID(uint16_t bitSet) {
    for (int ID = 0; ID < 16; ID++) {
        if (bitSet & bitmasksSet[ID]) {
            return ID;
        }
    }
    return 255;
}

void recoverNodes(uint8_t &numNodes) {
    bool rx, txDone, maxTry;
    
    uint8_t rxLen = 0;
    uint8_t pipe = 0;

    uint8_t currentBuffer[PAYLOAD_MAX_LEN] = {0};

    uint8_t recoverCmd[2] = {COMMAND_TYPE_GENERAL_COMMAND, GENERAL_COMMAND_RECOVER_GLOVE};
    
    for (uint8_t i = 0; i < MAX_NUM_NODES; ++i)
    {
        uint8_t tempID = -1;
        nrf_data_address[4] = i;
        
        nrf.setTXAddress(nrf_data_address, NRF_ADDR_LEN);
        nrf.setRXAddress(0, nrf_data_address, NRF_ADDR_LEN);
        
        nrf.writeTXData(recoverCmd, 2);
        ThisThread::sleep_for(5ms);
        // get status and reset interrupt flags
        nrf.getIRQStatus(rx, txDone, maxTry);
        nrf.resetIRQFlags();


        
        while (txDone && !rx) {                 //TODO find abort condition in case of fault data
            nrf.writeTXData(recoverCmd, 2);
            ThisThread::sleep_for(1ms);
        }

        if (rx) //TODO find abort condition in case of fault data
        {
			while (true)
			{
				nrf.readRXData(currentBuffer, rxLen, pipe);
	        	if (rxLen == 6 && currentBuffer[5] == 0xAA)
	        	{
                    tempID = currentBuffer[2];
                    cmdTypeAndCmd[tempID][0] = currentBuffer[0];    //Empfangenen Befehlsytp nutzen
                    cmdTypeAndCmd[tempID][1] = currentBuffer[1];    //Empfangenen Befehl nutzen
                    updateBitset(confNodes, tempID);                //Empfangene GloveID nutzen
                    updateBitset(activNodes, tempID);               //Empfangene GloveID nutzen

                    updateSessionID(currentBuffer[3]);              //Empfangene SessionID nutzen

                    gloveUID[i] = currentBuffer[4];                 //Empfangene GloveUID nutzen

                    numNodes++;
                    break;
				}
                nrf.writeTXData(recoverCmd, 2);
			}
		}
    }
}

void do_broadcast(uint8_t cmdType, uint8_t cmd, uint8_t nextID, uint8_t sessionID, uint8_t &numNodes) 
{
    uint8_t rxLen {0}, pipe{0};
    if(!nrf.RXFifoEmpty()) {   //empty the RX-FIFO to ensure that each received message is from a glove that is to be configured
        nrf.readRXData(RX_buffer_3, rxLen, pipe);
        ThisThread::sleep_for(1ms);
    }

    nrf.setTXAddress(nrf_broadcast_address, NRF_ADDR_LEN);
    nrf.setRXAddress(0, nrf_broadcast_address, NRF_ADDR_LEN);

    uint8_t BS_broadcast_payload[4] = {cmdType, cmd, nextID, sessionID };

    nrf.writeTXData(BS_broadcast_payload, 4);

    bool rx, txDone, maxTry;

    // get status and reset interrupt flags
    nrf.getIRQStatus(rx, txDone, maxTry);
    nrf.resetIRQFlags();

    bool findDoubleUID = false;

    if (rx)
    {
        buzzer.write(0.2f);
        nrf.readRXData(RX_buffer_broadcast, rxLen, pipe);

        if (rxLen > 0)
        {
            //serial_write(RX_buffer_broadcast, rxLen); //See which data comes back
            updateBitset(confNodes,  nextID); //RX_buffer_broadcast 
            updateBitset(activNodes, nextID); //RX_buffer_broadcast
            
            numNodes++;
            dataChanged = true;

            if (rxLen == 3 && RX_buffer_broadcast[2] == 0xAA) { //should alwasy be the case
                uint8_t gUID = RX_buffer_broadcast[1];
                for (int i = 0; i < 16; i++) {
                    if (gUID == gloveUID[i]) {
                        buzzer.write(0.2f);
                        ThisThread::sleep_for(50ms);
                        buzzer.write(0.0f);
                        ThisThread::sleep_for(50ms);
                        findDoubleUID = true;
                        //set Flag true
                        //send in mainloop command to skript to start recover
                    }
                }
                gloveUID[nextID] = gUID;
            }
        }
  
    }
    if (findDoubleUID) {        // Send a command that a double GloveID was detected
        uint8_t scriptCmd[] = {0xAB, 0xCD, 0xFF, 0xFF, COMMAND_TYPE_SCRIPT_COMMAND, SCIRPT_COMMAND_FIND_INACTIVE_NODES};
        serial_write(scriptCmd, 6);
    }

    ThisThread::sleep_for(50ms);
    buzzer.write(0.0f);

}


void updateSessionID(uint8_t ID) {
    sessionID = ID;
     //TODO
    //Check which id is to be used
    //ExampleID 0, 5, 241, 255 = which to use? (should normally not happen, such a complete different range)
    //In oder, but what if different order
    //Smallest or biggest id
    //Better alternative for session id, like a timestamp?
}

/**
 * command = Welcher Befehl ausgeführt werden soll, z.B. SessionID, Mode, Reset, ...
 * data = Daten die verteilt werden sollen
 * dataLength = Anzahl der Daten in Byte (Maximum length of 30 Bytes)
 * broadcastCommand = ob das Commando an alle verteilt werden soll oder nur an eine oder mehrere bestimmte Nodes (noch nicht genutzt)
 */
void sendCommand(CommandType commandType, uint8_t command, uint8_t data[], uint8_t dataLength, bool broadcastCommand, uint16_t recipients) { // bool nooch ungenutzt aber nutzen, falls ein commando nur an eine Station gesendet werden soll
    uint8_t oldId = nrf_data_address[4];    //Save the old ID to restore it after sending the commands

    uint8_t BS_command_payload[dataLength+2];
    BS_command_payload[0] = commandType;    // CommandType

    if (commandType == COMMAND_TYPE_GENERAL_COMMAND) {
        BS_command_payload[1] = castGeneralCommand(command);    //Cast General Command
    } else if (commandType == COMMAND_TYPE_MODE_COMMAND) {
        BS_command_payload[1] = castModeCommand(command);       //Cast Mode Command
    } else if (commandType == COMMAND_TYPE_INFO_COMMAND) {
        BS_command_payload[1] = castInfoCommand(command);       //Cast Info Command
    } else if (commandType == COMMAND_TYPE_UPDATE_COMMAND) {
        BS_command_payload[1] = castUpdateCommand(command);     //Cast Update Command
    }
    
    if (dataLength > 0 && data != nullptr) {
        for (int i = 0; i < dataLength; i++) {
            BS_command_payload[i+2] = data[i];
        }
    }

    ThisThread::sleep_for(3ms); //To "make" sure, that the RX-Pipe of the gloves is empty 3ms could be enough but to be sure, make 5 milli seconds, so the the fifo should be empty

    if (broadcastCommand) {
        nrf.disableAutoAck(0);  // Kein Ack, da die Daten nur verteilt werden sollen, aber direkt an alle
        nrf_data_address[4] = 0xFF;
        nrf.setTXAddress(nrf_data_address, NRF_ADDR_LEN);
        nrf.setRXAddress(0, nrf_data_address, NRF_ADDR_LEN);
        nrf.writeTXData(BS_command_payload, dataLength+2);
        nrf.enableAutoAck(0);   // Ack wieder aktivieren für alle anderen Übertragungen
    } else {
            
            nrf.setRetries(500, 3);     //To increase the probability that the command will be received
            for (int i = 0; i < MAX_NUM_NODES; i++) {
                if (recipients & checkMask[i]) {
                    nrf_data_address[4] = i;
                    nrf.setTXAddress(nrf_data_address, NRF_ADDR_LEN);
                    nrf.setRXAddress(0, nrf_data_address, NRF_ADDR_LEN);
                    nrf.writeTXData(BS_command_payload, dataLength+2);
                }
            }
            nrf.resetIRQFlags();
            nrf.setRetries(500, 1);
    }

    nrf_data_address[4] = oldId;            //Restoring the old ID

}

void sendDataToBS(CommandType commandType, RequestBsData dataType, uint8_t data[], uint8_t dataLength) { //Sendet Daten an die BS im Moment einzeln eventuell umschreiben, dass mehrere Daten gelcihzeit gesendet werden
    uint8_t BS_data_payload[dataLength+5];
    BS_data_payload[0] = 0xAB; //SyncByte 
    BS_data_payload[1] = 0xCD; //SyncByte 
    BS_data_payload[2] = 0xFF; //Signalisiert, dass Informationsdaten und keine ...
    BS_data_payload[3] = 0xFF; //... Sensordaten gesendet werden
    BS_data_payload[4] = commandType;
    BS_data_payload[5] = dataType; //gibt den Datentyp an z.B. SessionID, Kalibierungsdaten, etc
    if (dataLength != 0 && data != nullptr) {
        for (int i = 0; i < dataLength; i++) { //Die Informationsdaten
            BS_data_payload[i+6] = data[i];
        }
    }

    serial_write(BS_data_payload, dataLength+6);
}

void sendDataToBS(CommandType commandType, RequestBsData dataType, uint16_t data[], uint8_t dataLength) {
    if (data == nullptr || dataLength == 0) {
        sendDataToBS(commandType, dataType, data, dataLength*2);
    }
    uint8_t tempData[dataLength*2];
    for (int i = 0; i < dataLength; i++) {
        tempData[i]   = data[i] >> 8;
        tempData[i+1] = data[i] & 0xFF;
    }
    sendDataToBS(commandType, dataType, tempData, dataLength*2);
}

void findInactiveNodes(uint16_t &confNodes, uint16_t &activNodes, uint16_t updateData, uint8_t &numNodes) {
    if (activNodes == updateData) {     //eleminate the GloveIDs/DataIDs permanently
        confNodes = updateData;
        numNodes = 0;
        for (int i = 0; i < 16; i++) {
            if (confNodes & checkMask[i]) {
                numNodes++;
            } else {
                gloveUID[i] = 0x00; // set the GloveUID for the given ID to zero
            }
        }
    } else {    //elemeniate the GloveIDs/DataIDs temporarly
        activNodes = updateData;
    }
    dataChanged = true;
}

void readQuatLinAccCompressed(BNO055 *bno, uint8_t *data_buffer) {
    if (bno->available()) {
        data_buffer[0] = BNO_DATA_QUAT_LIN_ACC;
        bno->get_quat_lin_acc();
        data_buffer[1]  = bno->quat.rawx & 0xff;
        data_buffer[2]  = bno->quat.rawx >> 8;
        data_buffer[3]  = bno->quat.rawy & 0xff;
        data_buffer[4]  = bno->quat.rawy >> 8;
        data_buffer[5]  = bno->quat.rawz & 0xff;
        data_buffer[6]  = bno->quat.rawz >> 8;
        data_buffer[7]  = bno->lia.rawx & 0xff;
        data_buffer[8]  = bno->lia.rawx >> 8;
        data_buffer[9]  = bno->lia.rawy & 0xff;
        data_buffer[10] = bno->lia.rawy >> 8;
        data_buffer[11] = bno->lia.rawz & 0xff;
        data_buffer[12] = bno->lia.rawz >> 8;
    }
}

void addToBuffer(char* tempChar, int position) {
    serialRXBuffer[position] = *tempChar;
}

//COMMAND_TYPE_GENERAL_COMMAND // 0
GeneralCommand castGeneralCommand(char commandChar){  //static cast also possible?!
    switch(commandChar) {
        case 0x00:
            return GENERAL_COMMAND_QUERY;
        case 0x01:
            return GENERAL_COMMAND_IDLE;
        case 0x02:
            return GENERAL_COMMAND_RESTART;
        case 0x03:
            return GENERAL_COMMAND_RECOVER_GLOVE;
        case 0x04:
            return GENERAL_COMMAND_CONFIGURE_GLOVE;
        case 0x05:
            return GENERAL_COMMAND_SHOW_ID;
    }
    return GENERAL_COMMAND_UNKNOWN;
}

//COMMAND_TYPE_MODE_COMMAND // 1
ModeCommand castModeCommand(char commandChar){  //static cast also possible?!
    switch(commandChar) {
        case 0x00:
            return MODE_COMMAND_QUAT;
        case 0x01:
            return MODE_COMMAND_QUAT_LIN_ACC;
    }
    return MODE_COMMAND_UNKNOWN;
} 

//COMMAND_TYPE_INFO_COMMAND // 2
InfoCommand castInfoCommand(char commandChar){  //static cast also possible?!
    switch(commandChar) {
        case 0x00:
            return INFO_COMMAND_CALIBRATION_DATA;
        case 0x01:
            return INFO_COMMAND_GLOVE_CONFIG;
    }
    return INFO_COMMAND_UNKNOWN;
} 

// COMMAND_TYPE_UPDATE_COMMAND // 3
UpdateCommand castUpdateCommand(char commandChar){  //static cast also possible?!
    switch(commandChar) {
        case 0x00:
            return UPDATE_COMMAND_SESSION_ID;
    }
    return UPDATE_COMMAND_UNKNOWN;
}

BasestationMode castBSModeCommand(char commandChar) {
    switch(commandChar) {
        case 0x00:
            return BASESTATION_MODE_GLOVE_MODE;
        case 0x01:
            return BASESTATION_MODE_BS_MODE;
    }
    return BASESTATION_MODE_UNKNOWN;
}

UpdateBsData castUpdateBSCommand(char commandChar) {
    switch(commandChar) {
        case 0x00:
            return UPDATE_BS_DATA_SESSION_ID;
        case 0x01:
            return UPDATE_BS_DATA_CONFIGURED_GLOVES;
        case 0x02:
            return UPDATE_BS_DATA_ACTIVE_GLOVES;
        case 0x03:
            return UPDATE_BS_DATA_FIND_INACTIVE_NODES;
    }
    return UPDATE_BS_DATA_UNKNOWN;
}

RequestBsData castRequestBSCommand(char commandChar) {
    switch(commandChar) {
        case 0x00:
            return REQUEST_BS_DATA_NUMBER_OF_NODES;
        case 0x01:
            return REQUEST_BS_DATA_SESSION_ID;
        case 0x02:
            return REQUEST_BS_DATA_BNO;
        case 0x03:
            return REQUEST_BS_DATA_CONFIGURED_NODES;
        case 0x04:
            return REQUEST_BS_DATA_ACTIVE_NODES;
    }
    return REQUEST_BS_DATA_UNKNOWN;
}
    
// CommandType
CommandType castCommandType(char commandChar){  //static cast also possible?!
    switch(commandChar) {
        case 0x00:
            return COMMAND_TYPE_GENERAL_COMMAND;
        case 0x01:
            return COMMAND_TYPE_MODE_COMMAND;
        case 0x02:
            return COMMAND_TYPE_INFO_COMMAND;
        case 0x03:
            return COMMAND_TYPE_UPDATE_COMMAND;
        case 0x7F:
            return COMMAND_TYPE_SCRIPT_COMMAND;
        case 0x80:
            return COMMAND_TYPE_CHANGE_BS_MODE;
        case 0xFD:
            return COMMAND_TYPE_UPDATE_BS_DATA;
        case 0xFE:
            return COMMAND_TYPE_REQUEST_BS_DATA;

    }
    return COMMAND_TYPE_UNKNOWN;
} 

// TODO: Prüfen, ob die serialData array die Bufferlänge des CommandBuffer nicht überschreitet 
//TODO alle Parameter wenn mögich auf korrektheit überprüfen
bool extractCommandData(char* serialData, int serialDataLength, uint8_t (& commandDataBuffer)[BUFFER_LENGTH], uint8_t &commandDuration, uint16_t &commandRecipient, CommandType &commandType, uint8_t &command, uint8_t &commandLength) {
    bool dataFalse = false;
    if (serialDataLength >= 7 && serialData[0] == 0xAB && serialData[1] == 0xCD) { //2 Byte
        commandDuration = serialData[2];                            //Befehlsdauer 1 Byte
        commandRecipient = serialData[3] << 8 | serialData[4];      //Empfänger 2 Byte
        commandType = castCommandType(serialData[5]);               //Befehlsart 1 Byte
        command = serialData[6];                                    //Befehlstyp 1 Bytes
        commandLength = serialDataLength-7; //Länge minus 7 da die ersten 7 Daten von der Seriellen Daten keine Daten sind sonder nur Sync oder der Befehl bzw der Empfänger, ...
        checkCmdParameter(commandDuration, commandRecipient, commandType, command);
        for (int i = 7; i < serialDataLength; i++) {
            commandDataBuffer[i-7] = serialData[i];
        }
        
        //if cmdType == 255 return false;
        return true;
    }
    return false;
}

bool checkCmdParameter(uint8_t commandDuration, uint16_t commandRecipient, uint8_t commandType, uint8_t command) {
    if (commandDuration != 0x00 && commandDuration != 0x01) {       //Wrong duration
        return false;
    }
    if (commandType >= 0x81 && commandRecipient != 0x00 || commandType <= 0x7E && commandRecipient == 0x00) {   //Wrong command type for Glove or BS
        return false;
    } 
    if (commandRecipient == 0x00) { //check BS Commands
        if (commandType == COMMAND_TYPE_CHANGE_BS_MODE) { 
            if (castBSModeCommand(command) == 0xFF) {
                return false;
            }
        } else if (commandType == COMMAND_TYPE_UPDATE_BS_DATA) {
            if (castUpdateBSCommand(command) == 0xFF) {
                return false;
            }
        } else if (commandType == COMMAND_TYPE_REQUEST_BS_DATA) {
            if (castRequestBSCommand(command) == 0xFF) {
                return false;
            }
        } 
    } else if (commandRecipient != 0x00) { //check Glove Commands
        if (commandType == COMMAND_TYPE_GENERAL_COMMAND) { 
            if (castGeneralCommand(command) == 0xFF) {
                return false;
            }
        } else if (commandType == COMMAND_TYPE_MODE_COMMAND) { 
            if (castModeCommand(command) == 0xFF) {
                return false;
            }
        } else if (commandType == COMMAND_TYPE_INFO_COMMAND) { 
            if (castInfoCommand(command) == 0xFF) {
                return false;
            }
        } else if (commandType == COMMAND_TYPE_UPDATE_COMMAND) { 
            if (castUpdateCommand(command) == 0xFF) {
                return false;
            }
        }
    }

    return true;
}

void clearBuffer(){
    for (int i = 0; i < BUFFER_LENGTH; i++) {
        serialRXBuffer[i] = 0;
    }
}


void executeBSCommand(BNO055 &bno, uint8_t stationData[32], uint8_t bsCmd, uint8_t numNodes, bool inBSMode) {
    if (bsCmd == REQUEST_BS_DATA_NUMBER_OF_NODES) {                                                     //Send Number of Nodes
        sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_NUMBER_OF_NODES, &numNodes, 1);
    } else if (bsCmd == REQUEST_BS_DATA_SESSION_ID) {                                                   //Send SessionID
        sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_SESSION_ID, &sessionID, 1);
    } else if (bsCmd == REQUEST_BS_DATA_BNO) {                                                          //Send BNO-Data
        readQuatLinAccCompressed(&bno, stationData); //Liest alle 12 Bytes für Quat (6) und LinAcc (6) gibt aber 6 oder 12 aus
        if (baseStationBNOMode == BNO_DATA_QUAT) {                                                      //Nur Quat senden
            stationData[0] = BNO_DATA_QUAT;
            sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_BNO, stationData, 7);
        } else if (baseStationBNOMode == BNO_DATA_QUAT_LIN_ACC) {                                       //Quat und LinAcc senden
            stationData[0] = BNO_DATA_QUAT_LIN_ACC;
            sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_BNO, stationData, 13);
        }
    } else if (bsCmd == REQUEST_BS_DATA_CONFIGURED_NODES) {                                             //Configured Nodes
        sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_CONFIGURED_NODES, &confNodes, 1);
    } else if (bsCmd == REQUEST_BS_DATA_ACTIVE_NODES) {                                                 //Active Nodes
        sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_ACTIVE_NODES, &activNodes, 1);
    }

    if (inBSMode) {
        ThisThread::sleep_for(10ms);    //Needed for BNO to send with 100Hz (for the others not needed, but so the data will no be send to often)
    }
}