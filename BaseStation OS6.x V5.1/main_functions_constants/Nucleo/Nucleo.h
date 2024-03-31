#ifndef __NUCLEO_H__
#define __NUCLEO_H__

#include "../Constants.h"
#include "../../BNO055/BNO055.h"
#include "../../Nrf24l019/Nrf24l01p.h"

#include "mbed.h"

//User button with interrupt + Timer for debounce
extern InterruptIn button;      //Interrupt for the Button
extern Timer buttonTimer;       //Timer for the Button

//Objects of needed classes
extern Nrf24l01p nrf; 	        // Create an object for the NRF24L01Plus class
extern BufferedSerial pc;		// Serial connection

extern PwmOut buzzer;                                   //Buzzer

/** Adresses **/
extern uint8_t nrf_data_address[NRF_ADDR_LEN];          //NRF-Data-Address
extern uint8_t nrf_broadcast_address[NRF_ADDR_LEN];     //NRF-Broadcast-Address

//Payload-"Buffer" for sending
extern uint8_t BS_payload_TX[TX_PAYLOAD_LEN];           //
extern uint8_t BS_payload_command[2];                   //

//Buffer for Receiving
extern uint8_t RX_buffer_1[PAYLOAD_MAX_LEN];            //RX-Buffer 1 for recieved NRF-payload
extern uint8_t RX_buffer_2[PAYLOAD_MAX_LEN];            //RX-Buffer 2 for recieved NRF-payload
extern uint8_t RX_buffer_broadcast[PAYLOAD_MAX_LEN];    //RX-Buffer for recieved NRF-payload, when a broadcast is send

/** Mode **/
extern uint8_t baseStationMode;                         //Speichert den Modus der Basisstation //0 = Glove Mode, 1 = Basestation Mode     
extern uint8_t baseStationCmd;                          //Gibt an welcher Befehl von der BS empfangen werden sollen
extern uint8_t baseStationBNOMode;                      //Gibt an welche Daten vom BNO abgefragt werden sollen.                        
extern uint8_t cmdTypeAndCmd[MAX_NUM_NODES][2];         //Speichert alle Commandtypen und Commandos der Nodes
extern uint8_t cmdTypeCmdAndData[MAX_NUM_NODES][3];     //Speichert alle Commandtypen und Commandos der Nodes plus eventuelle Daten (1 Byte für die Daten)
extern uint8_t gloveUID[MAX_NUM_NODES];

/*****---Node managment---*****/
extern uint16_t confNodes;  //Array of configured nodes
extern uint16_t activNodes; //Array of active nodes
extern uint8_t curNode;     //Currently active Node = Node for which aata should be requested or to which a command should be send
extern uint8_t freeNodeID;  //Variable, in which the smallest free node id are saved

/*****-----*****/
extern uint8_t sessionID;   //

extern bool dataChanged;    //Indicates that the SessionID should be updated and send

/*****---Serial---*****/
extern char serialRXBuffer[BUFFER_LENGTH];      //Buffer for the recieved serial data
extern uint8_t commandBuffer[BUFFER_LENGTH];    //Buffer for the extracted data to an command


void Nucleo_Init(void);                                 //

void serial_write(uint8_t* buffer, uint8_t rxLen);      //

void Button_Interrupt();                                //Mehtod that is called when the interrupt for the button is triggered

/** **/
void updateBitset(uint16_t &bitSet, uint8_t nodeID, bool setBit = true);    //
bool getBitState(uint16_t bitSet, uint8_t id);                              //
uint8_t getNextFreeNodeID(uint16_t bitSet);                                 //
uint8_t getNextNodeID(uint16_t bitSet, uint8_t currentNode);                //
uint8_t getFirstNodeID(uint16_t bitSet);                                    //

void recoverNodes(uint8_t &numNodes);                                                                       //
void do_broadcast(uint8_t cmdType, uint8_t cmd, uint8_t nextID, uint8_t sessionID, uint8_t &numNodes);      //

void updateSessionID(uint8_t ID);   //

void sendCommand(CommandType commandType, uint8_t command, uint8_t data[] = nullptr, uint8_t dataLength = 0, bool broadcastCommand = true, uint16_t recipients = 255);
void sendDataToBS(CommandType commandType, RequestBsData dataType, uint8_t data[] = nullptr, uint8_t dataLength = 0);
void sendDataToBS(CommandType commandType, RequestBsData dataType, uint16_t data[], uint8_t dataLength);

void findInactiveNodes(uint16_t &confNodes, uint16_t &activNodes, uint16_t updateData, uint8_t &numNodes);

void readQuatLinAccCompressed(BNO055 *bno, uint8_t *data_buffer);


void addToBuffer(char* tempChar, int position);                          //
bool extractCommandData(char* serialData, int serialDataLength, uint8_t (& commandDataBuffer)[BUFFER_LENGTH], uint8_t &commandDuration, uint16_t &commandRecipient, CommandType &commandType, uint8_t &command, uint8_t &commandLength);                  //
bool checkCmdParameter(uint8_t commandDuration, uint16_t commandRecipient, uint8_t commandType, uint8_t command);


CommandType castCommandType(char commandChar);                           //
GeneralCommand castGeneralCommand(char commandChar);                     //
ModeCommand castModeCommand(char commandChar);                           //
InfoCommand castInfoCommand(char commandChar);                           //
UpdateCommand castUpdateCommand(char commandChar);                       //


BasestationMode castBSModeCommand(char commandChar);                     //
UpdateBsData castUpdateBSCommand(char commandChar);                      //
RequestBsData castRequestBSCommand(char commandChar);                    //

void clearBuffer();

void executeBSCommand(BNO055 &bno, uint8_t stationData[32], uint8_t bsCmd, uint8_t numNodes, bool inBSMode = true);

#endif /* __NUCLEO_H__ */