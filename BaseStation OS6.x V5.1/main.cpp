#include "./main_functions_constants/Nucleo/Nucleo.h"

#include <cstdint>

#include "Constants.h"
#include "mbed.h"

/**************************************************************
**                       MAIN
***************************************************************/
int main()
{   
   buzzer.period_ms(1);

	Nucleo_Init();
	pc.set_baud(500'000);                                       // Set Serial baud-rate

    uint8_t stationData[32];
    
    //BNO Test und Ausgabe, sonst noch nicht genutzt
    
    BNO055 bno(I2C_SDA, I2C_SCL);
    bno.reset();
    bno.set_mapping(3);
    ThisThread::sleep_for(100ms);
    //wait_ms(100);
    bno.get_mapping();
    bno.setmode(OPERATION_MODE_NDOF);
    ThisThread::sleep_for(100ms);

	baseStationMode = 0;                //
    uint8_t numNodes = 0;               // No nodes available at beginning
    uint8_t broadcast_counter = 0;      // Wann Broadcast abgefragt werden soll, am besten ein vielfaches von der Anzahl der verbundenen Nodes, damit jede Node mehrmals abgefagt werden und nicht so viele Broadcast gemacht werden
	uint8_t loop_pass_for_query = 16;   // Teilt den BradcastCounter und gibt an wie oft jede Node abgefagt werden soll

    nrf.init(0x69, DR_1M, NRF_ADDR_LEN, 1);
	
	nrf.openDynamicTXPipe(nrf_broadcast_address, true, false); //darf nicht von Anfang an auf der broadcast-adresse sein, sonst bleibt die BS hängen muss ich also ändern
	//nrf.openDynamicTXPipe(nrf_data_address, true, false);

	// do not use interrupts, use polling instead
    nrf.maskIRQ(true, true, true);
	
	nrf.setModeTX();
    nrf.flushTX();
    nrf.flushRX();
    nrf.resetIRQFlags();
	
    //Attach the address of the Button Interrupt function to the falling edge
    //button.fall(&Button_Interrupt); 
    //button.enable_irq();
    
    bool rx, txDone, maxTry;
    
    uint8_t rxLen = 0;
    uint8_t pipe = 0;
    
    uint8_t* currentBuffer = RX_buffer_1;
    
    // prevent mbed from going into deep sleep
    DeepSleepLock lock;

    // set num retries higher and set it back to 1 after init phase
	nrf.setRetries(500, 5);

    // check if nodes already are online (e.g. when base station was reset during a session)
    // if so, recover session
    recoverNodes(numNodes);

    if (sessionID < 0xFF) {
        sessionID += 1; //setzen der neusten ID + 1
    } else {
        sessionID = 0x00;  //neuste ID ist 255, also wieder mit 0 beginnen
        //serial_write(&sessionID, 1);
    }

    sendCommand(COMMAND_TYPE_UPDATE_COMMAND, UPDATE_COMMAND_SESSION_ID, &sessionID, 1);  //Senden der aktuellen SessionID an die Gloves
    ThisThread::sleep_for(2s);
    //sendCommand(COMMAND_RestartNode);

    //send number of nodes and session id via the serial interface in order to use it
    sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_NUMBER_OF_NODES, &numNodes, 1); 
    ThisThread::sleep_for(1ms);
    sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_SESSION_ID, &sessionID, 2);
    // ThisThread::sleep_for(1ms);
    

    // only 1 retry after 500us to ensure high performance and short reaction times
	nrf.setRetries(500, 1);

    curNode = getFirstNodeID(confNodes);

    //#########TODO
    //## Befehle anpassen und testen, aber zuerst den Empfang von Daten auf der seriellen Schnittstelle prüfen, siehe TODO unten und extra fall für 0xA0
    //## Ändern des seriellen Empfangs mit den neuen Konstanten

    // ######################################################################################################
    // Main loop starts here:
    // Loop through all sensors, receive and forward data, and regularly check if new sensors available
    // ######################################################################################################

    char msg[] = "Echoes back to the screen anything you type\n";
    char *buff = new char[1];
    pc.write(msg, sizeof(msg));

    int cnt{0};
    //int sendCnt{0};                                   //For debugging only
    int dataCnt{0};
    bool add{true};
    bool dataExtracted{false};

    uint8_t commandDuration{0};
    uint16_t commandRecipient{0};
    CommandType commandType = COMMAND_TYPE_UNKNOWN;
    uint8_t cmd = COMMAND_UNKNOWN;
    uint8_t cmdLength{0};
    bool dataAvailable{false};

    //buzzer.write(0.20f);

    curNode = getFirstNodeID(confNodes);

    while (1)
    {

        if (pc.readable()) {    //TODO als zweite einfügen der Macros
            dataAvailable = true;
            while(cnt < 32) {
                if (pc.readable()) {
                    pc.read(buff, sizeof(buff));
                    if (buff[0] == 0xA0 && cnt > 4) { //Das dritte und/oder vierte Byte kann jeweils theoretisch auch 0xA0 sein, wenn der Befehl für die Nodes 13 & 15 und/oder 5 & 7 ist. Damit dann trotzdem der Befehl und eventuelle Daten noch eingelesen werden, dürfen die Daten erst nach dem vierten Byte  "ignoriert" werden, wenn 0xA0 erscheint. // Sync-Bytes (AB CD) + Duration-Byte (00 or 01) + Recipient-Bytes (0x0000 - 0xFFFF => 3tes und 4tes Byte) 
                    //Fehler wenn die SessionID mit dem Wert 160 geupdatet werden soll. erstmal ignorieren ()
                        add = false;
                    }
                    if (add == true) {
                        addToBuffer(buff, dataCnt);
                        dataCnt++;
                    }
                    cnt++;
                }   
            }

            dataExtracted = extractCommandData(serialRXBuffer, dataCnt, commandBuffer, commandDuration, commandRecipient, commandType, cmd, cmdLength);

        }

        if (dataAvailable) {
                
            if (dataExtracted) { //if 255 eigentlich nicht mehr nötig maximal if 0 == basisstation, alles andere ist bitcodiert und muss dann rausgeholt werden
                /*uint8_t receivedCommand[BUFFER_LENGTH] = {0};
                receivedCommand[0] = commandDuration;           //Dauer
                receivedCommand[1] = commandRecipient >> 8;     //Empfänger 1
                receivedCommand[2] = commandRecipient & 0xFF;   //Empfänger 2
                receivedCommand[3] = commandType;               //CommandType
                receivedCommand[4] = cmd;                       //Command
                receivedCommand[5] = cmdLength;                 //CommandLength
                if (cmdLength >= 1) {
                    for (int i = 0; i <cmdLength; i++) {
                        receivedCommand[6+i] = commandBuffer[i];
                    }
                }
                serial_write(receivedCommand, 6+cmdLength);     //Ausgabe des Befehls + eventuellem CommandBuffer
                */
                //serial_write(commandBuffer, dataCnt-7);         //Ausgabe des CommandBuffers
                //buzzer.write(0.20f);        // 20% duty cycle, relative to period
                //ThisThread::sleep_for(100ms);
                //buzzer.write(0.0f);         // 0% duty cycle, relative to period
                
                if (commandRecipient == 0) {                                    //Befehl für die Basisstation
                    if (commandType == COMMAND_TYPE_CHANGE_BS_MODE) {
                        if (cmd == BASESTATION_MODE_GLOVE_MODE) {               //Set Basetation in Glove Mode
                            baseStationMode = BASESTATION_MODE_GLOVE_MODE;      //Set in Glove Modes
                        } else if (cmd == BASESTATION_MODE_BS_MODE) {           //Set Basestation in BS Mode
                            baseStationMode = BASESTATION_MODE_BS_MODE;         //Set in BS Mode
                        }
                    } else {                                                    //Command for the Basestation
                        if (commandDuration == 0) {                             // Save the command for the Basestation and do it permanently
                            if (commandType == COMMAND_TYPE_REQUEST_BS_DATA) {
                                baseStationCmd = cmd;                           //Set the Command for the BS-Mode
                                if (baseStationCmd == REQUEST_BS_DATA_BNO) {    //if Operation Mode is requesting bno data
                                    baseStationBNOMode = commandBuffer[0];      //Set BNO Data Mode
                                }
                            } else if (commandType == COMMAND_TYPE_UPDATE_BS_DATA) {            //Update Cmd
                                if (cmd == UPDATE_BS_DATA_FIND_INACTIVE_NODES) {
                                    if (baseStationMode == 0x00) {                              //Only when the BS is in the glove Mode
                                        uint16_t updateData = commandBuffer[0] << 8 | commandBuffer[1];
                                        findInactiveNodes(confNodes,activNodes, updateData, numNodes);
                                    }                //
                                } else if (cmd == UPDATE_BS_DATA_SESSION_ID) {
                                    sessionID = commandBuffer[0];
                                } else if (cmd == UPDATE_BS_DATA_CONFIGURED_GLOVES) {           //Update ConfNodes
                                    confNodes = commandBuffer[0] << 8 | commandBuffer[1];
                                } else if (cmd == UPDATE_BS_DATA_ACTIVE_GLOVES) {               //Update ActiveNodes
                                    activNodes = commandBuffer[0] << 8 | commandBuffer[1];
                                }
                            }
                        } else {
                            executeBSCommand(bno, stationData, cmd, numNodes, false);   //Executed two times, so that the data will be receieved by the script and get not lost
                            ThisThread::sleep_for(1ms);
                            executeBSCommand(bno, stationData, cmd, numNodes, false);
                        }
                    }

                } else if (commandRecipient != 0 && commandType < 4) {                          //Befehl für die Gloves
                    if (commandDuration == 0) {                                                 //Dauerhaftes setzen der Befehle
                        for (int i = 0; i < MAX_NUM_NODES; i++) {                               //
                            if (commandRecipient & checkMask[i]) {                              //Wenn der Befehl für die Node gesetzt werden soll
                                cmdTypeAndCmd[i][0] = commandType;
                                cmdTypeAndCmd[i][1] = cmd;

                                if (commandType == COMMAND_TYPE_UPDATE_COMMAND) {               //Assume that the Data for the command exact 1 Byte for all Update-Commands
                                    cmdTypeCmdAndData[i][0] = commandType;
                                    cmdTypeCmdAndData[i][1] = cmd;
                                    cmdTypeCmdAndData[i][2] = commandBuffer[0];
                                }
                            }
                        }
                        /*uint8_t tempData[2] = {0};                                              //Fürs debuggen sonst nichts
                        for (int i = 0; i < MAX_NUM_NODES; i++) {
                            tempData[0] = cmdTypeAndCmd[i][0];
                            tempData[1] = cmdTypeAndCmd[i][1];
                            serial_write(tempData, 2);
                        }*/
                    }

                    else if (commandDuration == 1) {                                            //Einmaliges senden von Befehlen
                        uint8_t tempCmdTypeAndCmd[2] = {0};
                        tempCmdTypeAndCmd[0] = commandType;
                        tempCmdTypeAndCmd[1] = cmd;
                        //sendCommand(commandType, cmd, commandBuffer, cmdLength, true, commandRecipient);      //send as broadcast command
                        nrf.flushTX();  //Empty the TX-FIFO, to make sure that the command could be written in den TX-PIPE //Should normally already be empty
                        if (commandType != COMMAND_TYPE_UPDATE_COMMAND) {
                            sendCommand(commandType, cmd, nullptr, 0, false, commandRecipient);                 //send as normal command, but only once
                        } else {
                            sendCommand(commandType, cmd, commandBuffer, cmdLength, false, commandRecipient);   //send as normal command, but only once (updateCommand)
                        }
                        
                        /*for (int i = 0; i < MAX_NUM_NODES; i++) {
                            if (commandRecipient & checkMask[i]) {                              //Wenn der Befehl gesendet werden soll
                                nrf.writeTXData(tempCmdTypeAndCmd, 2);
                            }
                        }*/
                    }
                }

                
                buzzer.write(0.20f);
                ThisThread::sleep_for(50ms);
                buzzer.write(0.0f);

            } else {
                serialRXBuffer[33] = cnt;
                serialRXBuffer[34] = dataCnt;
                pc.write(serialRXBuffer, 35);
            }
            //sendCommand(COMMAND_RestartNode);
            dataAvailable = false;
            dataExtracted = false;
            cnt = 0;
            dataCnt = 0;
            add = true;
            ThisThread::sleep_for(1s);
        }

        //if (baseStationMode == BASESTATION_MODE_GLOVE_MODE) {                                                       //Basestation request and forward glove data   
            if (curNode < 16) {
                if (cmdTypeAndCmd[curNode][0] != COMMAND_TYPE_UPDATE_COMMAND) {      //All command except the Update Command     //UNKNOWN COMMAND Type also handled hear and send even if it doesn't make a change on the node, it will be send
                    nrf.writeTXData(cmdTypeAndCmd[curNode], 2);
                } else {
                    if (cmdTypeAndCmd[curNode][1] == UPDATE_COMMAND_SESSION_ID) {    //Update Command and Update SessionID
                        nrf.writeTXData(cmdTypeCmdAndData[curNode], 3); 
                    }
                }
                
            }                                               
        //}

        //else 
        if (baseStationMode == BASESTATION_MODE_BS_MODE) {                                                    //Basestation send own data
            /*if (baseStationCmd == REQUEST_BS_DATA_NUMBER_OF_NODES) {                                          //Send Number of Nodes
                sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_NUMBER_OF_NODES, &numNodes, 1);
                ThisThread::sleep_for(10ms);
            } else if (baseStationCmd == REQUEST_BS_DATA_SESSION_ID) {                                        //Send SessionID
                sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_SESSION_ID, &sessionID, 1);
                ThisThread::sleep_for(10ms);
            } else if (baseStationCmd == REQUEST_BS_DATA_BNO) {                                               //Send BNO-Data
                readQuatLinAccCompressed(&bno, stationData); //Liest alle 12 Bytes für Quat (6) und LinAcc (6) gibt aber 6 oder 12 aus
                if (baseStationBNOMode == BNO_DATA_QUAT) {                                                          //Nur Quat senden
                    stationData[0] = BNO_DATA_QUAT;
                    sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_BNO, stationData, 7);
                } else if (baseStationBNOMode == BNO_DATA_QUAT_LIN_ACC) {                                           //Quat und LinAcc senden
                    stationData[0] = BNO_DATA_QUAT_LIN_ACC;
                    sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_BNO, stationData, 13);
                }
                ThisThread::sleep_for(10ms);    //100 HZ Frequenz
            } else if (baseStationCmd == REQUEST_BS_DATA_CONFIGURED_NODES) {                                  //Configured Nodes
                sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_CONFIGURED_NODES, &confNodes, 1);
                ThisThread::sleep_for(10ms);
            } else if (baseStationCmd == REQUEST_BS_DATA_ACTIVE_NODES) {                                      //Active Nodes
                sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_ACTIVE_NODES, &activNodes, 1);
                ThisThread::sleep_for(10ms);
            }*/
            executeBSCommand(bno, stationData, baseStationCmd, numNodes);
        }
        


        // get status and reset interrupt flags
        nrf.getIRQStatus(rx, txDone, maxTry);
    	nrf.resetIRQFlags();
        
        rxLen = 0;
        
        if (rx)
        {
			while (nrf.dataAvailable())
			{
				nrf.readRXData(currentBuffer, rxLen, pipe);
				
	        	if (rxLen > 5 && baseStationMode == BASESTATION_MODE_GLOVE_MODE) //Wenn die Daten vom Glove angezeigt werden sollen
	        	{  
                    serial_write(currentBuffer, rxLen); //wieder einfügen
		        	
		        	// change buffer to not accidentially ovwerwrite data while sending via serial port in parallel
		        	if (currentBuffer == RX_buffer_1)
		        	{
		        		currentBuffer = RX_buffer_2;
		        	}
		        	else
		        	{
		        		currentBuffer = RX_buffer_1;
		        	}
                    //alle Daten abfragen mit
                    //nrf.writeTXData(BS_payload_TX + mode, 1); //TODO wieder einfügen und anpassen
                    //Wenn in BS Mode dann prüfen auf sender und fragen nach weiteren daten
                    //wenn nicht in BS mode einfach nach weiteren daten fragen mit 
				} else {
                    break;
                }
			}

		}
		
        else if (txDone || maxTry || curNode > 15) //if ((txDone && rxLen == 0) || maxTry)
		{   
            broadcast_counter++;

            if ((broadcast_counter/loop_pass_for_query) == numNodes) {
                freeNodeID = getNextFreeNodeID(confNodes);
                if (freeNodeID < 16) {
                    do_broadcast(cmdTypeAndCmd[freeNodeID][0], cmdTypeAndCmd[freeNodeID][1], freeNodeID, sessionID, numNodes); 
                    //do_broadcast(0x01, 0x00, freeNodeID, sessionID, numNodes); 
                    //cmdTypeAndCmd[freeNodeID][0] = 0x01;
                    //cmdTypeAndCmd[freeNodeID][1] = 0x00;
                }
                broadcast_counter = 0;
            }

            curNode = getNextNodeID(confNodes, curNode);
            if (curNode < 16) {
                nrf_data_address[4] = curNode;
                nrf.setTXAddress(nrf_data_address, NRF_ADDR_LEN);
                nrf.setRXAddress(0, nrf_data_address, NRF_ADDR_LEN);
            }
		}

        if (dataChanged) {
            if (sessionID < 255) {
                sessionID++;
            } else {
                sessionID = 0;
            }

            sendCommand(COMMAND_TYPE_UPDATE_COMMAND, UPDATE_COMMAND_SESSION_ID, &sessionID, 1, true);      //send as broadcast command to all gloves
            
            sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_SESSION_ID, &sessionID, 1);          //Send SessionID to BS
            ThisThread::sleep_for(1ms);
            sendDataToBS(COMMAND_TYPE_REQUEST_BS_DATA, REQUEST_BS_DATA_NUMBER_OF_NODES, &numNodes, 1);      //Send NUmber of Nodes to BS

            dataChanged = false;
        }

    }
}
