#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include <cstdint>

/*****NRF_CONSTANTS*****/
#define PAYLOAD_MAX_LEN     32 //Darf nicht größer als 32 sein
#define TX_PAYLOAD_LEN      10 //Darf nicht größer als 32 sein

#define NRF_ADDR_LEN        5  //Darf nur die Werte 3, 4 oder 5 annehmen

/***** *****/
#define MAX_NUM_NODES       16 //Darf 0 - 16 sein, ansonsten muss der Code abgeändert werden.

/***** *****/
//#define MAX_REQUESTS        5 TODO herausfinden ob wirklich ungenutzt

/*****SERIAL_CONSTANTS*****/
#define SERIAL_COMMAND_LENGTH   7           //Anzahl der Daten in Byte, die zu einem Command gehören (Sync {2 Bytes} + Empfänger {2 Bytes} + Befehlstype {1 Byte} + Befehl {1 Byte} + EndByte {1 Byte} = 7 Byte)

#define SERIAL_DATA_LENGTH  25              //Maximale Länge der Daten die zu einem Command gehören können (Kann nicht größer wie 30 Byte sein, wenn die Daten an einen Glove gesendet werden sollen, da zwei Byte für CommandType und Command genutz werden, die zwei Bytes + 30 sind 32 Bytes, also die Maximale Payload-Size einer TX/RX-FIFO)

#define SERIAL_INPUT_LENGTH COMMAND_INFORMATION_LENGTH+SERIAL_DATA_LENGTH //Anzahl der Bytes aus denen ein Command bestehen muss!!! Alle nicht benötigten Daten nach dem Byte 0x0A werden verweorfen. Müssen aber vorhanden sein, dass auch x entsprechende Byte aus dem Buffer gelesen werden

#define BUFFER_LENGTH       36              //Für das senden von Befehlen reicht eine Länge von 36 Byte 
                                            // (2 Sync- + 2 Empänger- + 1 Befehlstype- + 1 Befehls- + 30 Daten Bytes  => 
                                            // Befehl = 1 Byte das ein Befehl ausgeführt werden soll, 1 Byte Befehl + bis zu 30 Byte Daten)
                                            // NRF TX-FIFO bis 32 Byte = Befehlslänge, anders muss der Befehl aufgeteilt werden.

#if BUFFER_LENGTH < SERIAL_INPUT_LENGTH         //Um sicherzustellen, dass auch alle Daten im Buffer gespeichert werden können
    #define BUFFER_LENGTH SERIAL_INPUT_LENGTH
#endif                                            

#define COMMAND_DELIMITER   0xA0    //Zeigt an dass ein Command zu ende ist, auch wenn noch weitere Bytes folgen

/***** *****/
#define COMMAND_UNKNOWN     255     //Wert für unbekannte commands


/*****COMMAND_ENUMS*****/
typedef enum {
    COMMAND_TYPE_GENERAL_COMMAND,           // 0
    COMMAND_TYPE_MODE_COMMAND,              // 1
    COMMAND_TYPE_INFO_COMMAND,              // 2
    COMMAND_TYPE_UPDATE_COMMAND,            // 3
    COMMAND_TYPE_SCRIPT_COMMAND = 127,      // 127                          //0x7F  //
    COMMAND_TYPE_CHANGE_BS_MODE = 128,      // 128  //Ändern des BS-Modus   //0x80
    COMMAND_TYPE_UPDATE_BS_DATA = 253,      // 253  //BS CMD                //0xFD
    COMMAND_TYPE_REQUEST_BS_DATA = 254,     // 254  //BS CMD                //0xFE
    COMMAND_TYPE_UNKNOWN                    // 255                          //0xFF
} CommandType;

typedef enum {                              //schauen ob die werte noch stimmen
    GENERAL_COMMAND_QUERY,                  // 0
    GENERAL_COMMAND_IDLE,                   // 1
    GENERAL_COMMAND_RESTART,                // 2
    GENERAL_COMMAND_RECOVER_GLOVE,          // 3
    GENERAL_COMMAND_CONFIGURE_GLOVE,        // 4 //TODO später noch nutzen im Broadcast
    GENERAL_COMMAND_SHOW_ID,                // 5
    GENERAL_COMMAND_UNKNOWN = 255           // 255
} GeneralCommand;                           // CommandType 0

typedef enum {
    MODE_COMMAND_QUAT,                      // 0
    MODE_COMMAND_QUAT_LIN_ACC,              // 1
    MODE_COMMAND_UNKNOWN = 255,             // 255
} ModeCommand;                              // CommandType 1

typedef enum {
    INFO_COMMAND_CALIBRATION_DATA,          // 0
    INFO_COMMAND_GLOVE_CONFIG,              // 1
    INFO_COMMAND_UNKNOWN = 255              // 255
} InfoCommand;                              // CommandType 2

typedef enum {
    UPDATE_COMMAND_SESSION_ID,              // 0
    UPDATE_COMMAND_UNKNOWN = 255            // 255
} UpdateCommand;                            // CommandType 3

typedef enum {
    BASESTATION_MODE_GLOVE_MODE,            // 0 //Get Data von Glove
    BASESTATION_MODE_BS_MODE,               // 1 //Get Data von BS
    BASESTATION_MODE_UNKNOWN = 255          //
} BasestationMode;

typedef enum {
    UPDATE_BS_DATA_SESSION_ID,              // 0
    UPDATE_BS_DATA_CONFIGURED_GLOVES,       // 1
    UPDATE_BS_DATA_ACTIVE_GLOVES,           // 2
    UPDATE_BS_DATA_FIND_INACTIVE_NODES,     // 3 Used to first update the active Nodes and then the configured nodes
    UPDATE_BS_DATA_UNKNOWN = 255
} UpdateBsData;

typedef enum {
    REQUEST_BS_DATA_NUMBER_OF_NODES,        // 0
    REQUEST_BS_DATA_SESSION_ID,             // 1
    REQUEST_BS_DATA_BNO,                    // 2
    REQUEST_BS_DATA_CONFIGURED_NODES,       // 3
    REQUEST_BS_DATA_ACTIVE_NODES,           // 4
    REQUEST_BS_DATA_UNKNOWN = 255
} RequestBsData;    //BasestationDatatype

typedef enum {
    BNO_DATA_QUAT,                          // 0
    BNO_DATA_QUAT_LIN_ACC,                  // 1
} BasestationBnoData;

typedef enum {
    SCIRPT_COMMAND_FIND_INACTIVE_NODES,     //0
} ScriptCommand;

/*
typedef enum {
	PACKAGE_Zero,					        // 0
	PACKAGE_One,					        // 1
	PACKAGE_Two,					        // 2
	PACKAGE_Three,					        // 3
} Package;*/

/*
enum Pipe { //Anpassen an BS //not necessary
	PIPE_Data = 0,                          //
	PIPE_Broadcast = 0,                     //
};
*/

const static uint16_t bitmasksSet[16] = {0x0001, 0x0002, 0x0004, 0x0008, 
                                         0x0010, 0x0020, 0x0040, 0x0080,
                                         0x0100, 0x0200, 0x0400, 0x0800,
                                         0x1000, 0x2000, 0x4000, 0x8000};

const static uint16_t *checkMask = bitmasksSet;
                                         
const static uint16_t bitmasksClear[16] = {   0xFFFE, 0xFFFD, 0xFFFB, 0xFFF7, 
                                                0xFFEF, 0xFFDF, 0xFFBF, 0xFF7F,
                                                0xFEFF, 0xFDFF, 0xFBFF, 0xF7FF,
                                                0xEFFF, 0xDFFF, 0xBFFF, 0x7FFF}; 

#endif /* __CONSTANTS_H__ */