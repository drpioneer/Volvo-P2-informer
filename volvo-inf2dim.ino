/*
 * VOLVO P2 Informer
 * https://github.com/drpioneer/Volvo-P2-informer/
 * Designed to ArduinoNano & 2x MCP2515
 * Code uses ideas and practices of different authors, available in open sources
 * tested on Volvo XC90 2011
 * (c) 2025 drPioneer
 */

#include "mcp_can.h"

// Low speed / high speed CAN-drivers parameters
#define      LS_CAN_SPD   CAN_125KBPS                                                 // LS-CAN settings for Volvo P2 (1999+)
//#define    HS_CAN_SPD   CAN_250KBPS                                                 // HS-CAN settings for Volvo P2 (1999-2004)
#define      HS_CAN_SPD   CAN_500KBPS                                                 // HS-CAN settings for Volvo P2 (2005+)
#define      LS_CAN_QRZ   MCP_8MHZ                                                    // frequency of quartz resonator for LS-CAN driver
#define      HS_CAN_QRZ   MCP_8MHZ                                                    // frequency of quartz resonator for HS-CAN driver
#define      LS_CAN_INT   2                                                           // assigning interrupt pin to LS-CAN receiving buffer
#define      HS_CAN_INT   3                                                           // assigning interrupt pin to HS-CAN receiving buffer
MCP_CAN       LS_CAN_CS   (9);                                                        // using CS-pin for using LS-CAN driver
MCP_CAN       HS_CAN_CS   (10);                                                       // using CS-pin for using HS-CAN driver

// CAN modules identifiers
#define          DEM_ID   0x01204001ul                                                // differential electronic module (2005+)
//#define        ECM_ID   0x00800021ul                                                // engine control module (????-2004)
#define          ECM_ID   0x01200021ul                                                // engine control module (2005+)
#define          REM_ID   0x00800401ul                                                // rear electronic module (2005+)
#define          CCM_ID   0x00801001ul                                                // climate electronic module (2005+)
//#define        TCM_ID   0x00800005ul                                                // transmission control module (1999-2004)
#define          TCM_ID   0x01200005ul                                                // transmission control module (2005+)
//#define        SWM_ID   0x0111300aul                                                // steering wheel module (left lever) (2000-2001)
#define          SWM_ID   0x0131726cul                                                // steering wheel module (left lever) (2005+)
//#define        PHM_ID   0x00400008ul                                                // phone module (2001)
//#define        PHM_ID   0x00C00008ul                                                // phone module (2002)
#define          PHM_ID   0x01800008ul                                                // phone module (2005+)
//#define        SCR_ID   0x00c0200eul                                                // screen on driver information module (2001)
//#define        SCR_ID   0x0220200eul                                                // screen on driver information module (2002)
#define          SCR_ID   0x02a0240eul                                                // screen on driver information module (2005+)
#define          CEL_ID   0x02803008ul                                                // central electronic module LS-CAN (2005+)
#define          CEH_ID   0x03200408ul                                                // central electronic module HS-CAN (2005+)
#define          DIA_ID   0x000ffffeul                                                // diagnostic tool (VIDA)
#define          MSK_ID   0x1ffffffful                                                // mask for 29-bit CAN packets
#define          NON_ID   0x00000000ul                                                // none id
#define    LS_CAN_MASK1   (MSK_ID | SWM_ID | CEL_ID)                                  // mask1 to reduce load of Arduino when listening of LS-CAN
#define    LS_CAN_MASK2   (MSK_ID | CCM_ID | REM_ID)                                  // mask2 to reduce load of Arduino when listening of LS-CAN
#define    HS_CAN_MASK1   (MSK_ID | TCM_ID | ECM_ID)                                  // mask1 to reduce load of Arduino when listening of HS-CAN
#define    HS_CAN_MASK2   (MSK_ID | DEM_ID | CEH_ID)                                  // mask2 to reduce load of Arduino when listening of HS-CAN

#define             EXT   1                                                           // CAN packet parameter: EXTENDED
#define             LEN   8                                                           // CAN packet parameter: LENGTH
#define              MS   30                                                          // delay (in miliseconds)
#define            WAIT   50                                                          // response waiting time
#define           PAUSE   50000                                                       // pause (in cycles)
//#define      INFO_BUT   0xc0                                                        // button 'INFO' code (2000-2001)
#define        INFO_BUT   0xbf                                                        // button 'INFO' code (2005+)
//#define BYTE_INFO_BUT   0x04                                                        // button 'INFO' byte (2000-2001)
#define   BYTE_INFO_BUT   0x07                                                        // button 'INFO' byte (2005+)

/**
 * @brief  Structure describing CAN requests and actions
 */
struct request_t {
               uint32_t   id;                                                         // CAN module id
                uint8_t   cmd[8];                                                     // request command
                uint8_t   form[6];                                                    // parameters to process in formula
                   char   text[16];                                                   // text message
}          static const   req[] PROGMEM = {                                           // table place in PROGram MEMory
//
// ------------------------------------------------------------------------------------------------------------------+
//   id  |                     command                     |          formula parameters         |       text        |
//       |                                                 |   a     b     c     d     e     f   |                   |  
//       |                                    calculated value = ((rxBuf[a] * 2^b + rxBuf[c] / 2^d) / 2^e - f)       |
// ------------------------------------------------------------------------------------------------------------------|
// --- AW55-51 ------------------------------------------------------------------------------------------------------|
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x0c, 0x01, 0x00, 0x00, 0x00}, {0x06, 0x08, 0x07, 0x00, 0x00, 0x00}, "ATF TEMP"        }, // TCM temperature ATF              =  byte[6] * 256 + byte[7]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x06, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x00, 0x00, 0x00}, "S1 SOLENOID"     }, // TCM S1 solenoid status           =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x07, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x00, 0x00, 0x00}, "S2 SOLENOID"     }, // TCM S2 solenoid status           =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x20, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x00, 0x00, 0x00}, "S3 SOLENOID"     }, // TCM S3 solenoid status           =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x21, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x00, 0x00, 0x00}, "S4 SOLENOID"     }, // TCM S4 solenoid status           =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x22, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x00, 0x00, 0x00}, "S5 SOLENOID"     }, // TCM S5 solenoid status           =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0xb2, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x00, 0x00}, "SLT CURR"        }, // TCM SLT solenoid current         =  byte[4] * 256 + byte[5]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0xb3, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x00, 0x00}, "SLS CURR"        }, // TCM SLS solenoid current         =  byte[4] * 256 + byte[5]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0xb4, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x00, 0x00}, "SLU CURR"        }, // TCM SLU solenoid current         =  byte[4] * 256 + byte[5]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x01, 0x01, 0x00, 0x00, 0x00}, {0x05, 0x0e, 0x06, 0x00, 0x0e, 0x00}, "GEARBOX POS"     }, // TCM gearbox position             =  byte[5] & 0x3
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x93, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x00, 0x00}, "RATIO X1000"     }, // TCM gear ratio                   = (byte[4] * 256 + byte[5]) * 0.001
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x15, 0x01, 0x00, 0x00, 0x00}, {0x06, 0x08, 0x07, 0x00, 0x00, 0x00}, "ENG TORQUE"      }, // TCM engine torque                =  byte[6] * 256 + bytes[7]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x15, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x00, 0x00}, "TORQUE REDUC"    }, // TCM torque reduction             =  byte[4] * 256 + bytes[5]
// --- B5254T2 ------------------------------------------------------------------------------------------------------|
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0xb8, 0x01, 0x00, 0x00}, {0x06, 0x00, 0x05, 0x01, 0x00, 0x30}, "COOLANT TMP"     }, // ECM coolant temperature          =  byte[5] * 0.75 - 48
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x05, 0x01, 0x00, 0x00}, {0x05, 0x02, 0x05, 0x00, 0x00, 0x00}, "ATM PRESS"       }, // ECM atmospheric pressure         =  byte[5] * 5
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0xef, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x05, 0x00}, "BOOST PRESS"     }, // ECM boost pressure               = (byte[5] * 256  + byte[6]) / 25.6
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0xaf, 0x01, 0x00, 0x00}, {0x05, 0x01, 0x05, 0x00, 0x02, 0x30}, "INT AIR TMP"     }, // ECM intake air temperature       =  byte[5] * 0.75 - 48
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x01, 0x01, 0x00, 0x00}, {0x05, 0x04, 0x04, 0x00, 0x00, 0xb0}, "AC PRESS"        }, // ECM A/C pressure                 =  byte[5] * 13.54 - 176
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x02, 0x01, 0x00, 0x00}, {0x05, 0x0f, 0x04, 0x00, 0x0f, 0xb0}, "AC COMPRESS ACT" }, // ECM A/C compressor activ         =  byte[5] & 1
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x93, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x02, 0x00}, "ENGINE RPM"      }, // ECM engine speed                 = (byte[5] * 256  + byte[6]) / 4
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x51, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x15, 0x00}, "SHORT TERM FC"   }, // ECM short-term fuel correction   =((byte[5] * 256  + byte[6]) * 2.0) / 65535
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x11, 0x4c, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x04, 0x00}, "LONG TERM FC L"  }, // ECM long-term fuel corr low      = (byte[5] + byte[6]) * 0.046875
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x11, 0x4e, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x15, 0x00}, "LONG TERM FC M"  }, // ECM long-term fuel corr medium   = (byte[5] + byte[6]) * 0.00003052
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x11, 0x50, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x15, 0x00}, "LONG TERM FC H"  }, // ECM long-term fuel corr high     = (byte[5] + byte[6]) * 0.00003052
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x12, 0x48, 0x01, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x08, 0x00, 0x00}, "ENG FAN X100"    }, // ECM Engine fan duty              =  byte[5] * 100 / 255
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0xad, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x00, 0x00}, "MISFIRES CNT"    }, // ECM ignition misfires number     =  byte[5] * 256 + byte[6]
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x15, 0x7d, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x04, 0x45}, "FUEL PRESS"      }, // ECM fuel pressure                = (byte[5] * 256 + byte[6]) * 0.0724792480 - 69
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x15, 0x83, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x09, 0x00}, "FUEL PUM DUT"    }, // ECM fuel pump duty               = (byte[5] * 256 + byte[6]) * 0.0015287890625
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x2d, 0x01, 0x00, 0x00}, {0x05, 0x07, 0x05, 0x00, 0x08, 0x00}, "TCV DUT"         }, // ECM TCV duty                     =  byte[5] * 191.25 / 255
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x4e, 0x01, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x08, 0x00, 0x00}, "ANGL THROT X100" }, // ECM throttle angle               =  byte[5] * 100 / 255
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x9a, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x00, 0x00}, "MAF X10"         }, // ECM air flow meter               = (byte[5] * 256 + byte[6]) * 0.1
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x13, 0x63, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x08, 0x00}, "ANGLE VVT IN"    }, // ECM VVT inlet angle              = (byte[5] * 256 + byte[6]) * 0.0390625
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x13, 0x62, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x08, 0x00}, "ANGLE VVT EXT"   }, // ECM VVT exhaust angle            = (byte[5] * 256 + byte[6]) * 0.0390625
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x2c, 0x01, 0x00, 0x00}, {0x05, 0x07, 0x05, 0x00, 0x08, 0x00}, "BTDC"            }, // ECM BTDC                         = (byte[5] * 191.25 / 255
// --- Haldex -------------------------------------------------------------------------------------------------------|
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x05, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x00, 0x00}, "PUMP CURR"       }, // DEM pump current                 =  byte[5] * 256 + byte[6]
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x05, 0x01, 0x00, 0x00}, {0x07, 0x08, 0x01, 0x00, 0x00, 0x00}, "SOLENOID CURR"   }, // DEM solenoid current             =  byte[7] * 256 + byte[8]
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x03, 0x01, 0x00, 0x00}, {0x06, 0x00, 0x05, 0x06, 0x00, 0x00}, "OIL PRESS"       }, // DEM oil pressure                 =  byte[5] * 0.0164
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x02, 0x01, 0x00, 0x00}, {0x05, 0x00, 0x04, 0x00, 0x00, 0x00}, "OIL TMP"         }, // DEM oil temperature              = (signed char)byte[5]
//{DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x06, 0x01, 0x00, 0x00}, {0x00, 0x00, 0x01, 0x00, 0x06, 0x00}, "FL SPEED"        }, // DEM FL velocity                  = (byte[8] * 256 + bytes[9]) * 0.0156
//{DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x06, 0x01, 0x00, 0x00}, {0x06, 0x00, 0x07, 0x00, 0x06, 0x00}, "FR SPEED"        }, // DEM FR velocity                  = (byte[6] * 256 + bytes[7]) * 0.0156
//{DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x06, 0x01, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x00, 0x06, 0x00}, "RL SPEED"        }, // DEM RL velocity                  = (byte[12] * 256 + bytes[13]) * 0.0156
//{DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x06, 0x01, 0x00, 0x00}, {0x02, 0x00, 0x03, 0x00, 0x06, 0x00}, "RR SPEED"        }, // DEM RR velocity                  = (byte[10] * 256 + bytes[11]) * 0.0156
// --- Rear unit ----------------------------------------------------------------------------------------------------|
  {REM_ID, {0xcd, 0x46, 0xa6, 0xd0, 0xd4, 0x01, 0x00, 0x00}, {0x05, 0x00, 0x06, 0x00, 0x03, 0x00}, "BATT VOLTAGE"    }, // REM battery voltage              =  byte[5] / 8
// --- Climate unit -------------------------------------------------------------------------------------------------|
  {CCM_ID, {0xcd, 0x29, 0xa6, 0x00, 0x01, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x06, 0x64}, "EVAPORAT TMP"    }, // CCM evaporator temperature       =  byte[5] * 256 + byte[6]) * 0.015625 - 100
  {CCM_ID, {0xcd, 0x29, 0xa6, 0x00, 0xa1, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x06, 0x64}, "CABIN TMP"       }, // CCM cabin temperature            =  byte[5] * 256 + byte[6]) * 0.015625 - 100
  {CCM_ID, {0xcd, 0x29, 0xa6, 0x00, 0x30, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x06, 0x00}, "CABIN FAN SPD"   }, // CCM cabin fan speed              =  byte[5] * 256 + byte[6]) * 0.015625
// ------------------------------------------------------------------------------------------------------------------+
};

// LS/HS-CAN Rx buffers
uint32_t        CANrxId = 0;                                                          // header of LS-CAN packet for receiving
uint8_t     CANrxBuf[8] = {0};                                                        // buffer for receiving a packet from LS-CAN
uint8_t        CANrxLen = 8;                                                          // length of receiving buffer for LS-CAN
uint8_t          CANext = 1;                                                          // extended attribute of LS-CAN packet (29 bit)

// Main variables
uint8_t         SCREENS = sizeof(req) / sizeof(request_t);                            // active screens number
uint8_t       actScreen = 0;                                                          // index of current screen
uint32_t      progCycle = 0;                                                          // cycle counter for determining elapsed time
uint32_t             id = 0;                                                          // CAN module id
uint8_t          cmd[8] = {0};                                                        // request command
uint8_t         form[6] = {0};                                                        // parameters to process in formula
char           text[16] = {0};                                                        // text message
uint32_t     LONG_PRESS = 1000;                                                       // long press duration
uint32_t     pressStart = 0;                                                          // button press time
boolean       isPressed = false;                                                      // button current status
boolean          readIt = false;                                                      // request response status
boolean       screenEna = true;

/**
 * @brief   Initializing LS-CAN driver
 * @param   none
 * @retval  bool:  true | false
 */
boolean SetupLSCAN() {
  Serial.println("LS-CAN driver initialize...");
  pinMode(LS_CAN_INT, INPUT_PULLUP);                                                  // set up INT Pin for LS-CAN
  if (LS_CAN_CS.begin(MCP_ANY, LS_CAN_SPD, LS_CAN_QRZ) == CAN_OK)                     // init LS-CAN bus
    if (LS_CAN_CS.init_Mask(0, EXT, MSK_ID & LS_CAN_MASK1) == MCP2515_OK)             // config LS-CAN masks and filters
      if (LS_CAN_CS.init_Mask(1, EXT, MSK_ID & LS_CAN_MASK2) == MCP2515_OK)
        if (LS_CAN_CS.init_Filt(0, EXT, MSK_ID &       SWM_ID) == MCP2515_OK)
          if (LS_CAN_CS.init_Filt(1, EXT, MSK_ID &       CEL_ID) == MCP2515_OK)
            if (LS_CAN_CS.init_Filt(2, EXT, MSK_ID &       REM_ID) == MCP2515_OK)
              if (LS_CAN_CS.init_Filt(3, EXT, MSK_ID &       CCM_ID) == MCP2515_OK)
                if (LS_CAN_CS.init_Filt(4, EXT, MSK_ID &       NON_ID) == MCP2515_OK)
                  if (LS_CAN_CS.init_Filt(5, EXT, MSK_ID &       NON_ID) == MCP2515_OK) {
                    LS_CAN_CS.setSleepWakeup(1);
                    LS_CAN_CS.setMode(MCP_NORMAL);                                    // set up LS-CAN in normal mode
                    Serial.println("LS-CAN driver initialized successfully!");
                    return true;
                  };
  Serial.println("LS-CAN driver initialized failed!");
  return false;
}

/**
 * @brief   Initializing HS-CAN driver
 * @param   none
 * @retval  bool:  true | false
 */
boolean SetupHSCAN() {
  Serial.println("HS-CAN BUS driver initialize...");
  pinMode(HS_CAN_INT, INPUT_PULLUP);                                                  // set up INT Pin for HS-CAN
  if (HS_CAN_CS.begin(MCP_ANY, HS_CAN_SPD, HS_CAN_QRZ) == CAN_OK)                     // init HS-CAN bus
    if (HS_CAN_CS.init_Mask(0, EXT, MSK_ID & HS_CAN_MASK1) == MCP2515_OK)             // config HS-CAN masks and filters
      if (HS_CAN_CS.init_Mask(1, EXT, MSK_ID & HS_CAN_MASK2) == MCP2515_OK) 
        if (HS_CAN_CS.init_Filt(0, EXT, MSK_ID &       TCM_ID) == MCP2515_OK) 
          if (HS_CAN_CS.init_Filt(1, EXT, MSK_ID &       ECM_ID) == MCP2515_OK) 
            if (HS_CAN_CS.init_Filt(2, EXT, MSK_ID &       DEM_ID) == MCP2515_OK) 
              if (HS_CAN_CS.init_Filt(3, EXT, MSK_ID &       CEH_ID) == MCP2515_OK) 
                if (HS_CAN_CS.init_Filt(4, EXT, MSK_ID &       NON_ID) == MCP2515_OK) 
                  if (HS_CAN_CS.init_Filt(5, EXT, MSK_ID &       NON_ID) == MCP2515_OK) {
                    HS_CAN_CS.setSleepWakeup(1);
                    HS_CAN_CS.setMode(MCP_NORMAL);                                    // set up HS-CAN in normal mode
                    Serial.println("HS-CAN driver initialized successfully!");
                    return true;
                  };
  Serial.println("HS-CAN driver initialized failed!");
  return false;
}

/**
 * @brief   Stopping work in case of equipment malfunction
 * @param   none
 * @retval  none
 */
void HardFault() {
  Serial.println("!!! ATENTION!!! ABNORMAL TERMINATION !!! CHECK EQUIPMENT !!!");
  SPI.end();
  DDRB |= _BV(DDB5);                                                                  // force-set PB5 (pin 13) as an output
  PORTB &= ~_BV(PORTB5);                                                              // LED turn off
  while (1) {
    PINB = _BV(PINB5);                                                                // LED toggle via PIN register
    delay(500);
  };
}

/**
 * @brief   Turn on screen
 * @param   none
 * @retval  none
 */
void EnableScreen() {
  //uint8_t DIM_screen_enable[2][8] = {                                               // command to turn on screen, 2000-2001
  //  {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05},
  //  {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  //};
  uint8_t DIM_screen_enable[2][8] = {                                                 // command to turn on screen, 2005+ or 2003-2004
    {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35},
    {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31},
  };
  for (uint8_t i = 0; i < 2; i++) {
    LS_CAN_CS.sendMsgBuf(SCR_ID, EXT, LEN, DIM_screen_enable[i]);
    delay(MS);
  };
}

/**
 * @brief   Turn off screen
 * @param   none
 * @retval  none
 */
void DisableScreen() {
  uint8_t DIM_screen_disable[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};   // command to turn off screen
  LS_CAN_CS.sendMsgBuf(SCR_ID, EXT, LEN, DIM_screen_disable);
  delay(MS);
}

/**
 * @brief   Clear screen
 * @param   none
 * @retval  none
 */
void ClearScreen() {
  uint8_t DIM_screen_clear[8] = {0xe1, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};     // command to clear screen
  LS_CAN_CS.sendMsgBuf(SCR_ID, EXT, LEN, DIM_screen_clear);
  delay(MS);
}

/**
 * @brief   Output message
 * @param   value - numeric value (may be negative)
 * @param   label - text label
 * @retval  none
 */
void PrintScreen(int16_t value, const char* label) {
  static char buffer[33];                                                           // 32 symbols + '\0'
  int len = len = snprintf(buffer, sizeof(buffer), "%s %d", label, value);          // forming a string
  if (len < (sizeof (buffer) - 1)) {
    for (int i = len; i < (sizeof (buffer) - 1); i++)  buffer[i] = ' ';             // filling voids via spaces
  buffer[(sizeof (buffer) - 1)] = '\0';
  };
  uint8_t msg[5][8] = {
    {0xa7,       0x00, buffer[ 0], buffer[ 1], buffer[ 2], buffer[ 3], buffer[ 4], buffer[ 5]},
    {0x21, buffer[ 6], buffer[ 7], buffer[ 8], buffer[ 9], buffer[10], buffer[11], buffer[12]},
    {0x22, buffer[13], buffer[14], buffer[15], buffer[16], buffer[17], buffer[18], buffer[19]},
    {0x23, buffer[20], buffer[21], buffer[22], buffer[23], buffer[24], buffer[25], buffer[26]},
    {0x65, buffer[27], buffer[28], buffer[29], buffer[30], buffer[31],       0x00,       0x00},
  };
  Serial.println(buffer);
  for (uint8_t i = 0; i < (sizeof(msg) >> 3); i++) {
    LS_CAN_CS.sendMsgBuf(PHM_ID, EXT, LEN, msg[i]);
    delay(MS);
  };
}

/**
 * @brief   Output message
 * @param   label - text label
 * @retval  none
 */
void PrintScreen(const char* label) {
  static char buffer[33];                                                           // 32 symbols + '\0'
  int len = len = snprintf(buffer, sizeof(buffer), "%s", label);                    // forming a string
  if (len < (sizeof (buffer) - 1)) {
    for (int i = len; i < (sizeof (buffer) - 1); i++)  buffer[i] = ' ';             // filling voids via spaces
  buffer[(sizeof (buffer) - 1)] = '\0';
  };
  uint8_t msg[5][8] = {
    {0xa7,       0x00, buffer[ 0], buffer[ 1], buffer[ 2], buffer[ 3], buffer[ 4], buffer[ 5]},
    {0x21, buffer[ 6], buffer[ 7], buffer[ 8], buffer[ 9], buffer[10], buffer[11], buffer[12]},
    {0x22, buffer[13], buffer[14], buffer[15], buffer[16], buffer[17], buffer[18], buffer[19]},
    {0x23, buffer[20], buffer[21], buffer[22], buffer[23], buffer[24], buffer[25], buffer[26]},
    {0x65, buffer[27], buffer[28], buffer[29], buffer[30], buffer[31],       0x00,       0x00},
  };
  Serial.println(buffer);
  for (uint8_t i = 0; i < (sizeof(msg) >> 3); i++) {
    LS_CAN_CS.sendMsgBuf(PHM_ID, EXT, LEN, msg[i]);
    delay(MS);
  };
}

/**
 * @brief   Hardware setup
 * @param   none
 * @retval  none
 */
void setup() 
{
  Serial.begin(115200);
  if (!SetupLSCAN() or !SetupHSCAN())   HardFault();
  Serial.print("Total active screens: ");
  Serial.println(SCREENS, DEC);
  EnableScreen();                                                                     // screen turn on
  ClearScreen();
  PrintScreen("*   VOLVO P2   **   INFORMER   *");
  delay(1000);
}

/**
 * @brief   Button handler
 * @param   buf - 
 * @retval  none
 */
void HandlerSWM(uint8_t *buf) {
  if (buf[BYTE_INFO_BUT] == INFO_BUT) {                                               // when 'INFO/RESET' button on SWM is pressed
    if (!isPressed) {
      isPressed = true;
      pressStart = millis();
      Serial.println("Button pressed");
    };
    if (isPressed && (millis() - pressStart >= LONG_PRESS)) {                         // long button press
      progCycle = PAUSE;
      isPressed = false;
      Serial.println("Long press detected");
      if (screenEna) {
        screenEna = false;
      } else {
        screenEna = true;
      };
    };
  } else {
    if (isPressed) {
      if (millis() - pressStart < LONG_PRESS) {                                       // short button press
        actScreen++;
        if (actScreen >= SCREENS)   actScreen = 0;
        readIt = false;
        progCycle = PAUSE;
        Serial.println("Short press");
      };
      isPressed = false;
    };
  };
}

/**
 * @brief   CEM-L handler
 * @param   buf
 * @retval  none
 */
void HandlerCEL(uint8_t *buf) {
  Serial.print("Ambient light: ");
  uint8_t val = buf[4] & 0xf;
  Serial.println(val, DEC);
}

/**
 * @brief   LS-CAN packet reader
 * @param   none
 * @retval  none
 */
void ReadLSCAN()
{
  while (LS_CAN_CS.checkReceive() == CAN_MSGAVAIL) {                                  // checking periodic messages of LS-CAN
    LS_CAN_CS.readMsgBuf(&CANrxId, &CANext, &CANrxLen, CANrxBuf);
    switch (CANrxId & MSK_ID) {
      case SWM_ID:
        HandlerSWM (CANrxBuf);
        break;

      case CEL_ID:
        //HandlerCEL (CANrxBuf);
        break;

      default:
        break;
    };
  };
}

/**
 * @brief   CAN ID define purpose
 * @param   id
 * @retval  0 | 1 | 2
 */
uint8_t PurposeCAN(uint32_t id) {
  static const uint32_t hsCanId [] = {CEH_ID, ECM_ID, TCM_ID, DEM_ID};
  static const uint32_t lsCanId [] = {CEL_ID, CCM_ID, SWM_ID, PHM_ID, REM_ID};
  for (uint8_t i = 0; i < (sizeof(hsCanId) / sizeof(uint32_t)); i++)
    if (hsCanId[i] == id)   return 1;                                                 // packet for HS-CAN
  for (uint8_t i = 0; i < (sizeof(lsCanId) / sizeof(uint32_t)); i++)
    if (lsCanId[i] == id)   return 2;                                                 // packet for LS-CAN
  return 0;                                                                           // packet not for CAN
}

void loop() {
  ReadLSCAN();                                                                        // listening LS-CAN for button pressing
  progCycle++;                                                                        // increasing delay counter before a new parameter request in CAN
  if (actScreen >= SCREENS) actScreen = 0;
  if (progCycle >  PAUSE)   progCycle = 0;
  if (progCycle == PAUSE) {                                                           // every 50 000 cycles (conditional 1 seс)

    id = pgm_read_dword(&req[actScreen].id);                                          // buffer filling with current id
    for (uint8_t i = 0; i < 8; i++)
      cmd[i] = pgm_read_byte(&req[actScreen].cmd[i]);                                 // buffer filling with current command
    for (uint8_t i = 0; i < 6; i++)
      form[i] = pgm_read_byte(&req[actScreen].form[i]);                               // buffer filling with current formula parameters
    for (uint8_t i = 0; i < 16; i++)
      text[i] = pgm_read_byte(&req[actScreen].text[i]);                               // buffer filling with current text

    switch (actScreen) {                                                              // switch according to active screen

      default:
        Serial.print("Case: ");
        Serial.println(actScreen, DEC);

        if (!readIt) {
          if (screenEna) {
            EnableScreen();                                                           // screen turn on
            ClearScreen();
            PrintScreen(text);
            } else {
              DisableScreen();                                                        // screen turn off
              delay (1000);
            };
          };

        // ***** request-response via HS-CAN *****
        if (PurposeCAN(id) == 1) {                                                  // action with HS-CAN
          HS_CAN_CS.sendMsgBuf(DIA_ID, EXT, LEN, cmd);                              // sending request command
          uint32_t timeout = millis() + WAIT;                                       // response waiting time
          while (millis() < timeout) {
            if (HS_CAN_CS.checkReceive() == CAN_MSGAVAIL) {                         // checking reception of data from HS-CAN while waiting for response
              HS_CAN_CS.readMsgBuf(&CANrxId, &CANext, &CANrxLen, CANrxBuf);         // reading incoming packet on HS-CAN driver
              if (id == (MSK_ID & CANrxId)) {                                       // checking packet ID
                Serial.print("Found in HS-CAN required ID: ");
                Serial.println(CANrxId, HEX);
                int16_t val = ((((uint16_t)CANrxBuf[form[0]] << form[1]) + (CANrxBuf[form[2]] >> form[3])) >> form[4]) - form[5];
                PrintScreen(val, text);                                             // print 'value + text'
                readIt = true;                                                      // request response status
                break;
              }; // if (CANrxId...
            };
          }; // while (checkReceive...
        }; // if (PurposeCAN...

        // ***** request-response via LS-CAN *****
        if (PurposeCAN(id) == 2) {                                                  // action with LS-CAN
          LS_CAN_CS.sendMsgBuf(DIA_ID, EXT, LEN, cmd);                              // sending request command
          uint32_t timeout = millis() + WAIT;                                       // response waiting time
          while (millis() < timeout) {
            if (LS_CAN_CS.checkReceive() == CAN_MSGAVAIL) {                         // checking reception of data from HS-CAN while waiting for response
              LS_CAN_CS.readMsgBuf(&CANrxId, &CANext, &CANrxLen, CANrxBuf);         // reading incoming packet on LS-CAN driver
              if (id == (MSK_ID & CANrxId)) {                                       // checking packet ID
                Serial.print("Found in LS-CAN required ID: ");
                Serial.println(CANrxId, HEX);
                int16_t val = ((((uint16_t)CANrxBuf[form[0]] << form[1]) + (CANrxBuf[form[2]] >> form[3])) >> form[4]) - form[5];
                PrintScreen(val, text);                                             // print 'value + text'
                readIt = true;                                                      // request response status
                break;
              }; // if (CANrxId...
            };
          }; // while (checkReceive...
        }; // if (PurposeCAN...

        // ***** NO CAN *****
        if (PurposeCAN(id) == 0) { }                                                // action with NO CAN

        progCycle = 0;
        break; // case default
    };
  }; // switch (actScreen...
}
