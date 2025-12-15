/*
 * VOLVO P2 Informer
 * https://github.com/drpioneer/Volvo-P2-informer/
 * https://www.drive2.com/l/718050088366114704/
 * Designed to ArduinoNano & 2x MCP2515
 * Code uses ideas and practices of different authors, available in open sources
 * tested on Volvo XC90 2011
 * (c) 2025 drPioneer
 */

#include "mcp_can.h"

#define            LS_CAN_QRZ   MCP_8MHZ                                              // frequency of quartz resonator for LS-CAN driver
#define            HS_CAN_QRZ   MCP_8MHZ                                              // frequency of quartz resonator for HS-CAN driver
#define            LS_CAN_INT   2                                                     // assigning interrupt pin to LS-CAN receiving buffer
#define            HS_CAN_INT   3                                                     // assigning interrupt pin to HS-CAN receiving buffer
MCP_CAN             LS_CAN_CS   (9);                                                  // using CS-pin for using LS-CAN driver
MCP_CAN             HS_CAN_CS   (10);                                                 // using CS-pin for using HS-CAN driver
#define                NON_ID   0x00000000ul                                          // none module id
#define                DEM_ID   0x01204001ul                                          // differential electronic module (2005+)
//#define              ECM_ID   0x00800021ul                                          // engine control module (????-2004)
#define                ECM_ID   0x01200021ul                                          // engine control module (2005+)
#define                REM_ID   0x00800401ul                                          // rear electronic module (2005+)
#define                CCM_ID   0x00801001ul                                          // climate electronic module (2005+)
//#define              TCM_ID   0x00800005ul                                          // transmission control module (1999-2004)
#define                TCM_ID   0x01200005ul                                          // transmission control module (2005+)
//#define              SWM_ID   0x0111300aul                                          // steering wheel module (left lever) (2000-2001)
#define                SWM_ID   0x0131726cul                                          // steering wheel module (left lever) (2005+)
//#define              PHM_ID   0x00400008ul                                          // phone module (2001)
//#define              PHM_ID   0x00C00008ul                                          // phone module (2002)
#define                PHM_ID   0x01800008ul                                          // phone module (2005+)
//#define              LCD_ID   0x00c0200eul                                          // LCD on driver information module (2001)
//#define              LCD_ID   0x0220200eul                                          // LCD on driver information module (2002)
#define                LCD_ID   0x02a0240eul                                          // LCD on driver information module (2005+)
#define                CEL_ID   0x02803008ul                                          // central electronic module LS-CAN (2005+)
#define                CEH_ID   0x03200408ul                                          // central electronic module HS-CAN (2005+)
#define                DIA_ID   0x000ffffeul                                          // diagnostic tool (VIDA)
#define                MSK_ID   0x1ffffffful                                          // mask for 29-bit CAN packets
#define                MSK_L1   (SWM_ID | CEL_ID)                                     // mask1 to reduce load of Arduino when listening of LS-CAN
#define                MSK_L2   (CCM_ID | REM_ID)                                     // mask2 to reduce load of Arduino when listening of LS-CAN
#define                MSK_H1   (TCM_ID | ECM_ID)                                     // mask1 to reduce load of Arduino when listening of HS-CAN
#define                MSK_H2   (DEM_ID | CEH_ID)                                     // mask2 to reduce load of Arduino when listening of HS-CAN
//#define         LCD_ENABLE1   {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05}      // command_1 to turn on LCD, 2000-2001
//#define         LCD_ENABLE2   {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}      // command_2 to turn on LCD, 2000-2001
#define           LCD_ENABLE1   {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35}      // command_1 to turn on LCD, 2005+ or 2003-2004
#define           LCD_ENABLE2   {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31}      // command_2 to turn on LCD, 2005+ or 2003-2004
#define           LCD_DISABLE   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04}      // command to turn off LCD
#define             LCD_CLEAR   {0xe1, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}      // command to clear LCD
//#define            INFO_BUT   0xc0                                                  // button 'INFO' code (2000-2001)
#define              INFO_BUT   0xbf                                                  // button 'INFO' code (2005+)
//#define       BYTE_INFO_BUT   0x04                                                  // button 'INFO' byte (2000-2001)
#define         BYTE_INFO_BUT   0x07                                                  // button 'INFO' byte (2005+)
#define                   STD   0                                                     // CAN packet parameter: STANDART
#define                   EXT   1                                                     // CAN packet parameter: EXTENDED
#define                   LEN   8                                                     // CAN packet parameter: LENGTH
#define                    MS   30                                                    // delay between requests (ms)
#define                  WAIT   50                                                    // response waiting time (ms)
#define                 POLLS   1000                                                  // pause between polls (ms)

/**
 * @brief  Structure describing CAN requests and actions
 */
struct request_t {
                     uint32_t   id;                                                   // CAN module id
                      uint8_t   cmd[8];                                               // request command
                      uint8_t   form[6];                                              // parameters to process in formula
                         char   text[16];                                             // text message
}                static const   req[] PROGMEM = {                                     // table place in PROGram MEMory
//
// ------------------------------------------------------------------------------------------------------------------+
//   id  |                     command                     |          formula parameters         |       text        |
//       |                                                 |   a     b     c     d     e     f   |                   |  
//       |                                    calculated value = ((rxBuf[a] * 2^b + rxBuf[c] / 2^d) / 2^e - f)       |
// ------------------------------------------------------------------------------------------------------------------|
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0xb8, 0x01, 0x00, 0x00}, {0x05, 0x01, 0x05, 0x00, 0x02, 0x30}, "COOLANT TEMP"    }, // ECM_coolant temperature        =  byte[5] * 0.75 - 48
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x0c, 0x01, 0x00, 0x00, 0x00}, {0x06, 0x08, 0x07, 0x00, 0x00, 0x00}, "ATF TEMP"        }, // TCM_temperature ATF            =  byte[6] * 256 + byte[7]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x06, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x08, 0x00, 0x00}, "S1 SLND"         }, // TCM_S1 solenoid status         =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x07, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x08, 0x00, 0x00}, "S2 SLND"         }, // TCM_S2 solenoid status         =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x20, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x08, 0x00, 0x00}, "S3 SLND"         }, // TCM_S3 solenoid status         =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x21, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x08, 0x00, 0x00}, "S4 SLND"         }, // TCM_S4 solenoid status         =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x22, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x00, 0x05, 0x08, 0x00, 0x00}, "S5 SLND"         }, // TCM_S5 solenoid status         =  byte[4]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0xb2, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x00, 0x00}, "SLT CURR"        }, // TCM_SLT solenoid current       =  byte[4] * 256 + byte[5]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0xb3, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x00, 0x00}, "SLS CURR"        }, // TCM_SLS solenoid current       =  byte[4] * 256 + byte[5]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0xb4, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x00, 0x00}, "SLU CURR"        }, // TCM_SLU solenoid current       =  byte[4] * 256 + byte[5]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x01, 0x01, 0x00, 0x00, 0x00}, {0x05, 0x0e, 0x06, 0x08, 0x0e, 0x00}, "GEARBOX POS"     }, // TCM_gearbox position           =  byte[5] & 0x3
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x93, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x0a, 0x00}, "GEAR RATIO"      }, // TCM gear ratio                 = (byte[4] * 256 + byte[5]) * 0.001
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x15, 0x01, 0x00, 0x00, 0x00}, {0x06, 0x08, 0x07, 0x00, 0x00, 0x00}, "ENG TORQ"        }, // TCM_engine torque              =  byte[6] * 256 + bytes[7]
  {TCM_ID, {0xcc, 0x6e, 0xa5, 0x15, 0x01, 0x00, 0x00, 0x00}, {0x04, 0x08, 0x05, 0x00, 0x00, 0x00}, "TORQ REDUCT"     }, // TCM_torque reduction           =  byte[4] * 256 + bytes[5]
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x05, 0x01, 0x00, 0x00}, {0x05, 0x02, 0x05, 0x00, 0x00, 0x00}, "ATMO PRESS"      }, // ECM_atmospheric pressure       =  byte[5] * 5
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0xef, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x08, 0x00}, "BOOST *0.1"      }, // ECM boost pressure             = (byte[5] * 256  + byte[6]) / 25.6
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0xaf, 0x01, 0x00, 0x00}, {0x05, 0x01, 0x05, 0x00, 0x02, 0x30}, "INT AIR TEMP"    }, // ECM_intake air temperature     =  byte[5] * 0.75 - 48
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x01, 0x01, 0x00, 0x00}, {0x05, 0x04, 0x04, 0x00, 0x00, 0xb0}, "AC PRESS"        }, // ECM A/C pressure               =  byte[5] * 13.54 - 176
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x02, 0x01, 0x00, 0x00}, {0x05, 0x0f, 0x04, 0x00, 0x0f, 0x00}, "COMPRESSOR ACT"  }, // ECM_A/C compressor activ       =  byte[5] & 1
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x93, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x02, 0x00}, "ENGINE RPM"      }, // ECM_engine speed               = (byte[5] * 256  + byte[6]) / 4
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x51, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x15, 0x00}, "SHRT-TERM FC"    }, // ECM short-term fuel correction =((byte[5] * 256  + byte[6]) * 2.0) / 65535
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x11, 0x4c, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x04, 0x00}, "LONG-TERM FC L"  }, // ECM long-term fuel corr low    = (byte[5] + byte[6]) * 0.046875
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x11, 0x4e, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x15, 0x00}, "LONG-TERM FC M"  }, // ECM long-term fuel corr medium = (byte[5] + byte[6]) * 0.00003052
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x11, 0x50, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x15, 0x00}, "LONG-TERM FC H"  }, // ECM long-term fuel corr high   = (byte[5] + byte[6]) * 0.00003052
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x12, 0x48, 0x01, 0x00, 0x00}, {0x05, 0x10, 0x05, 0x00, 0x08, 0x00}, "ENG FAN"         }, // ECM_Engine fan duty            =  byte[5] * 100 / 255
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0xad, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x00, 0x00}, "MISFIRES"        }, // ECM_ignition misfires number   =  byte[5] * 256 + byte[6]
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x15, 0x7d, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x04, 0x45}, "FUEL PRESS"      }, // ECM fuel pressure              = (byte[5] * 256 + byte[6]) * 0.0724792480 - 69
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x15, 0x83, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x09, 0x00}, "FUEL PUMP"       }, // ECM fuel pump duty             = (byte[5] * 256 + byte[6]) * 0.0015287890625
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x2d, 0x01, 0x00, 0x00}, {0x05, 0x07, 0x05, 0x00, 0x08, 0x00}, "TCV"             }, // ECM TCV duty                   =  byte[5] * 191.25 / 255
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x4e, 0x01, 0x00, 0x00}, {0x05, 0x10, 0x05, 0x00, 0x08, 0x00}, "THROTTLE"        }, // ECM throttle angle             =  byte[5] * 100 / 255
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x9a, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x00, 0x00}, "MAF"             }, // ECM air flow meter             = (byte[5] * 256 + byte[6]) * 0.1
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x13, 0x63, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x08, 0x00}, "CVVT IN"         }, // ECM VVT inlet angle            = (byte[5] * 256 + byte[6]) * 0.0390625
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x13, 0x62, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x08, 0x00}, "CVVT EX"         }, // ECM VVT exhaust angle          = (byte[5] * 256 + byte[6]) * 0.0390625
  {ECM_ID, {0xcd, 0x7a, 0xa6, 0x10, 0x2c, 0x01, 0x00, 0x00}, {0x05, 0x07, 0x05, 0x00, 0x08, 0x00}, "BTDC"            }, // ECM BTDC                       = (byte[5] * 191.25 / 255
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x05, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x00, 0x00}, "DEM PUMP"        }, // DEM pump current               =  byte[5] * 256 + byte[6]
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x05, 0x01, 0x00, 0x00}, {0x07, 0x08, 0x08, 0x00, 0x00, 0x00}, "DEM SLND"        }, // DEM solenoid current           =  byte[7] * 256 + byte[8]
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x03, 0x01, 0x00, 0x00}, {0x06, 0x00, 0x05, 0x06, 0x00, 0x00}, "OIL PRESS"       }, // DEM oil pressure               =  byte[5] * 0.0164
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x02, 0x01, 0x00, 0x00}, {0x05, 0x00, 0x05, 0x10, 0x00, 0x00}, "OIL TEMP"        }, // DEM oil temperature            = (signed char)byte[5]
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x06, 0x01, 0x00, 0x00}, {0x08, 0x00, 0x09, 0x00, 0x06, 0x00}, "FL SPD"          }, // DEM FL velocity                = (byte[8] * 256 + bytes[9]) * 0.0156
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x06, 0x01, 0x00, 0x00}, {0x06, 0x00, 0x07, 0x00, 0x06, 0x00}, "FR SPD"          }, // DEM FR velocity                = (byte[6] * 256 + bytes[7]) * 0.0156
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x06, 0x01, 0x00, 0x00}, {0x0c, 0x00, 0x0d, 0x00, 0x06, 0x00}, "RL SPD"          }, // DEM RL velocity                = (byte[12] * 256 + bytes[13]) * 0.0156
  {DEM_ID, {0xcd, 0x1a, 0xa6, 0x00, 0x06, 0x01, 0x00, 0x00}, {0x0a, 0x00, 0x0b, 0x00, 0x06, 0x00}, "RR SPD"          }, // DEM RR velocity                = (byte[10] * 256 + bytes[11]) * 0.0156
  {REM_ID, {0xcd, 0x46, 0xa6, 0xd0, 0xd4, 0x01, 0x00, 0x00}, {0x05, 0x00, 0x05, 0x10, 0x03, 0x00}, "BAT VOLT"        }, // REM battery voltage            =  byte[5] / 8
  {CCM_ID, {0xcd, 0x29, 0xa6, 0x00, 0x01, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x06, 0x64}, "EVAPORATOR"      }, // CCM evaporator temperature     =  byte[5] * 256 + byte[6]) * 0.015625 - 100
  {CCM_ID, {0xcd, 0x29, 0xa6, 0x00, 0xa1, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x06, 0x64}, "CAB TEMP"        }, // CCM cabin temperature          =  byte[5] * 256 + byte[6]) * 0.015625 - 100
  {CCM_ID, {0xcd, 0x29, 0xa6, 0x00, 0x30, 0x01, 0x00, 0x00}, {0x05, 0x08, 0x06, 0x00, 0x06, 0x00}, "CAB FAN SPD"     }, // CCM cabin fan speed            =  byte[5] * 256 + byte[6]) * 0.015625
// ------------------------------------------------------------------------------------------------------------------+
};

uint32_t              CANrxId = 0;                                                    // header of LS-CAN packet for receiving
uint8_t        CANrxBuf[0xff] = {0};                                                  // buffer for receiving a packet from LS-CAN
uint8_t              CANrxLen = 8;                                                    // length of receiving buffer for LS-CAN
uint8_t                CANext = 1;                                                    // extended attribute of LS-CAN packet (29 bit)
uint32_t                   id = 0;                                                    // CAN module id
char                 txt1[16] = {0};                                                  // text line 1
char                 txt2[16] = {0};                                                  // text line 2
uint8_t             curScreen = 0;                                                    // index of current screen
boolean             enableLCD = true;                                                 // indicates screen is active
const uint8_t   TOTAL_SCREENS = sizeof(req) / sizeof(request_t);                      // total screens number
const uint16_t     LONG_PRESS = 1000;                                                 // long press duration

/**
 * @brief  Structure stores current calculated values
 */
struct result_t {
                        float   res;                                                  // calculated result
                      boolean   rdy;                                                  // readiness sign
}       result[TOTAL_SCREENS] = {0};                                                  // array of calculated values

/**
 * @brief   Initializing LS-CAN driver
 * @param   none
 * @retval  bool:  true | false
 */
boolean SetupLSCAN(void) {
  Serial.print("LS-CAN driver initialized ");
  pinMode(LS_CAN_INT, INPUT_PULLUP);                                                  // set up INT Pin for LS-CAN
  if (LS_CAN_CS.begin(MCP_STDEXT, CAN_125KBPS, LS_CAN_QRZ) == CAN_OK)                 // init LS-CAN bus on 125kbps
    if (LS_CAN_CS.init_Mask(0, EXT, MSK_L1) == MCP2515_OK)                            // config LS-CAN mask0 & filters0-1
      if (LS_CAN_CS.init_Filt(0, EXT, SWM_ID) == MCP2515_OK)
        if (LS_CAN_CS.init_Filt(1, EXT, CEL_ID) == MCP2515_OK)
          if (LS_CAN_CS.init_Mask(1, EXT, MSK_L2) == MCP2515_OK)                      // config LS-CAN mask1 & filters2-5
            if (LS_CAN_CS.init_Filt(2, EXT, REM_ID) == MCP2515_OK)
              if (LS_CAN_CS.init_Filt(3, EXT, CCM_ID) == MCP2515_OK)
                if (LS_CAN_CS.init_Filt(4, EXT, NON_ID) == MCP2515_OK)
                  if (LS_CAN_CS.init_Filt(5, EXT, NON_ID) == MCP2515_OK) {
                    LS_CAN_CS.setSleepWakeup(1);
                    LS_CAN_CS.setMode(MCP_NORMAL);                                    // set up LS-CAN in normal mode
                    Serial.println("successfully!   ;-)");
                    return true;
                  };
  Serial.println("failed!   :-(");
  return false;
}

/**
 * @brief   Initializing HS-CAN driver
 * @param   none
 * @retval  bool:  true | false
 */
boolean SetupHSCAN(boolean spd) {
  boolean isOk = false;
  Serial.print("HS-CAN BUS driver initialized ");
  pinMode(HS_CAN_INT, INPUT_PULLUP);                                                  // set up INT Pin for HS-CAN
  if (spd) {
    if (HS_CAN_CS.begin(MCP_STDEXT, CAN_500KBPS, HS_CAN_QRZ) == CAN_OK) {             // init HS-CAN bus on 500kbps
      Serial.print("on 500kbs ");
      isOk = true;
    };
  } else {
    if (HS_CAN_CS.begin(MCP_STDEXT, CAN_250KBPS, HS_CAN_QRZ) == CAN_OK) {             // init HS-CAN bus on 250kbps
      Serial.print("on 250kbs ");
      isOk = true;
    };
  };
  if (isOk)
    if (HS_CAN_CS.init_Mask(0, EXT, MSK_H1) == MCP2515_OK)                            // config HS-CAN mask0 & filters0-1
      if (HS_CAN_CS.init_Filt(0, EXT, TCM_ID) == MCP2515_OK)
        if (HS_CAN_CS.init_Filt(1, EXT, ECM_ID) == MCP2515_OK)
          if (HS_CAN_CS.init_Mask(1, EXT, MSK_H2) == MCP2515_OK)                      // config HS-CAN mask1 & filters2-5
            if (HS_CAN_CS.init_Filt(2, EXT, DEM_ID) == MCP2515_OK)
              if (HS_CAN_CS.init_Filt(3, EXT, CEH_ID) == MCP2515_OK)
                if (HS_CAN_CS.init_Filt(4, EXT, NON_ID) == MCP2515_OK)
                  if (HS_CAN_CS.init_Filt(5, EXT, NON_ID) == MCP2515_OK) {
                    HS_CAN_CS.setSleepWakeup(1);
                    HS_CAN_CS.setMode(MCP_NORMAL);                                    // set up HS-CAN in normal mode
                    Serial.println("successfully!   ;-)");
                    return true;
                  };
  Serial.println("failed!   :-(");
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
  while (1) {
    PINB = _BV(PINB5);                                                                // toggle PB5
    delay(500);
  };
}

/**
 * @brief   Turn on LCD
 * @param   none
 * @retval  none
 */
void EnableLCD() {
  uint8_t DIM_LCD_enable[2][8] = {LCD_ENABLE1, LCD_ENABLE2,};                         // command to turn on LCD
  for (uint8_t i = 0; i < 2; i++) {
    LS_CAN_CS.sendMsgBuf(LCD_ID, EXT, LEN, DIM_LCD_enable[i]);
    delay(MS);
  };
}

/**
 * @brief   Turn off LCD
 * @param   none
 * @retval  none
 */
void DisableLCD() {
  uint8_t DIM_LCD_disable[8] = LCD_DISABLE;                                           // command to turn off LCD
  LS_CAN_CS.sendMsgBuf(LCD_ID, EXT, LEN, DIM_LCD_disable);
  delay(MS);
}

/**
 * @brief   Clearing LCD
 * @param   none
 * @retval  none
 */
void ClearLCD() {
  uint8_t DIM_LCD_clear[8] = LCD_CLEAR;                                               // command to clear LCD
  LS_CAN_CS.sendMsgBuf(LCD_ID, EXT, LEN, DIM_LCD_clear);
  delay(MS);
}

/**
 * @brief   Output 2 lines
 * @param   outVal - boolean output value print
 * @param   value1 - numeric value1 (may be negative)
 * @param   label1 - text label1
 * @param   value2 - numeric value2 (may be negative)
 * @param   label2 - text label2
 * @retval  none
 */
void PrintScreen(boolean outVal, int16_t value1, const char* label1, int16_t value2, const char* label2) {
  static char buffer[2][17] = {0};                                                    // 2x (16 symbols + '\0')
         uint8_t        len = 0;
  if (!label1 || strlen(label1) == 0) label1 = "---";
  if (outVal)  { len = snprintf(buffer[0], 17, "%s %d", label1, value1);              // forming a string with value
    } else     { len = snprintf(buffer[0], 17, "%s",    label1); }                    // forming a string without value
  if (len < 16) {
    for (uint8_t i = len; i < 16; i++)  buffer[0][i] = ' ';                           // filling voids via spaces
    buffer[0][16] = '\0';  
  };
  if (!label2 || strlen(label2) == 0) label2 = "---";
  if (outVal)  { len = snprintf(buffer[1], 17, "%s %d", label2, value2);              // forming a string with value
    } else     { len = snprintf(buffer[1], 17, "%s",    label2); }                    // forming a string without value
  if (len < 16 ) {
    for (uint8_t i = len; i < 16; i++)  buffer[1][i] = ' ';                           // filling voids via spaces
    buffer[1][16] = '\0';  
  };
  uint8_t msg[5][8] = {
    {0xa7,          0x00, buffer[0][ 0], buffer[0][ 1], buffer[0][ 2], buffer[0][ 3], buffer[0][ 4], buffer[0][ 5]},
    {0x21, buffer[0][ 6], buffer[0][ 7], buffer[0][ 8], buffer[0][ 9], buffer[0][10], buffer[0][11], buffer[0][12]},
    {0x22, buffer[0][13], buffer[0][14], buffer[0][15], buffer[1][ 0], buffer[1][ 1], buffer[1][ 2], buffer[1][ 3]},
    {0x23, buffer[1][ 4], buffer[1][ 5], buffer[1][ 6], buffer[1][ 7], buffer[1][ 8], buffer[1][ 9], buffer[1][10]},
    {0x65, buffer[1][11], buffer[1][12], buffer[1][13], buffer[1][14], buffer[1][15],          0x00,          0x00},
  };
  Serial.println(buffer[0]);
  Serial.println(buffer[1]);
  for (uint8_t i = 0; i < 5; i++) {
    LS_CAN_CS.sendMsgBuf(PHM_ID, EXT, LEN, msg[i]);
    delay(MS);
  };
}

/**
 * @brief   Reads VIN from CEM via LS-CAN
 * @param   vin_buffer - char[18] to store VIN (null-terminated)
 * @retval  true if VIN successfully read, false otherwise
 */
boolean ReadVIN(char* vin_buffer) {
//uint8_t id_request[]      = {0x00, 0x07, 0xdf};                                     // UDS request: 00 07 df (id)
//uint8_t vin_request[]     = {0x02, 0x09, 0x02, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc};       // UDS request: 02 09 02 (payload bytes - 02, mode - 09, PID - 02 => VIN)
  uint8_t vin_request[]     = {0x03, 0x22, 0xf1, 0x90, 0x00, 0x00, 0x00, 0x00};
  uint8_t vin_data[17]      = {0};
  uint8_t received_bytes    = 0;
  uint8_t expected_packets  = 5;                                                      // typical for VIN in Volvo P2
  LS_CAN_CS.sendMsgBuf(DIA_ID, EXT, LEN, vin_request);                                // sending STD request command
  uint32_t timeout = millis() + WAIT;                                                 // response waiting time
  uint8_t packet_count = 0;
  while (millis() < timeout) {
    if (LS_CAN_CS.checkReceive() == CAN_MSGAVAIL) {                                   // checking reception of data from LS-CAN while waiting for response
      LS_CAN_CS.readMsgBuf(&CANrxId, &CANext, &CANrxLen, CANrxBuf);                   // reading incoming packet on LS-CAN driver
      if (CEL_ID == (MSK_ID & CANrxId)) {                                             // checking packet ID
        if (CANext) { Serial.print("Found EXTended packet in LS-CAN required ID: ");
          } else { Serial.print("Found standard packet in LS-CAN required ID: "); };
        Serial.print(CANrxId, HEX); Serial.print("\tLEN: "); Serial.println(CANrxLen, DEC);
        if (packet_count == 0) {
          for (uint8_t i = 2; i < 8 && received_bytes < 17; i++) { vin_data[received_bytes++] = CANrxBuf[i]; }; // receiving first packet
        } else {
          for (uint8_t i = 1; i < 8 && received_bytes < 17; i++) { vin_data[received_bytes++] = CANrxBuf[i]; }; // receiving remaining packets
        };
        packet_count++;
        timeout = millis() + WAIT;                                                    // extend timeout when receiving package
      };
    };
  }; // while (checkReceive...
  if (received_bytes >= 17) {
    memcpy(vin_buffer, vin_data, 17);
    vin_buffer[17] = '\0';
    Serial.print("VIN: ");
    Serial.println(vin_buffer);
    return true;
  };
  Serial.println("VIN read failed");
  return false;
}

/**
 * @brief   Extract model year from VIN
 * @param   vin - VIN string (17 chars)
 * @retval  model year (e.g. 2005, 2011)
 */
uint16_t GetModelYearFromVIN(const char* vin) {
  if (strlen(vin) < 10) return 0;
  char year_char = vin[9]; // 10th character (0-based index 9)
  if (year_char >= '0' && year_char <= '9') {
    return 2000 + (year_char - '0');
  } else if (year_char >= 'A' && year_char <= 'L') {                                  // A=2010, B=2011, ..., L=2020
    return 2010 + (year_char - 'A');
  };
  return 0;
}

/**
 * @brief   Hardware setup
 * @param   none
 * @retval  none
 */
void setup() {
  Serial.begin(115200);
  if (!SetupLSCAN())  HardFault();                                                    // init LS-CAN 125kbps
  char vin[18]        = {0};                                                          // 17 characters VIN + \0
  uint16_t model_year = 0;                                                            // vehicle model year 
  boolean hs_500kbps  = true;
  if (ReadVIN(vin)) {                                                                 // VIN request to determine model year
    model_year = GetModelYearFromVIN(vin);
    if (model_year > 0) {
      hs_500kbps = (model_year >= 2005);
      Serial.print("Detected model year: ");
      Serial.println(model_year);
    };
  } else { Serial.println("Using default: 500 kbps HS-CAN (2005+)"); };
  if (!SetupHSCAN(hs_500kbps))  HardFault();                                          // init HS-CAN
  EnableLCD();                                                                        // LCD turn on
  ClearLCD();
  PrintScreen(false, 0, "*   VOLVO P2   *", 0, "*   INFORMER   *");
  delay(1000);
}

/**
 * @brief   Button handler
 * @param   buf - received response
 * @retval  none
 */
void HandlerSWM(uint8_t *buf) {
  static uint32_t pressStart = 0;                                                     // button press time
  static boolean   isPressed = false;                                                 // button current status
  if (buf[BYTE_INFO_BUT] == INFO_BUT) {                                               // when 'RESET' button on SWM is pressed
    if (!isPressed) {
      isPressed = true;
      pressStart = millis();
      Serial.println("Button pressed");
    };
    if (isPressed && (millis() - pressStart >= LONG_PRESS)) {                         // long button press
      isPressed = false;
      Serial.println("Long press detected");
      enableLCD = !enableLCD;
    };
  } else {
    if (isPressed) {
      if (millis() - pressStart < LONG_PRESS) {                                       // short button press
        curScreen++;
        if (curScreen >= TOTAL_SCREENS)   curScreen = 0;
        Serial.println("Short press");
      };
      isPressed = false;
    };
  };
}

/**
 * @brief   CEM low speed CAN bus handler
 * @param   buf - received response
 * @retval  none
 */
void HandlerCEL(uint8_t *buf) { }

/**
 * @brief   LS-CAN packet reader
 * @param   none
 * @retval  none
 */
void ReadLSCAN() {
  while (LS_CAN_CS.checkReceive() == CAN_MSGAVAIL) {                                  // checking periodic messages of LS-CAN
    LS_CAN_CS.readMsgBuf(&CANrxId, &CANext, &CANrxLen, CANrxBuf);
    switch (CANrxId & MSK_ID) {
      case SWM_ID:
        HandlerSWM (CANrxBuf);
        break;
      case CEL_ID:
        HandlerCEL (CANrxBuf);
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

/**
 * @brief   Request-response via CAN-bus
 * @param   none
 * @retval  none
 */
float RequestResponseCAN(uint8_t index) {
  id = pgm_read_dword(&req[index].id);                                                // buffer filling with current id
  uint8_t cmd[8] = {0};                                                               // request command
  for (uint8_t i = 0; i < 8; i++)
    cmd[i] = pgm_read_byte(&req[index].cmd[i]);                                       // buffer filling with current command
  uint8_t form[6] = {0};                                                              // parameters to process in formula
  for (uint8_t i = 0; i < 6; i++)
    form[i] = pgm_read_byte(&req[index].form[i]);                                     // buffer filling with current formula parameters
  char text[16] = {0};                                                                // text message
  for (uint8_t i = 0; i < 16; i++)
    text[i] = pgm_read_byte(&req[index].text[i]);                                     // buffer filling with current text
  if (PurposeCAN(id) == 1) {                                                          // action with HS-CAN
    HS_CAN_CS.sendMsgBuf(DIA_ID, EXT, LEN, cmd);                                      // sending request command
    uint32_t timeout = millis() + WAIT;                                               // response waiting time
    while (millis() < timeout) {
      if (HS_CAN_CS.checkReceive() == CAN_MSGAVAIL) {                                 // checking reception of data from HS-CAN while waiting for response
        HS_CAN_CS.readMsgBuf(&CANrxId, &CANext, &CANrxLen, CANrxBuf);                 // reading incoming packet on HS-CAN driver
        if (id == (MSK_ID & CANrxId)) {                                               // checking packet ID
//          if (CANext)  { Serial.print("Found EXTended packet in HS-CAN required ID: ");
//            } else {     Serial.print("Found standard packet in HS-CAN required ID: "); };
//          Serial.print(CANrxId, HEX);
//          Serial.print("\tLEN: ");
//          Serial.println(CANrxLen, DEC);
          result[index].res = ((((uint16_t)CANrxBuf[form[0]] << form[1]) + (CANrxBuf[form[2]] >> form[3])) >> form[4]) - form[5];
          result[index].rdy = true;
          return;
        }; // if (CANrxId...
      }; // if (CAN_MSGAVAIL...
    }; // while (checkReceive...
  }; // if (PurposeCAN...
  if (PurposeCAN(id) == 2) {                                                          // action with LS-CAN
    LS_CAN_CS.sendMsgBuf(DIA_ID, EXT, LEN, cmd);                                      // sending request command
    uint32_t timeout = millis() + WAIT;                                               // response waiting time
    while (millis() < timeout) {
      if (LS_CAN_CS.checkReceive() == CAN_MSGAVAIL) {                                 // checking reception of data from LS-CAN while waiting for response
        LS_CAN_CS.readMsgBuf(&CANrxId, &CANext, &CANrxLen, CANrxBuf);                 // reading incoming packet on LS-CAN driver
        if (id == (MSK_ID & CANrxId)) {                                               // checking packet ID
//          if (CANext) { Serial.print("Found EXTended packet in LS-CAN required ID: ");
//            } else { Serial.print("Found standard packet in LS-CAN required ID: "); };
//          Serial.print(CANrxId, HEX);
//          Serial.print("\tLEN: ");
//          Serial.println(CANrxLen, DEC);
          result[index].res = ((((uint16_t)CANrxBuf[form[0]] << form[1]) + (CANrxBuf[form[2]] >> form[3])) >> form[4]) - form[5];
          result[index].rdy = true;
          return;
        }; // if (CANrxId...
      }; // if (CAN_MSGAVAIL...
    }; // while (checkReceive...
  }; // if (PurposeCAN...
  return;
}

void loop() {
  static bool lastEnableLCD = !enableLCD;                                             // flag of previous state
  static uint32_t startTime = millis();
  uint8_t nexScreen = curScreen + 1;
  if (nexScreen >= TOTAL_SCREENS)   nexScreen = 0;
  if (enableLCD != lastEnableLCD)  {
    if (enableLCD)  { EnableLCD(); } else { DisableLCD(); };                          // LCD turn on/off
    lastEnableLCD = enableLCD;
  };
  ReadLSCAN();                                                                        // listening LS-CAN for button pressing
  if (millis() - startTime >= POLLS) {                                                // pause between polls
    for (uint8_t i = 0; i < TOTAL_SCREENS; i++) {
      RequestResponseCAN(i);                                                          // request-response via CAN-bus
      ReadLSCAN();
    };
    for (uint8_t i = 0; i < 16; i++) {
      txt1[i] = pgm_read_byte(&req[curScreen].text[i]);                               // buffer filling with text line1
      txt2[i] = pgm_read_byte(&req[nexScreen].text[i]);                               // buffer filling with text line2
    }
    PrintScreen(result[curScreen].rdy, result[curScreen].res, txt1, result[nexScreen].res, txt2); // print 'value + txt'
    startTime = millis();
  };
}
