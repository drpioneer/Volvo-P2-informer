/*
 * VOLVO P2 Informer
 * https://github.com/drpioneer/Volvo-P2-informer/
 * Designed to ArduinoNano & 2x MCP2515
 * tested on Volvo XC90 2011
 * (c) 2025 Code uses ideas and practices of authors: vtl1349, Olegelm, ten-da, HalfPunchMan, astaninss, drPioneer and others ...
 */

#include <SPI.h>
#include "mcp_can.h"

#define            LED_PIN    13                                                      // LED pin

// LS/HS-CAN shields parameters
#define         LS_CAN_SPD    CAN_125KBPS                                             // LS-CAN settings for Volvo P2 1999+
//#define       HS_CAN_SPD    CAN_250KBPS                                             // HS-CAN settings for Volvo P2 1999-2004
#define         HS_CAN_SPD    CAN_500KBPS                                             // HS-CAN settings for Volvo P2 2005+
#define         LS_CAN_QRZ    MCP_8MHZ                                                // frequency of quartz resonator for LS-CAN shield
#define         HS_CAN_QRZ    MCP_8MHZ                                                // frequency of quartz resonator for HS-CAN shield
#define         LS_CAN_INT    2                                                       // assigning interrupt pin to LS-CAN receiving buffer
#define         HS_CAN_INT    3                                                       // assigning interrupt pin to HS-CAN receiving buffer
MCP_CAN         LS_CAN_CS     (9);                                                    // using CS-pin for using LS-CAN shield
MCP_CAN         HS_CAN_CS     (10);                                                   // using CS-pin for using HS-CAN shield

// identifiers of CAN modules
//#define           ???_ID    0x80c00003                                              // vehicle speed on LS-CAN (central electronic module?) https://github.com/AcollaMolla/V70_autolock/blob/main/main/main.ino
//#define           ???_ID    0x81c01022                                              // doorlock info on LS-CAN (driver door module?)        https://github.com/AcollaMolla/V70_autolock/blob/main/main/main.ino
#define             DEM_ID    0x01204001                                              // differential electronic module 2005+
#define             ECM_ID    0x01200021                                              // engine control module 2005+
//#define           REM_ID    0x00800401                                              // rear electronic module 2005+
#define             CCM_ID    0x00801001                                              // climate electronic module 2005+
//#define           TCM_ID    0x00800005                                              // transmission control module 1999-2004
#define             TCM_ID    0x01200005                                              // transmission control module 2005+
//#define           SWM_ID    0x00100066                                              // steering wheel module 1999-2000
//#define           SWM_ID    0x00200066                                              // steering wheel module 2000-2001
//#define           SWM_ID    0x00400066                                              // steering wheel module 2002
//#define           SWM_ID    0x00404066                                              // steering wheel module 2003-2004
#define             SWM_ID    0x0131726c                                              // steering wheel module 2005+
//#define           PHM_ID    0x00400008                                              // phone module 2001
//#define           PHM_ID    0x00C00008                                              // phone module 2002
#define             PHM_ID    0x01800008                                              // phone module 2005+
//#define           SCR_ID    0x00c0200e                                              // screen on driver information module 2001
//#define           SCR_ID    0x0220200e                                              // screen on driver information module 2002
#define             SCR_ID    0x02a0240e                                              // screen on driver information module 2005+
#define            CEML_ID    0x02803008                                              // central electronic module LS-CAN 2005+
#define            CEMH_ID    0x03200408                                              // central electronic module HS-CAN 2005+
#define             DIA_ID    0x000ffffe                                              // diagnostic tool (VIDA)
#define       LS_CAN_MASK1    (SWM_ID |    0x00)                                      // mask1 to reduce load of Arduino when listening of LS-CAN
#define       LS_CAN_MASK2    (CCM_ID | CEML_ID)                                      // mask2 to reduce load of Arduino when listening of LS-CAN
#define       HS_CAN_MASK1    (TCM_ID |  ECM_ID)                                      // mask1 to reduce load of Arduino when listening of HS-CAN
#define       HS_CAN_MASK2    (DEM_ID |    0x00)                                      // mask2 to reduce load of Arduino when listening of HS-CAN

#define                EXT    1                                                       // CAN packet parameter: EXTENDED
#define                LEN    8                                                       // CAN packet parameter: LENGTH
#define           DELAY_MS    30                                                      // delay (in miliseconds)
#define              PAUSE    50000                                                   // pause (in cycles)
#define           REQUESTS    50                                                      // count of requests in CAN bus

//uint8_t       ena1SCR[8]  = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05};       // command 1 of 2 to turn on screen, 2000-2001
//uint8_t       ena2SCR[8]  = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};       // command 2 of 2 to turn on screen, 2000-2001
uint8_t         ena1SCR[8]  = {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35};       // command 1 of 2 to turn on screen, 2005+ или 2003-2004
uint8_t         ena2SCR[8]  = {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31};       // command 2 of 2 to turn on screen, 2005+ или 2003-2004
uint8_t          disSCR[8]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};       // command to turn off screen
uint8_t          clrSCR[8]  = {0xe1, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};       // command to clear screen
uint8_t         tempATF[8]  = {0xcc, 0x6e, 0xa5, 0x0c, 0x01, 0x00, 0x00, 0x00};       // transmission fluid temperature request
uint8_t         pumpCur[8]  = {0xcd, 0x1a, 0xa6, 0x00, 0x05, 0x01, 0x00, 0x00};       // DEM pump current request
uint8_t          engRPM[8]  = {0xcd, 0x7a, 0xa6, 0x10, 0x93, 0x01, 0x00, 0x00};       // ECM engine RPM request

// LS/HS-CAN Variables
uint32_t          lsCANtxId = 0;                                                      // header of LS-CAN packet for sending
uint8_t       lsCANtxBuf[8] = {0};                                                    // buffer for sending a packet in LS-CAN
uint8_t          lsCANtxLen = 8;                                                      // length of sending buffer in LS-CAN
uint32_t          lsCANrxId = 0;                                                      // header of LS-CAN packet for receiving
uint8_t       lsCANrxBuf[8] = {0};                                                    // buffer for receiving a packet from LS-CAN
uint8_t          lsCANrxLen = 8;                                                      // length of receiving buffer for LS-CAN
uint8_t            lsCANext = 1;                                                      // extended attribute of LS-CAN packet (29 bit)
uint32_t          hsCANtxId = 0;                                                      // header of HS-CAN packet for sending
uint8_t       hsCANtxBuf[8] = {0};                                                    // buffer for sending a packet in HS-CAN
uint8_t          hsCANtxLen = 8;                                                      // length of sending buffer in HS-CAN
uint32_t          hsCANrxId = 0;                                                      // header of HS-CAN packet for receiving
uint8_t       hsCANrxBuf[8] = {0};                                                    // buffer for receiving a packet from HS-CAN
uint8_t          hsCANrxLen = 8;                                                      // length of receiving buffer for HS-CAN
uint8_t            hsCANext = 1;                                                      // extended attribute of HS-CAN packet (29 bit)

uint8_t          actScreen  = 0;                                                      // index of current screen
uint32_t         progCycle  = 0;                                                      // cycle counter for determining elapsed time

boolean ls_can_ok = false, readit = false;
boolean SetupLSCAN()
{
  Serial.println("LS-CAN shield initialize...");
  pinMode(LS_CAN_INT, INPUT_PULLUP);                                                  // set up INT Pin for LS-CAN
  for (uint8_t i = 0; i < 4; i++)
  {
    if (LS_CAN_CS.begin(MCP_ANY, LS_CAN_SPD, LS_CAN_QRZ) == CAN_OK)                   // init LS-CAN bus
    {
      LS_CAN_CS.setMode(MCP_NORMAL);                                                  // set up LS-CAN in normal mode
      ls_can_ok = true;
      break;
    }
  }
  if (ls_can_ok)
  {
    uint8_t err = 0;
    if (LS_CAN_CS.init_Mask(0, EXT, LS_CAN_MASK1) != MCP2515_OK)
      err++;
    if (LS_CAN_CS.init_Mask(1, EXT, LS_CAN_MASK2) != MCP2515_OK)
      err++;
    if (LS_CAN_CS.init_Filt(0, EXT,       SWM_ID) != MCP2515_OK)
      err++;
    if (LS_CAN_CS.init_Filt(1, EXT,         0x00) != MCP2515_OK)
      err++;
    if (LS_CAN_CS.init_Filt(2, EXT,         0x00) != MCP2515_OK)
      err++;
    if (LS_CAN_CS.init_Filt(3, EXT,         0x00) != MCP2515_OK)
      err++;
    if (LS_CAN_CS.init_Filt(4, EXT,         0x00) != MCP2515_OK)
      err++;
    if (LS_CAN_CS.init_Filt(5, EXT,         0x00) != MCP2515_OK)
      err++;
    if (err)
      ls_can_ok = false;
  }
  if (!ls_can_ok)
  {
    Serial.println("LS-CAN shield initialized failed!");
    return false;
  }
  LS_CAN_CS.setSleepWakeup(1);
  Serial.println("LS-CAN shield initialized successfully!");
  return true;
}

boolean hs_can_ok = false;
boolean SetupHSCAN()
{
  Serial.println("HS-CAN BUS Shield initialize...");
  pinMode(HS_CAN_INT, INPUT_PULLUP);                                                    // set up INT Pin for HS-CAN
  for (uint8_t i = 0; i < 4; i++)
  {
    if (HS_CAN_CS.begin(MCP_ANY, HS_CAN_SPD, HS_CAN_QRZ) == CAN_OK)                     // init HS-CAN bus
    {
      HS_CAN_CS.setMode(MCP_NORMAL);                                                    // set up HS-CAN in normal mode
      hs_can_ok = true;
      break;
    }
    Serial.print(".");
    delay(DELAY_MS);
  }

  if (hs_can_ok)
  {
    uint8_t err = 0;
    if (HS_CAN_CS.init_Mask(0, EXT, HS_CAN_MASK1) != MCP2515_OK)
      err++;
    if (HS_CAN_CS.init_Mask(1, EXT, HS_CAN_MASK2) != MCP2515_OK)
      err++;
    if (HS_CAN_CS.init_Filt(0, EXT,       TCM_ID) != MCP2515_OK)
      err++;
    if (HS_CAN_CS.init_Filt(1, EXT,       DEM_ID) != MCP2515_OK)
      err++;
    if (HS_CAN_CS.init_Filt(2, EXT,       ECM_ID) != MCP2515_OK)
      err++;
    if (HS_CAN_CS.init_Filt(3, EXT,         0x00) != MCP2515_OK)
      err++;
    if (HS_CAN_CS.init_Filt(4, EXT,         0x00) != MCP2515_OK)
      err++;
    if (HS_CAN_CS.init_Filt(5, EXT,         0x00) != MCP2515_OK)
      err++;
    if (err)
      hs_can_ok = false;
  }
  if (!hs_can_ok)
  {
    Serial.println("HS-CAN shield initialized failed!");
    return false;
  }
  HS_CAN_CS.setSleepWakeup(1);
  Serial.println("HS-CAN shield initialized successfully!");
  return true;
}

void HardFault()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.println("!!! ATENTION!!! ABNORMAL TERMINATION !!! CHECK EQUIPMENT !!!");
  while (1)
  {
    digitalWrite(LED_PIN, HIGH);                                                      // LED on
    delay(DELAY_MS * 30);                                                             // pause
    digitalWrite(LED_PIN, LOW);                                                       // LED off
    delay(DELAY_MS * 30);                                                             // pause
  }
}

void EnaSCR()
{
  LS_CAN_CS.sendMsgBuf(SCR_ID, EXT, LEN, ena1SCR);
  delay(DELAY_MS);
  LS_CAN_CS.sendMsgBuf(SCR_ID, EXT, LEN, ena2SCR);
  delay(DELAY_MS);
  Serial.println("Screen is turned on!");
}

void DisSCR()
{
  LS_CAN_CS.sendMsgBuf(SCR_ID, EXT, LEN, clrSCR);
  delay(DELAY_MS);
  LS_CAN_CS.sendMsgBuf(SCR_ID, EXT, LEN, disSCR);
  delay(DELAY_MS);
  Serial.println("Screen is turned off!");
}

void ClrSCR()
{
  LS_CAN_CS.sendMsgBuf(SCR_ID, EXT, LEN, clrSCR);
  delay(DELAY_MS);
  Serial.println("Screen is clear!");
}

void PrnSCR(String string)
{ 
  Serial.println("Print message: '" + string + "'");
  string = string + "                                ";
  char message[32] = { 0 }, ltr[32] = { 0 };
  for (uint8_t i = 0; i < sizeof(ltr); i+=1)
  {
    message[32] = 0x20;
    ltr[i]      = 0x20;                                                               // filling of spaces in message
  }
  string.toCharArray(message, 33);
  memcpy(ltr, message, 32);                                                           // moving message to ltr array
  uint8_t Msg_1[8] = {0xa7,   0x00,  ltr[0],  ltr[1],  ltr[2],  ltr[3],  ltr[4],  ltr[5]}; 
  LS_CAN_CS.sendMsgBuf(PHM_ID, EXT, LEN, Msg_1);
  delay(DELAY_MS);
  uint8_t Msg_2[8] = {0x21, ltr[6],  ltr[7],  ltr[8],  ltr[9],  ltr[10], ltr[11], ltr[12]}; 
  LS_CAN_CS.sendMsgBuf(PHM_ID, EXT, LEN, Msg_2);
  delay(DELAY_MS);
  uint8_t Msg_3[8] = {0x22, ltr[13], ltr[14], ltr[15], ltr[16], ltr[17], ltr[18], ltr[19]}; 
  LS_CAN_CS.sendMsgBuf(PHM_ID, EXT, LEN, Msg_3);
  delay(DELAY_MS);
  uint8_t Msg_4[8] = {0x23, ltr[20], ltr[21], ltr[22], ltr[23], ltr[24], ltr[25], ltr[26]}; 
  LS_CAN_CS.sendMsgBuf(PHM_ID, EXT, LEN, Msg_4);
  delay(DELAY_MS);
  uint8_t Msg_5[8] = {0x65, ltr[27], ltr[28], ltr[29], ltr[30], ltr[31],    0x00,    0x00}; 
  LS_CAN_CS.sendMsgBuf(PHM_ID, EXT, LEN, Msg_5);
  delay(DELAY_MS);
}

void setup() 
{
  Serial.begin(115200);
  Serial.println("");
  if (!SetupLSCAN())
    HardFault();
  if (!SetupHSCAN())
    HardFault();
  DisSCR();
}

void ActSWM (uint8_t *buf)
{
  if (buf[7] == 0xbf)                                                                 // when 'INFO/RESET' button on SWM is pressed
  {
    actScreen++;                                                                      // increasing index of information screen
    readit = false;                                                                   // reset read it status
    progCycle = PAUSE;
    Serial.println("Pressed 'RESET' key on SWM");
  } 
}

void ReadLSCAN()
{
  while (LS_CAN_CS.checkReceive() == CAN_MSGAVAIL)
  {
    LS_CAN_CS.readMsgBuf(&lsCANrxId, &lsCANext, &lsCANrxLen, lsCANrxBuf);
    switch (lsCANrxId)
    {
      case SWM_ID:
        ActSWM (lsCANrxBuf);
        break;
      default:
        break;
    }
  }
}

void loop() 
{
  progCycle++;                                                                        // increasing delay counter before a new parameter request in CAN
  if (progCycle > PAUSE)
    progCycle = 0;
  ReadLSCAN();                                                                        // listening LS-CAN
  if (progCycle == PAUSE)                                                             // every 50 000 cycles (conditional 1 seс)
  {
    switch (actScreen)                                                                // switch according to active screen
    {
      case 1:
        Serial.println("Case 1: enable screen + greating");
        EnaSCR();                                                                     // enable screen
        ClrSCR();                                                                     // clear  screen
        PrnSCR("*   VOLVO P2   **   INFORMER   *");                                   // print on screen
        progCycle = 0;
        break;

      case 2:
        Serial.println("Case 2: ATF temperature");
        if (!readit)
          PrnSCR(String("--C ATF TEMP"));
        for (uint8_t i = 0; i < REQUESTS; i++)
        {
          HS_CAN_CS.sendMsgBuf(DIA_ID, 1, 8, tempATF);                                // sending request to get AT temperature
          while (HS_CAN_CS.checkReceive() == CAN_MSGAVAIL)                            // check if data is coming on shield
          {
            HS_CAN_CS.readMsgBuf(&hsCANrxId, &hsCANext, &hsCANrxLen, hsCANrxBuf);     // reading incoming packet on HS-CAN
            if (hsCANrxId == TCM_ID)                                                  // comparing packet ID
            {
              Serial.print("Found required ID: ");
              Serial.println(hsCANrxId, HEX);
              PrnSCR(String(256L * hsCANrxBuf[6] + hsCANrxBuf[7]) + "C ATF TEMP");    // print AT temperature in celcius
              readit = true;
              break;
            }
          }
        }
        progCycle = 0;
        break;

      case 3:
        Serial.println("Case 3: DEM pump current");
        if (!readit)
          PrnSCR(String("---MA PUMP CURR"));
        for (uint8_t i = 0; i < REQUESTS; i++)
        {
          HS_CAN_CS.sendMsgBuf(DIA_ID, 1, 8, pumpCur);                                // sending request to DEM pump current
          while (HS_CAN_CS.checkReceive() == CAN_MSGAVAIL)                            // check if data is coming on shield
          {
            HS_CAN_CS.readMsgBuf(&hsCANrxId, &hsCANext, &hsCANrxLen, hsCANrxBuf);     // reading incoming packet on HS-CAN
            if (hsCANrxId == DEM_ID)                                                  // comparing packet ID
            {
              Serial.print("Found required ID: ");
              Serial.println(hsCANrxId, HEX);
              PrnSCR(String(256L * hsCANrxBuf[5] + hsCANrxBuf[6]) + "MA PUMP CURR");  // print DEM pump current
              readit = true;
              break;
            }
          }
        }
        progCycle = 0;
        break;

      case 4:
        Serial.println("Case 4: ECM engine RPM");
        if (!readit)
          PrnSCR(String("----RPM ENGINE"));
        for (uint8_t i = 0; i < REQUESTS; i++)
        {
          HS_CAN_CS.sendMsgBuf(DIA_ID, 1, 8, engRPM);                                 // sending request to ECM engine RPM
          while (HS_CAN_CS.checkReceive() == CAN_MSGAVAIL)                            // check if data is coming on shield
          {
            HS_CAN_CS.readMsgBuf(&hsCANrxId, &hsCANext, &hsCANrxLen, hsCANrxBuf);     // reading incoming packet on HS-CAN
            //Serial.println(hsCANrxId, HEX);
            if (hsCANrxId == ECM_ID)                                                  // comparing packet ID
            {
              Serial.print("Found required ID: ");
              Serial.println(hsCANrxId, HEX);
              PrnSCR(String((256L * hsCANrxBuf[5] + hsCANrxBuf[6])/4) + "RPM ENGINE"); // print ECM boost pressure
              readit = true;
              break;
            }
          }
        }
        progCycle = 0;
        break;

      case 5:
        Serial.println("Case 5: nothing");
        if (!readit)
          PrnSCR(String("NOTHING"));
        progCycle = 0;
        break;

      default:
        Serial.println("Default case: disable screen");
        DisSCR();                                                                     // disable screen on DIM
        actScreen = 0;
        progCycle = 0;
        delay (500);
        break;
    }
  }
}
