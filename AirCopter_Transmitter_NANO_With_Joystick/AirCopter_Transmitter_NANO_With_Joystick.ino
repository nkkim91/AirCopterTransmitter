/*************************************************************************************
 * 
 * Once the debug mask is enabled and debug message is printed, the loop cycle time increases and consequently, increase the delay for TX.
 * So, it's recommended to enable the mask, only necessary
 * 
 *************************************************************************************/

#include "StreamData.h"
#include "config.h"
#include "printf.h"

#ifdef ARDUINO_AVR_UNO
    #define __BOARD_TYPE "UNO"
#endif
#ifdef ARDUINO_AVR_NANO
    #define __BOARD_TYPE "NANO"
#endif
#ifdef ARDUINO_AVR_MINI
    #define __BOARD_TYPE "ProMini"
#endif
#ifdef ARDUINO_AVR_MEGA2560
    #define __BOARD_TYPE "MEGA"
#endif
#ifdef ARDUINO_AVR_LEONARDO
    #define __BOARD_TYPE "LEONARDO"
#endif
#ifdef ARDUINO_AVR_MICRO
    #define __BOARD_TYPE "ProMicro"
#endif

#define TX_PERIOD (11)    // 11 ms, smaller number is not working because of the operation time in the loop

#ifdef DEBUG_NK
/***************************************************
 *
 * Debug MASK Data structure / Macro       
 *
 ***************************************************/
#define NK_DEBUG_SPOT  (1 << 1)
#define NK_DEBUG_INFO  (1 << 2)
#define NK_DEBUG_DEBUG  (1 << 3)
#define NK_DEBUG_DELTA (1 << 4)

//static uint32_t  gunDebugMask = NK_DEBUG_INFO | NK_DEBUG_SPOT | NK_DEBUG_DELTA | NK_DEBUG_DEBUG;
//static uint32_t  gunDebugMask = NK_DEBUG_DEBUG | NK_DEBUG_DELTA;
//static uint32_t  gunDebugMask = NK_DEBUG_DEBUG | NK_DEBUG_SPOT;
//static uint32_t  gunDebugMask = NK_DEBUG_DEBUG | NK_DEBUG_INFO;
static uint32_t  gunDebugMask = NK_DEBUG_INFO;
#endif

unsigned long ulLastTxTime = 0;
union TxStreamData stTxStreamData;

unsigned int    gunChannelData[MAX_TX_CHANNEL] = {             // 16
                            0x0400,  0x0400,  0x00CC,  0x0400,
                            0x00CC,  0x0400,  0x00CC,  0x00CC,
                            0x00CC,  0x0400,  0x0400,  0x0400,
                            0x0400,  0x0400,  0x0400,  0x0400,
                            };

unsigned long ulLastIRQTime = millis();

#define MSP_CMD_SET_RAW_RC_TINY (0x96)
#define MSP_CMD_ARM             (0x97)
#define MSP_CMD_DISARM          (0x98)

#define MSP_CMD_TRIM_UP         (0x99)
#define MSP_CMD_TRIM_DOWN       (0x9A)
#define MSP_CMD_TRIM_LEFT       (0x9B)
#define MSP_CMD_TRIM_RIGHT      (0x9C)

#define MSP_CMD_SKYCOPTER       (0xC7)

unsigned char TX_MSG[MAX_MSG_BUF_SIZE];

unsigned long ullCounter = 0;
unsigned long ullErrorCounter = 0;
unsigned long ullCurrentmS = 0;

unsigned char ucMSP_Header[] = { '$', 'M', '<' };

#ifdef SKYCOPTER
unsigned char ucMSP_SET_RAW_RC_TINY[] = { '$', 'M', '<', 0x05, MSP_CMD_SET_RAW_RC_TINY, 0x7D, 0x7D, 0x7D, 0x00, 0x55, 0xBB };
#elif defined(AIRCOPTER)
unsigned char ucMSP_SET_RAW_RC_TINY[] = { '$', 'M', '<', 0x05, MSP_CMD_SET_RAW_RC_TINY, 0x00, 0x00, 0x00, 0x00, 0x05, 0xEB };
#endif

unsigned char ucMSP_ARM[] = { '$', 'M', '<', 0x00, MSP_CMD_ARM, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  
unsigned char ucMSP_DISARM[] = { '$', 'M', '<', 0x00, MSP_CMD_DISARM, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  

unsigned char CalcMSPCRC(unsigned char *ucData, int nLength)
{
  int i;
  unsigned char ucCRC;
  
  for(i = 1, ucCRC = ucData[0]; i < nLength; i++) {
    ucCRC ^= ucData[i];
#ifdef DEBUG_NK
  // within your code, wherever necessary:
    if( gunDebugMask & NK_DEBUG_DEBUG ) {
      printf("0x%02x ", ucData[i]);
    }
#endif // DEBUG_NK
  }

#ifdef DEBUG_NK
  // within your code, wherever necessary:
  if( gunDebugMask & NK_DEBUG_DEBUG ) {
    printf("\n");
  }
#endif // DEBUG_NK

  return ucCRC;
}

void setup() {

#ifdef DEBUG_NK
  Serial.begin(115200);
  printf_begin();
#endif // DEBUG_NK

#ifdef DEBUG_NK
  // within your code, wherever necessary:
  if( gunDebugMask & NK_DEBUG_DEBUG ) {
    Serial.print("Board Type : "); Serial.println(__BOARD_TYPE);
  }
#endif // DEBUG_NK

  JS_Setup();

  /* 
   *  Timer1 for counting in external interrupt (button)
   */
  TCNT1 = 0;
  TCCR1A = 0x0; // COM1A1/B1, COM1A0/B0, WGM11/10 = 0x0 (Normal)
//  TCCR1B = 0x8 | 0x5; // WGM13/12 (01 - CTC), CS12/11/10 = 0x5 (/1024), 0x0(/1)
  TCCR1B = 0x5; // WGM13/12 (01 - CTC), CS12/11/10 = 0x5 (/1024), 0x0(/1)  
  OCR1A = 0x3E80;
//  TIMSK1 = 0x2; // OCIE1A(bit1)

#ifdef DEBUG_NK
  if( gunDebugMask & NK_DEBUG_DEBUG ) {

    printf("%u TCNT1           : %u \n", millis(), TCNT1);
    printf("%u TCCR1A          : %u \n", millis(), TCCR1A);
    printf("%u TCCR1B          : %u \n", millis(), TCCR1B);
    printf("%u TCCR1C          : %u \n", millis(), TCCR1C);
    printf("%u TIMSK1          : %u \n", millis(), TIMSK1);
  }
#endif // DEBUG_NK

#ifdef DEBUG_NK
  if( gunDebugMask & NK_DEBUG_INFO ) {
    printf("setup() done !!\n");
  }
#endif // DEBUG_NK
}

void loop() {

  unsigned char ucCRC;
  unsigned char ucCmd;
  unsigned char ucMsgLength;
  unsigned char *pucMSPMsg;
  int i;
 
  if( millis() - ulLastTxTime >= TX_PERIOD ) {
    ulLastTxTime = millis();    
  }
  
#ifdef DEBUG_NK
    if( gunDebugMask & NK_DEBUG_DEBUG ) {
      printf("Send TX Stream data !! \n");
    }
#endif

  JS_Input();

  if( gunChannelData[ARM] == CHANNEL_MAX_100 ) {

    ucCRC = CalcMSPCRC(&(ucMSP_ARM[MSP_ARM_SIZE]), MSP_ARM_CRC_SZ);  //
    ucMSP_ARM[MSP_ARM_CRC] = ucCRC;

    ucCmd = MSP_ARM;
    ucMsgLength = MSP_ARM_MSG_SZ;
    
  } else if ( gunChannelData[DISARM] == CHANNEL_MAX_100 ) {

    ucCRC = CalcMSPCRC(&(ucMSP_DISARM[MSP_DISARM_SIZE]), MSP_DISARM_CRC_SZ);  //
    ucMSP_DISARM[MSP_DISARM_CRC] = ucCRC;

    ucCmd = MSP_DISARM;
    ucMsgLength = MSP_DISARM_MSG_SZ;

  } else {
    ucMSP_SET_RAW_RC_TINY[MSP_SET_RAW_RC_TINY_AILERON] = gunChannelData[AILERON];
    ucMSP_SET_RAW_RC_TINY[MSP_SET_RAW_RC_TINY_ELEVATOR] = gunChannelData[ELEVATOR];
    ucMSP_SET_RAW_RC_TINY[MSP_SET_RAW_RC_TINY_RUDDER] = gunChannelData[RUDDER];
    ucMSP_SET_RAW_RC_TINY[MSP_SET_RAW_RC_TINY_THROTTLE] = gunChannelData[THROTTLE];

    ucCRC = CalcMSPCRC(&(ucMSP_SET_RAW_RC_TINY[MSP_SET_RAW_RC_TINY_SIZE]), MSP_SET_RAW_RC_TINY_CRC_SZ);  //
    ucMSP_SET_RAW_RC_TINY[MSP_SET_RAW_RC_TINY_CRC] = ucCRC;

    ucCmd = MSP_SET_RAW_RC_TINY;
    ucMsgLength = MSP_SET_RAW_RC_TINY_MSG_SZ;
  }

//  printf("ucCRC : %x\n", ucCRC);

  switch(ucCmd) {
    case MSP_ARM :
      pucMSPMsg = ucMSP_ARM;
//      printf("[ARM]\n");
      break;

    case MSP_DISARM :
      pucMSPMsg = ucMSP_DISARM;
//      printf("[DISARM]\n");
      break;

    case MSP_SET_RAW_RC_TINY :
      pucMSPMsg = ucMSP_SET_RAW_RC_TINY;
//      printf("[SET_RAW_RC_TINY]\n");
      break;

    default :
      pucMSPMsg = "A"; ucMsgLength = 1;
      break;
  }
#if 1
  Serial.write(pucMSPMsg, ucMsgLength);
  // if data packet is sent too fast such as without delay, looks it's missed/not handled by the remote receiver (SkyCopter) 
  // And cauze fails to ARM/Disarm.
  delayMicroseconds(5000);  // Not necessary for serial to serial communication. but, when it's transferred via WiFiUDP packet, there is a delay and cauze serial data lost. Hence it's required.
#else

#if 1
  for( i = 0; i < ucMsgLength; i++) {
    // Send the msg to ESP8266
    Serial.write(pucMSPMsg[i]);
    delayMicroseconds(1000);  // Not necessary for serial to serial communication. but, when it's transferred via WiFiUDP packet, there is a delay and cauze serial data lost. Hence it's required.
  }
#else
  ullCurrentmS = millis();
  
  for(int j = 0; j < 4; j++) { // 8 * 2 = 16 * 50 = 800
    ullCounter++;
    
    Serial.write((char *)&ullCurrentmS, sizeof(ullCurrentmS));
    Serial.write((char *)&ullCounter, sizeof(ullCounter));
  }
  delayMicroseconds(1000);  // Not necessary for serial to serial communication. but, when it's transferred via WiFiUDP packet, there is a delay and cauze serial data lost. Hence it's required.  
#endif
#endif
}
