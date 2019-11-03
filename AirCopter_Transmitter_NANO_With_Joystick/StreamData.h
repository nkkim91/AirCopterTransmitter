#include "config.h"

#define MAX_MSG_BUF_SIZE  (64)

#define MAX_TX_STREAM_LEN			26
#define MAX_TX_CHANNEL_BUF_LEN		22
#define MAX_TX_CHANNEL				16

#define DEFAULT_XMIT_DELAY	(4000)

//Channel MIN MAX values
#define CHANNEL_MAX_100  1844  //  100%
#define CHANNEL_MIN_100 204   //  100%
#define CHANNEL_MAX_125 2047  //  125%
#define CHANNEL_MIN_125 0   //  125%

#define CHANNEL_MIN_COMMAND 784   // 1350us
#define CHANNEL_SWITCH    1104  // 1550us
#define CHANNEL_MAX_COMMAND 1424  // 1750us

#define SPURIOUS_IRQ_FILTER_TIME  (50)
#define BUTTON_EVENT_DURATION (50)

#define TIMER1_1SEC   (0x3E80)

#define BUTTON_HIGH   (1)
#define BUTTON_LOW    (0)

#if defined(CONFIG_ESP8266_TRANSMITTER)


#define MSP_SET_RAW_RC_TINY   (0x96)
#define MSP_ARM               (0x97)
#define MSP_DISARM            (0x98)
#define MSP_MAG_CALIBRATION   (0xCE)


#define MSP_SET_RAW_RC_TINY_MSG_SZ  (11)
#define MSP_ARM_MSG_SZ  (18)
#define MSP_DISARM_MSG_SZ  (18)

#define MSP_SET_RAW_RC_TINY_CRC_SZ  (7)
#define MSP_ARM_CRC_SZ  (2)
#define MSP_DISARM_CRC_SZ  (2)


#define AILERON_MIN   (  0)
#define AILERON_MID   (127)
#define AILERON_MAX   (255)

#define ELEVATOR_MIN  ( 0)
#define ELEVATOR_MID  (127)
#define ELEVATOR_MAX  (255)

#define THROTTLE_MIN  (  0)
#define THROTTLE_MID  (127)
#define THROTTLE_MAX  (255)

#define RUDDER_MIN    (  0)
#define RUDDER_MID    (127)
#define RUDDER_MAX    (255)

#if 1
#define JOYSTICK_AILERON_MIN  (0)
#define JOYSTICK_AILERON_MID  (511)
#define JOYSTICK_AILERON_MAX  (1023)

#define JOYSTICK_ELEVATOR_MIN (0)
#define JOYSTICK_ELEVATOR_MID (511)
#define JOYSTICK_ELEVATOR_MAX (1023)

#define JOYSTICK_THROTTLE_MIN (0)
#define JOYSTICK_THROTTLE_MID (511)
#define JOYSTICK_THROTTLE_MAX (1023)

#define JOYSTICK_RUDDER_MIN   (0)
#define JOYSTICK_RUDDER_MID   (511)
#define JOYSTICK_RUDDER_MAX   (1023)
#endif
#endif

#define JOYSTICK_INPUT_MAX	(32767)
#define CHANNEL_MINUS125P_VALUE	(0)
#define CHANNEL_MINUS100P_VALUE	(204)
#define CHANNEL_PLUSM100P_VALUE	(1843)
#define CHANNEL_PLUSM125P_VALUE	(2047)
#define CHANNEL_MAX_VALUE	(CHANNEL_PLUSM100P_VALUE - CHANNEL_MINUS100P_VALUE + 1)	// 1640

#define BIND_TIME_OUT	(2)	// 3 Sec

//#define DEBUG_NK

#ifdef ARDUINO
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed int int16_t;
typedef unsigned int uint16_t;
typedef long int int32_t;
typedef long unsigned int uint32_t;
#else /* X86 */
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
#endif

#if defined(CONFIG_ESP8266_TRANSMITTER)
enum eChanTxProtocol {
  AILERON = 0,
  ELEVATOR = 1,
  THROTTLE = 2,
  RUDDER = 3,
  ARM = 4,
  TRIM_THROTTLE = 5,
  TRIM_YAW = 6,  
  TRIM_PITCH = 7,
  TRIM_ROLL = 8,
  DISARM = 9,
};
#endif

enum eState {
	BIND_YET = 0,
	BIND_DONE = 1,
	BIND_UNKNOWN = 2,
};

enum eMspMsgSetRawRcTinyIndex {
  MSP_SET_RAW_RC_TINY_SIZE = 3,
  MSP_SET_RAW_RC_TINY_CMD = 4,
  MSP_SET_RAW_RC_TINY_AILERON = 5,
  MSP_SET_RAW_RC_TINY_ELEVATOR = 6,  
  MSP_SET_RAW_RC_TINY_RUDDER = 7,
  MSP_SET_RAW_RC_TINY_THROTTLE = 8,
  MSP_SET_RAW_RC_TINY_CRC = 10,
};

enum eMspMsgArmIndex {
  MSP_ARM_SIZE = 3,
  MSP_ARM_CMD = 4,
  MSP_ARM_CRC = 5,
};

enum eMspMsgDisArmIndex {
  MSP_DISARM_SIZE = 3,
  MSP_DISARM_CMD = 4,
  MSP_DISARM_CRC = 5,
};

enum eMspArmState {
  MSP_ARMED = 1,
  MSP_DISARMED = 2,
};


/* Bit 0..4 */
enum eSubProtocol {
	SUB_PROTOCOL_V2X2 = 5,
	SUB_PROTOCOL_MIN = SUB_PROTOCOL_V2X2,
	SUB_PROTOCOL_SYMAX = 10,
	SUB_PROTOCOL_MAX = SUB_PROTOCOL_SYMAX,
};
#define SUB_PROTOCOL_MASK (0x1F)

/* Bit 5 */
enum eRangCheck {
  RANGE_CHECK_NO = 0,
  RANGE_CHECK_YES = 1,
};

/* Bit 6 */
enum eAutoBind {
  AUTO_BIND_OFF = 0,
  AUTO_BIND_ON = 1,
};

/* Bit 7 */
enum eBind {
  BIND_OFF = 0,
  BIND_ON = 1,
};

#define RX_NUM_MASK (0x0F)

/* Bit 4..6 */
enum eSubProtocolType {
	SUB_PROTOCOL_TYPE_SYMAX = 0,
	SUB_PROTOCOL_TYPE_SYMAX5C = 1,
	SUB_PROTOCOL_TYPE_V2X2 = 0,
	SUB_PROTOCOL_TYPE_JXD506 = 1,
};
#define SUB_PROTOCOL_TYPE_MASK (0x07)

/* Bit 7 */
enum ePower {
  POWER_HIGH = 0,
  POWER_LOW = 1,
};



enum InputSourceType {
	IST_REG_FILE = 0,
	IST_CHAR_DEV = 1,
};


struct TxStream {
	uint8_t ucHead;
	uint8_t ucSubProtocol;
	uint8_t	ucType;
	uint8_t	ucOptionProtocol;
	uint8_t ucChan[MAX_TX_CHANNEL_BUF_LEN];	// 22(x unsigned char) => 16 (x unsigned short)
};

#define TX_STREAM_SIZE	sizeof(struct TxStream)

union TxStreamData {
	struct TxStream stTS;
	uint8_t ucByte[TX_STREAM_SIZE];
};

#define TS_HEAD			stTS.ucHead
#define TS_SPROTOCOL	stTS.ucSubProtocol
#define TS_TYPE			stTS.ucType
#define TS_OPROTOCOL 	stTS.ucOptionProtocol
#define TS_CH(x)		stTS.ucChan[x]

