//#include "config.h"
#define JOYSTICK_CALIBRATION_TIME (7000)    // 7 sec
#define DEBUG_LED_BLINK_PERIOD  (100)       // 100 ms

#define JS_Throttle_Pin (A0)
#define JS_RUDDER_Pin   (A1)
#define JS_ELEVATOR_Pin (A2)
#define JS_AILERON_Pin  (A3)

int JS_THR_MIN_RAW, JS_THR_MAX_RAW;
int JS_RUD_MIN_RAW, JS_RUD_MAX_RAW;
int JS_ELE_MIN_RAW, JS_ELE_MAX_RAW;
int JS_AIL_MIN_RAW, JS_AIL_MAX_RAW;

unsigned long ulButtonEventStartTime;
unsigned long ulCalibrationTime;

int nCurrentMSPArmState = MSP_DISARMED; // 

int DEBUG_LED_Pin = 7;
int JS_Button_Pin = 3;
int JS_Button_Event = 0;

extern unsigned int gunChannelData[MAX_TX_CHANNEL];

void ToggleDebugLed()
{
  static int toggle = 0;

  if( toggle++ % 2 ) {
    digitalWrite(DEBUG_LED_Pin, HIGH);
    
  } else {
    digitalWrite(DEBUG_LED_Pin, LOW);
  }
}

void JS_Setup()
{
  int nThrottleRawData = 0;
  int nThrottleAdjData = 0;

  int nRudderRawData = 0;
  int nRudderAdjData = 0;

  int nElevatorRawData = 0;
  int nElevatorAdjData = 0;

  int nAileronRawData = 0;
  int nAileronAdjData = 0;

  int LedBlink;

  pinMode(JS_Throttle_Pin, INPUT);
  pinMode(JS_RUDDER_Pin, INPUT);
  pinMode(JS_ELEVATOR_Pin, INPUT);
  pinMode(JS_AILERON_Pin, INPUT);

  pinMode(JS_Button_Pin, INPUT_PULLUP);

  pinMode(DEBUG_LED_Pin, OUTPUT);
  digitalWrite(DEBUG_LED_Pin, LOW);

#ifdef JOYSTICK_BUTTON
  attachInterrupt(digitalPinToInterrupt(JS_Button_Pin), JS_Button_Handler, FALLING);
#endif /* JOYSTICK_BUTTON */

  JS_THR_MIN_RAW = 1023, JS_THR_MAX_RAW = 0;
  JS_RUD_MIN_RAW = 1023, JS_RUD_MAX_RAW = 0;
  JS_ELE_MIN_RAW = 1023, JS_ELE_MAX_RAW = 0;
  JS_AIL_MIN_RAW = 1023, JS_AIL_MAX_RAW = 0;

#ifdef DEBUG_NK
  if( gunDebugMask & NK_DEBUG_INFO ) {
    Serial.println("Start joystick calibration - Move joysticks to its max and min for 5 seconds. !!");
  }
#endif // DEBUG_NK

  ulCalibrationTime = millis();

  digitalWrite(DEBUG_LED_Pin, LOW);
  
  for(LedBlink = millis(); millis() - ulCalibrationTime < JOYSTICK_CALIBRATION_TIME ; ) {  // 5 Sec

    nThrottleRawData = analogRead(JS_Throttle_Pin); 
    delayMicroseconds(1000);
    
    nRudderRawData = analogRead(JS_RUDDER_Pin);
    delayMicroseconds(1000);

    nElevatorRawData = analogRead(JS_ELEVATOR_Pin);
    delayMicroseconds(1000);
        
    nAileronRawData = analogRead(JS_AILERON_Pin);
    delayMicroseconds(1000);    

    if( JS_THR_MAX_RAW < nThrottleRawData ) JS_THR_MAX_RAW = nThrottleRawData;
    if( JS_THR_MIN_RAW > nThrottleRawData ) JS_THR_MIN_RAW = nThrottleRawData;

    if( JS_RUD_MAX_RAW < nRudderRawData ) JS_RUD_MAX_RAW = nRudderRawData;
    if( JS_RUD_MIN_RAW > nRudderRawData ) JS_RUD_MIN_RAW = nRudderRawData;

    if( JS_ELE_MAX_RAW < nElevatorRawData ) JS_ELE_MAX_RAW = nElevatorRawData;
    if( JS_ELE_MIN_RAW > nElevatorRawData ) JS_ELE_MIN_RAW = nElevatorRawData;

    if( JS_AIL_MAX_RAW < nAileronRawData ) JS_AIL_MAX_RAW = nAileronRawData;
    if( JS_AIL_MIN_RAW > nAileronRawData ) JS_AIL_MIN_RAW = nAileronRawData;

    if( millis() - LedBlink > DEBUG_LED_BLINK_PERIOD ) {

      ToggleDebugLed();

      printf("Calibration : %4d / %4d \n", JS_THR_MIN_RAW, JS_THR_MAX_RAW);

      LedBlink = millis();
    }

  }

#ifdef DEBUG_NK
  if( gunDebugMask & NK_DEBUG_INFO ) {
    Serial.print("THR(Min/Max) : "); Serial.print(JS_THR_MIN_RAW); Serial.print("/"); Serial.print(JS_THR_MAX_RAW); Serial.print("("); Serial.print(JS_THR_MAX_RAW-JS_THR_MIN_RAW); Serial.println(")"); 
    Serial.print("RUD(Min/Max) : "); Serial.print(JS_RUD_MIN_RAW); Serial.print("/"); Serial.print(JS_RUD_MAX_RAW); Serial.print("("); Serial.print(JS_RUD_MAX_RAW-JS_RUD_MIN_RAW); Serial.println(")"); 
    Serial.print("ELE(Min/Max) : "); Serial.print(JS_ELE_MIN_RAW); Serial.print("/"); Serial.print(JS_ELE_MAX_RAW); Serial.print("("); Serial.print(JS_ELE_MAX_RAW-JS_ELE_MIN_RAW); Serial.println(")"); 
    Serial.print("AIL(Min/Max) : "); Serial.print(JS_AIL_MIN_RAW); Serial.print("/"); Serial.print(JS_AIL_MAX_RAW); Serial.print("("); Serial.print(JS_AIL_MAX_RAW-JS_AIL_MIN_RAW); Serial.println(")"); 
  }
#endif // DEBUG_NK

#ifdef DEBUG_NK
  if( gunDebugMask & NK_DEBUG_INFO ) {
    printf("Joystick calibration Done !!\n");
  }
#endif // DEBUG_NK
}


#ifdef JOYSTICK_BUTTON
void JS_Button_Handler(void)
{
  unsigned long ulStartTime;
  unsigned int unButtonState;
  unsigned int unSavedTCNT1;

#ifdef DEBUG_NK
  if( gunDebugMask & NK_DEBUG_DEBUG ) {
    printf("[+] JS_Button_Handler\n");
  }
#endif // DEBUG_NK

  for(TCNT1=0, ulStartTime = millis(); (unButtonState = digitalRead(JS_Button_Pin)) != BUTTON_HIGH && (unSavedTCNT1 = TCNT1) < TIMER1_1SEC; ) { // 0x3E80 (16000) = 16000000 / 1024 = 16000
  }

#ifdef DEBUG_NK
  if( gunDebugMask & NK_DEBUG_DEBUG ) {
    printf("%d\n", TCNT1);
  }
#endif // DEBUG_NK

  if( unSavedTCNT1 < TIMER1_1SEC ) {  // TIMER1_1SEC : 0x3E80 (16000)

    if( millis() - ulLastIRQTime <= SPURIOUS_IRQ_FILTER_TIME ) {  // 50 ms
#ifdef DEBUG_NK
      if( gunDebugMask & NK_DEBUG_DEBUG ) {
        printf("[-] JS_Button_Handler - Spurious\n");
      }
#endif // DEBUG_NK
      return;
    } else {

      if( JS_Button_Event == 0 ) {
        JS_Button_Event = 1;
        ulLastIRQTime = millis();
        ulButtonEventStartTime = millis();
      } else {
        // if it's the case, something wrong (?)
//        JS_Button_Event = 0;
      }
    }
  }

#ifdef DEBUG_NK
  if( gunDebugMask & NK_DEBUG_DEBUG ) {
    printf("[-] JS_Button_Handler\n");
  }
#endif // DEBUG_NK
}
#endif  /* JOYSTICK_BUTTON */

void JS_Input()
{

  int nThrottleRawData = 0;
  int nThrottleAdjData = 0;

  int nRudderRawData = 0;
  int nRudderAdjData = 0;

  int nElevatorRawData = 0;
  int nElevatorAdjData = 0;

  int nAileronRawData = 0;
  int nAileronAdjData = 0;
  
  nThrottleRawData = analogRead(JS_Throttle_Pin);
  
  if( nThrottleRawData < JS_THR_MIN_RAW ) nThrottleRawData = JS_THR_MIN_RAW;
  if( nThrottleRawData > JS_THR_MAX_RAW ) nThrottleRawData = JS_THR_MAX_RAW;

  nThrottleRawData = JS_THR_MAX_RAW - nThrottleRawData + JS_THR_MIN_RAW;
  nThrottleAdjData = map(nThrottleRawData, JS_THR_MIN_RAW, JS_THR_MAX_RAW, THROTTLE_MIN, THROTTLE_MAX);  

  nRudderRawData = analogRead(JS_RUDDER_Pin);

  if( nRudderRawData < JS_RUD_MIN_RAW ) nRudderRawData = JS_RUD_MIN_RAW;
  if( nRudderRawData > JS_RUD_MAX_RAW ) nRudderRawData = JS_RUD_MAX_RAW;
  nRudderRawData = JS_RUD_MAX_RAW - nRudderRawData + JS_RUD_MIN_RAW;
  nRudderAdjData = map(nRudderRawData, JS_RUD_MIN_RAW, JS_RUD_MAX_RAW, RUDDER_MIN, RUDDER_MAX);  

  nElevatorRawData = analogRead(JS_ELEVATOR_Pin);

  if( nElevatorRawData < JS_ELE_MIN_RAW ) nElevatorRawData = JS_ELE_MIN_RAW;
  if( nElevatorRawData > JS_ELE_MAX_RAW ) nElevatorRawData = JS_ELE_MAX_RAW;
  nElevatorRawData = JS_ELE_MAX_RAW - nElevatorRawData + JS_ELE_MIN_RAW;
  nElevatorAdjData = map(nElevatorRawData, JS_ELE_MIN_RAW, JS_ELE_MAX_RAW, ELEVATOR_MIN, ELEVATOR_MAX);  

  nAileronRawData = analogRead(JS_AILERON_Pin);

  if( nAileronRawData < JS_AIL_MIN_RAW ) nAileronRawData = JS_AIL_MIN_RAW;
  if( nAileronRawData > JS_AIL_MAX_RAW ) nAileronRawData = JS_AIL_MAX_RAW;
  nAileronRawData = JS_AIL_MAX_RAW - nAileronRawData + JS_AIL_MIN_RAW;
  nAileronAdjData = map(nAileronRawData, JS_AIL_MIN_RAW, JS_AIL_MAX_RAW, AILERON_MIN, AILERON_MAX);  

#ifdef DEBUG_NK
  if( gunDebugMask & NK_DEBUG_DEBUG ) {
    printf("Raw(TREA), Adj(TREA) - (%4d %4d %4d %4d), (%4d %4d %4d %4d)\n", nThrottleRawData, nRudderRawData, nElevatorRawData, nAileronRawData, nThrottleAdjData, nRudderAdjData, nElevatorAdjData, nAileronAdjData);
  }
#endif // DEBUG_NK

  gunChannelData[THROTTLE] = nThrottleAdjData;
  gunChannelData[RUDDER]   = nRudderAdjData;
  gunChannelData[ELEVATOR] = nElevatorAdjData;
  gunChannelData[AILERON]  = nAileronAdjData;

#ifdef JOYSTICK_BUTTON
  if( JS_Button_Event ) {
    if( millis() - ulButtonEventStartTime > BUTTON_EVENT_DURATION ) { // BUTTON_EVENT_DURATION : 50
      if( nCurrentMSPArmState == MSP_DISARMED ) {
        gunChannelData[ARM] = CHANNEL_MAX_100; /* CHANNEL_MAX_100 = 1844 */
        nCurrentMSPArmState = MSP_ARMED;
//        printf("Button ARMED\n");
      } else if( nCurrentMSPArmState == MSP_ARMED ) {
        gunChannelData[DISARM] = CHANNEL_MAX_100; /* CHANNEL_MAX_100 = 1844 */
        nCurrentMSPArmState = MSP_DISARMED;
//        printf("Button DisARMED\n");        
      }
      JS_Button_Event = 0;
#ifdef DEBUG_NK
      if( gunDebugMask & NK_DEBUG_DEBUG ) {
        printf("%d ) ## NK Button Event On !!\n", micros());
      }
#endif // DEBUG_NK
    }
  } else {
//    printf("JS_Button_Event : %d,  nCurrentMSPArmState : %d\n", JS_Button_Event, nCurrentMSPArmState);
    if( nCurrentMSPArmState == MSP_ARMED ) {
      gunChannelData[ARM] = CHANNEL_MIN_100; /* CHANNEL_MIN_100 = 204 */
//      printf("Button ARMED released\n");
    } else if( nCurrentMSPArmState == MSP_DISARMED ) {
      gunChannelData[DISARM] = CHANNEL_MIN_100; /* CHANNEL_MIN_100 = 204 */    
//      printf("Button DISARMED released\n");      
    }
    if( gunDebugMask & NK_DEBUG_DEBUG ) {
        printf("%d ) ## NK Button Event Off !!\n", micros());
    }
  }
#endif /* JOYSTICK_BUTTON */
}
