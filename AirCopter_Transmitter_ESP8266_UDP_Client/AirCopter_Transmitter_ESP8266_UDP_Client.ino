#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>


#define DEBUG_NK


/* Wemos d1 mini pin number
 *  
 * D0  IO  GPIO16
 * D1  IO, SCL GPIO5
 * D2  IO, SDA GPIO4
 * D3  IO, 10k Pull-up GPIO0
 * D4  IO, 10k Pull-up, BUILTIN_LED  GPIO2
 * D5  IO, SCK GPIO14
 * D6  IO, MISO  GPIO12
 * D7  IO, MOSI  GPIO13
 * D8  IO, 10k Pull-down, SS GPIO15
 *
 */

#define BAR_LED0  (D2)  // 4
#define BAR_LED1  (D3)  // 0
#define BAR_LED2  (D4)  // 2

#if 0
#include <SoftwareSerial.h>

const byte rxPin = D5; /* 14 */
const byte txPin = D6; /* 12 */

SoftwareSerial SerialS(D5, D6, false, 128);
#endif


const char* ssid     = "AIR0001";
const char* password = "";

const char* host = "192.168.4.1";
const uint16_t remote_port = 5000;

unsigned int localPort = 5000;       // local port to listen for UDP packets

#define MSP_SET_RAW_RC_SERIAL_MSG_SIZE  (11)
#define MSP_ARM_MSG_SIZE        (18)

#define RX_BUF_SZ     (64) // decrease the buffer size from 64 to 32 from the experiment

//시리얼 버퍼 초기화

char inData[RX_BUF_SZ]; // Allocate some space for the string
int SerialBufferCnt = 0; // Index into array; where to store the character

unsigned char ucBarLedState = 0x01;
unsigned long ulBarLedState;

unsigned int unRet;

unsigned long ullCounter = 0;
unsigned long ullErrorCounter = 0;
unsigned long ullCurrentmS = 0;



// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

void setup() {

  pinMode(BAR_LED0, OUTPUT);
  pinMode(BAR_LED1, OUTPUT);
  pinMode(BAR_LED2, OUTPUT);

  digitalWrite(BAR_LED0, LOW);
  digitalWrite(BAR_LED1, LOW);
  digitalWrite(BAR_LED2, LOW);

  for(int i = 0; i < 5; i++) {
    digitalWrite(BAR_LED0, HIGH);
    digitalWrite(BAR_LED1, HIGH);
    digitalWrite(BAR_LED2, HIGH);
    delay(100);
    digitalWrite(BAR_LED0, LOW);
    digitalWrite(BAR_LED1, LOW);
    digitalWrite(BAR_LED2, LOW);
    delay(100);
 }

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

#if 0
  SerialS.begin(38400); // 115200 bps is NOT stable on ESP8266(Wemos d1 mini) !!
#endif

  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);   //
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localPort); // return ERR_USE, if the spectified ipaddr and port are already bound to by another UDP PCB.
  
}


void loop() {


#if 1 /************************************************************************************************/
    //시리얼 통신으로 데이터가 들어왔다면
    while(Serial.available() > 0)
    {
        inData[SerialBufferCnt] = Serial.read(); //시리얼 데이터를 버퍼에 넣는다

        SerialBufferCnt++; // 버퍼카운터 증가
        if(SerialBufferCnt >= RX_BUF_SZ) {
#if 0
          if( ucBarLedState & 0x01 ) {
            digitalWrite(BAR_LED0, LOW);
            ucBarLedState = ucBarLedState & ~(0x01);            
          } else {
            digitalWrite(BAR_LED0, HIGH);
            ucBarLedState = ucBarLedState | (0x01);
          }
          if( ucBarLedState & 0x02 ) {
            digitalWrite(BAR_LED1, LOW);
            ucBarLedState = ucBarLedState & ~(0x02);
          } else {
            digitalWrite(BAR_LED1, HIGH);
            ucBarLedState = ucBarLedState | (0x02);
          }          
#endif
          // 입력이 빠르면 데이터가 계속 버려지는 문제 있음
          // 
          SerialBufferCnt = 0; // 버퍼가 64개가 넘어가면 버퍼카운터 초기화
        }
    }
  
    //시리얼 수신버퍼에 데이터가 있으면
    if(SerialBufferCnt > 0)
    {
        ullCurrentmS = millis();

        // beginPacket => connect() 함수는 ip 주소와 포트 정보만 저장, 함수 호출 여부가 큰 의미 없어 보임
        unRet = Udp.beginPacket(host,remote_port); //192.168.4.2는 휴대폰의 IP, 포트는 입력받은 포트
        if( unRet == 0 )
          Serial.printf("Packet begin error !!\n");
       
        inData[SerialBufferCnt] = 0;
        unRet = Udp.write(inData, SerialBufferCnt);
        if( unRet != SerialBufferCnt )
          Serial.printf("Packet write error !! (%d written)\n", unRet);

#if 1
        ulBarLedState++;        
        if( !(ulBarLedState % 10) ) {
          digitalWrite(BAR_LED0, HIGH);
        } else {
          digitalWrite(BAR_LED0, LOW);
        }
        if( !(ulBarLedState % 100) ) {
          digitalWrite(BAR_LED1, HIGH);
        } else {
          digitalWrite(BAR_LED1, LOW);
        }
        if( !(ulBarLedState % 1000) ) {
          digitalWrite(BAR_LED2, HIGH);
        } else {
          digitalWrite(BAR_LED2, LOW);
        }
#endif

        unRet = Udp.endPacket();
        if( unRet != 1 ) {
          ullErrorCounter++;
          Serial.printf("[%ld] Err:%d, %ld/%ld\n", ullCurrentmS, unRet, ullErrorCounter, ullCounter);
        } else {
          ullCounter++;
        }

        // 1. 수신 버퍼 크기와 무관한듯
        // 5 ms delayMicroseconds => 1097 sec, 43 회 에러, 26296576 bytes 전송, 64 bytes receiver buffer
        // 5 ms delayMicroseconds => 1091 sec, 47 회 에러, 26101888 bytes 전송, 8192 bytes receiver buffer
    
        // 2. delay time
        // 3 ms delayMicroseconds => No error ? 낮시간 WiFi Channel is clean ?, looks fine for the second run
        // 3 ms looks to be a bottom line !!
        // 0 ~ 2 ms delayMicroseconds => cause reset of remote system 
    
        // 3. packet size ( 8 * 4 = 32 bytes with 3 ms delays looks fine)
        // smaller size than 32 bytes doesn't make a problem too !!
        // 8 * 4 = 32 bytes with 3 ms delay => looks fine
        // 8 * 8 = 64 bytes with 3 ms delay => reset on target (last failed alloc call !!)

        // 4. 수신부 쪽 문제로 보임
        // 서버측, Serial.flush() 코드가 메모리 할당/관리에 문제를 읽으키는 것으로 보임
        // 빠른 반복적인 출력도 하나의 문제 가능성
        // delay 제거하여도 문제가 되지 않는 듯. 하지만 전송은 못하는 듯
        // 3 msec delay 조건에서도 662/198794 수준으로 전송 실패는 보임
        delayMicroseconds(3000);
    
        SerialBufferCnt = 0; //버퍼 카운터 초기화
    }

#else /************************************************************************************************/
    ullCurrentmS = millis();

    // beginPacket => connect() 함수는 ip 주소와 포트 정보만 저장, 함수 호출 여부가 큰 의미 없어 보임
    unRet = Udp.beginPacket(host,remote_port); //192.168.4.2는 휴대폰의 IP, 포트는 입력받은 포트
    if( unRet == 0 )
      Serial.printf("Packet begin error !!\n");

    // multiple Udp.write() call doesn't look making a problem. 
    // Problem is the size
    for(int j = 0; j < 4; j++) { // 8 * 2 = 16 * 50 = 800
      ullCounter++;

      unRet = Udp.write((char *)&ullCounter, sizeof(ullCounter));
      if( unRet != sizeof(ullCounter))
        Serial.printf("Packet write error !! (%d written)\n", unRet);

      unRet = Udp.write((char *)&ullCurrentmS, sizeof(ullCurrentmS));
      if( unRet != sizeof(ullCurrentmS))
        Serial.printf("Packet write error !! (%d written)\n", unRet);        
    }

    unRet = Udp.endPacket();
    if( unRet != 1 ) {
      ullErrorCounter++;
      Serial.printf("[%ld] Err:%d, %ld/%ld\n", ullCurrentmS, unRet, ullErrorCounter, ullCounter);
    }

    delayMicroseconds(3000);
#endif /************************************************************************************************/

}


