/*
 * 2015.08.04 최종버전
 * multiwii 보드와 통신하는 최신버전
 */
//#define DEBUG_NK

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

char MySSID[] = "AIR0001"; //SSID
//char MyPASS[] = "AIR00001"; //PASS


char PHONE_IP[] = "192.168.4.2"; //핸드폰 연결시 핸드폰의 IP주소

unsigned int localPort = 5000; //UDP 포트

WiFiUDP udp; //UDP통신 초기화

//시리얼 버퍼 초기화
char inData[64]; // Allocate some space for the string, NK - recommend to increase it ?

int SerialBufferCnt = 0; // Index into array; where to store the character

unsigned int PhonePort = 50000;//초반에 연결되는 포트

unsigned long ulLastLedUpdate = millis();
unsigned long ulLastPrintUpdate = millis();
unsigned int unLedState = 0;

unsigned char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,

char  replyPacket[] = "Hi there! Got the message :-)";  // a reply string to send back

void setup()
{
  //시리얼 통신 초기화
  Serial.begin(115200);//시리얼 통신속도

  //wifi 초기화
  //WiFi.softAP(MySSID,MyPASS);
  WiFi.softAP(MySSID);
  IPAddress myIP = WiFi.softAPIP(); //192.168.4.1 소프트 AP연결시 기본 IP

  pinMode(2, OUTPUT);

  //udp 초기화
  udp.begin(localPort);

  delay(1000);//초기화 후 1초정도 기다려 준다
}


void loop() 
{

  if( millis() - ulLastLedUpdate > 1000 ) {
    if( unLedState ) {
      unLedState = 0;
    } else {
      unLedState = 1;
    }
    digitalWrite(2, unLedState);

    ulLastLedUpdate = millis();
  }
  
  // wifi -> udp로 전송된 데이터 처리부분
  // wifi -> udp -> 시리얼 통신으로 토스
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
//    PhonePort = udp.remotePort(); //수신받은 포트를 저장해둔다
    int n = udp.read(packetBuffer, 64 /* UDP_TX_PACKET_MAX_SIZE */);
    if( n > 0 ) {      
      packetBuffer[n] = 0;
      Serial.write(packetBuffer, n); 

      // flush looks making a problem in the memory allocation 
      // Not allowed to call for synchronous operation
//    Serial.flush(); 
    }
  }

#ifdef DEBUG_NK
  if( millis() - ulLastPrintUpdate > 1000 ) {
    Serial.printf("[packetSize:%d]\n", packetSize); 
    Serial.printf("ESP Free heap size : %d\n", ESP.getFreeHeap());
    ulLastPrintUpdate = millis();
  }
#endif
  
  // 휴대폰으로 상태 정보를 데이터를 전송할 경우 필요
#if 1
  //시리얼 통신으로 데이터가 들어왔다면
  while(Serial.available() > 0)
  {
      delayMicroseconds(100);
      inData[SerialBufferCnt] = Serial.read(); //시리얼 데이터를 버퍼에 넣는다

      SerialBufferCnt++; // 버퍼카운터 증가
      if(SerialBufferCnt >= 64) SerialBufferCnt = 0; //버퍼가 64개가 넘어가면 버퍼카운터 초기화
  }

  //시리얼 수신버퍼에 데이터가 있으면
  if(SerialBufferCnt > 0)
  {
      udp.beginPacket(PHONE_IP,PhonePort); //192.168.4.2는 휴대폰의 IP, 포트는 입력받은 포트
      for (int i=0;i<SerialBufferCnt;i++)
      {
          udp.write(inData[i]); //시리얼 -> udp 로 전송
      }
      udp.endPacket();
      
      SerialBufferCnt = 0; //버퍼 카운터 초기화
  }
#endif
}
