
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>
#include <hd44780.h>
#include <Wire.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <math.h>

//pin 설정


const uint8_t PIN_SCK = 18;  // SPI Clock 
const uint8_t PIN_MOSI = 23; //SPI Master Out
const uint8_t PIN_MISO = 19; //SPI Master In
const uint8_t PIN_SS = 4;    // SPI Chip Select
const uint8_t PIN_RST = 15;  // Reset
const uint8_t PIN_IRQ = 17;  //IRQ

#define SDA_PIN 27
#define SCL_PIN 26

hd44780_I2Cexp lcd;

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255


// message flow state
volatile byte expectedMsgId = POLL_ACK;

// 전송할 메세지 저장 변수
volatile byte last_transmitted_msg_id = POLL; 

// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

// timestamps to remember
uint64_t timePollSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;

// data buffer  “메시지 ID(1바이트), 앵커 주소(1바이트), 3개의 타임스탬프(각 5바이트)” 최소 17바이트
#define LEN_DATA 20
byte data[LEN_DATA];


//태그 프로토콜 상태 체크 변수
uint8_t F1,F2,F3,F4 = 0;  //F1 : poll 전송, F2 : poll_ACK 수신, F3 : RANGE 전송, F4 : RANGE 수신,  


// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 500;

// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;


// ---------- 추가: 앵커 주소 목록/인덱스 -----------
byte anchorAddresses[2] = {1, 2}; // A=1, B=2
byte currentAnchorIndex = 0;


// ---------- 추가: 앵커 간의 통신 주기를 제어하기 위한 타이머 -----------

unsigned long Blink = 50; // A 거리통신완료 1초 쉬고 B 랑 거리통신
//

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
    true,
    true,
    true,
    false,
    true
};



void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    Serial.println(F("### DW1000Ng-arduino-ranging-tag ###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println("DW1000Ng initialized ...");
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

    DW1000Ng::setNetworkId(10);
    DW1000Ng::setAntennaDelay(16436);
    
    Serial.println(F("Committed configuration ..."));
    
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    // attach callback for (successfully) sent and received messages
    DW1000Ng::attachSentHandler(handleSent);
    DW1000Ng::attachReceivedHandler(handleReceived);
    // anchor starts by transmitting a POLL message
    
    Wire.begin(SDA_PIN, SCL_PIN);
    int status = lcd.begin(16, 2);
    
    if (status) {
        hd44780::fatalError(status);
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hello, World!");
    lcd.setCursor(0, 1);
    lcd.print("ESP32 + LCD");


    transmitPoll();
    noteActivity();

    
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}


//  리셋
void resetInactive() {

    expectedMsgId = POLL_ACK;

    DW1000Ng::forceTRxOff();
    delayMicroseconds(300);
    transmitPoll();


    noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
}




// ---------- POLL에 "어느 앵커에게 보낼 것인지"를 data[1]에 담음 ----------
void transmitPoll() {
    data[0] = POLL;
    data[1] = anchorAddresses[currentAnchorIndex];
    last_transmitted_msg_id = POLL;

    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();  

    // POLL 전송시 F1 증가
    F1 ++;
    Serial.print("현재 프로토콜 F1, ");
    Serial.println(F1);
    
}




// ---------- Range 에 "어느 앵커에게 보낼 것인지"를 data[1]에 담음 ----------
void transmitRange() {
  
   // 1) msgId, anchorAddr 설정
    data[0] = RANGE;
    data[1] = anchorAddresses[currentAnchorIndex];

     // 2) Delayed TX 설정
    /* Calculation of future time */
    byte futureTimeBytes[LENGTH_TIMESTAMP];


    last_transmitted_msg_id = RANGE; // 전송할 메세지 아이다 저장

    //timeRangeSent 지연 전송 
    timeRangeSent = DW1000Ng::getSystemTimestamp();
    timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);
    timeRangeSent += DW1000Ng::getTxAntennaDelay();

   // 3) timePollSent, timePollAckReceived, timeRangeSent -> data[2-6] 5byte , data[7-11] 5byte, data[12-6] 5byte
    DW1000NgUtils::writeValueToBytes(data + 2, timePollSent, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(data + 7, timePollAckReceived, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(data + 12, timeRangeSent, LENGTH_TIMESTAMP);

    // 4) 실제 송신
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);

    // POLL 전송시 F3 증가
    F3 ++;

    Serial.print("현재 프로토콜 F3, ");
    Serial.println(F3);

    //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void printToLCD(String dir, float d) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dir: ");
    lcd.print(dir);
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print(d, 2);
    lcd.print("m");
}

double cal_A(float x) {
    double a = -0.1848;
    double b =  0.3507;
    double c =  0.6857;
    double d =  0.6543;

    return a * x * x * x + b * x * x + c * x + d;
}

double cal_B(float x) {
    double a =  0.0206;
    double b = -0.0543;
    double c =  0.7641;
    double d =  0.6543;

    return a * x * x * x + b * x * x + c * x + d;
}


double calculate_vertical_distance(double d1, double d2, double D) {
    double x = 0.5 * (d1*d1 + d2*d2 - (2*D)*(2*D));
    double y = sqrt(fabs(x));
    return fabs(y);
}

// 통신 실패시 해당 앵커와 재시도
int retryCount = 0;
const int MAX_RETRY = 1;

const int F1_Limit = 15;  // 원하는 임계값 설정

double distanceA = -1;
double distanceB = -1;

float distanceA_RAW = -1;
float distanceB_RAW = -1;


double straightDist = 0; 
String dir = "Normal";

void loop() {

    if(F1 == 255) F1 = 0;
    if(F2 == 255) F2 = 0;
    if(F3 == 255) F3 = 0;
    if(F4 == 255) F4 = 0;

    if(F1 > F1_Limit) {

        Serial.println("TAG restart");
        delay(100);
        ESP.restart();
    }
    

    // 1) 송수신 인터럽트 모두 체크 안됨 (송수신 이벤트 안일어남)
    
   if (!sentAck && !receivedAck) {
        // check if inactive
        if (millis() - lastActivity > resetPeriod) {


            Serial.print("리셋, 프로토콜 모니터링 :  ");
            Serial.print(F1);
            Serial.print(F2);
            Serial.print(F3);
            Serial.println(F4);

            currentAnchorIndex = (currentAnchorIndex + 1) % 2;
            resetInactive();

        }
        return;
    }


    // poll  or range 보낸거 확인
    if (sentAck) {

        sentAck = false;
        DW1000Ng::startReceive();
        
    }


    // poll_ack, or range report 받은거 확인
    if (receivedAck) {
        receivedAck = false;

        // get message and parse
        DW1000Ng::getReceivedData(data, LEN_DATA);
        
        byte msgId = data[0];      // 수신 메시지에 담긴 종류(poll_ACK, rangereport)
        byte anchorAddr = data[1]; // 수신 메시지에 담긴 앵커 주소
        
        // msgId 가 expectedMsgId 가 아니면 다시 시작
        if (msgId != expectedMsgId) {
            
            // unexpected message, start over again
            Serial.print("Received wrong message # "); Serial.println(msgId);
            expectedMsgId = POLL_ACK;
            transmitPoll();
            noteActivity();
            
            return;
        }

        
        if (msgId == POLL_ACK && last_transmitted_msg_id == POLL) {
            
            timePollSent = DW1000Ng::getTransmitTimestamp();       //poll송신했던 시간 불러옴 (Transmit time)
            timePollAckReceived = DW1000Ng::getReceiveTimestamp();  //poll_ack 수신한 시간 기록
            

            expectedMsgId = RANGE_REPORT;    //POLL_ACK를 받은 경우 다음 기대 메세지는 RANGE_REPORT

            F2 ++;

            Serial.print(" 현재 프로토콜 F2 : ");
            Serial.println(F2);

            transmitRange();
            noteActivity();
        } 
        
        else if (msgId == RANGE_REPORT && last_transmitted_msg_id == RANGE) {

            expectedMsgId = POLL_ACK;

            F4++;
            
            Serial.print(" 현재 프로토콜 F4 : ");
            Serial.println(F4);
            Serial.print("Got RANGE_REPORT from anchor=");
            Serial.print(anchorAddr);
            

            
            // 거리값 읽어오기
            float distance = 0;
            memcpy(&distance, data + 2, 4);  // 앵커가 float을 data[2~5]에 보내는 경우
            Serial.print("@");
            Serial.println(distance);

            if (anchorAddr == 1) {
        
                distanceA_RAW = distance;
                distanceA = cal_A(distance);
            } 
            
            else if (anchorAddr == 2) {
                distanceB_RAW = distance;
                distanceB = cal_B(distance);
            }

            // 두 값 모두 준비됐으면 좌우, 직선 계산
            if (distanceA >= 0 && distanceB >= 0) {
                

                if (abs(distanceA_LOW - distanceB_RAW) < 0.15) {
                    dir = "STRAIGHT";
            
                } else if (distanceA_LOW  > distanceB_RAW ) {
                    dir = "LEFT";
                }
            
                else {
                dir = "RIGHT";
                    }

                

                float StraightDist = calculate_vertical_distance(distanceA, distanceB, 0.42);

                Serial.print("Direction: ");
                Serial.println(dir);
                Serial.print("Straight distance: ");
                Serial.println(StraightDist, 2);
                printToLCD(dir, StraightDist);

                
                // 초기화
                distanceA = -1;
                distanceB = -1;
    
            }

            
            
            
        
             //어댑티브 Blink 조절은 수신 성공 후에만

            float rxPower = DW1000Ng::getReceivePower();
            if (rxPower < -90) {
                Blink = 200;
                } 

            else if (rxPower > -80) {
                Blink = 200;
                }


             //--- 다음 앵커로 넘어가도록 인덱스 변경 ---
            
            
            
            Serial.print("주소 변경, F 초기화 ");
            
            F1=F2=F3=F4= 0;
            retryCount = 0;
            currentAnchorIndex = (currentAnchorIndex + 1) % 2;

            delay(Blink);

            transmitPoll();
            noteActivity();
            

        }
        
        else if (msgId == RANGE_FAILED) {

            expectedMsgId = POLL_ACK;
            Serial.println("Recieved : RANGE_FAILED");

            if (retryCount < MAX_RETRY) {
                retryCount++;
                Serial.println("Retrying...");
                delay(Blink/2);  // 짧게 쉬고 한 번 더 시도
                transmitPoll();    // 같은 앵커에 재시도
                noteActivity();
                return;
            }
            
            retryCount = 0; // 재시도도 실패하면 다음 앵커로
            currentAnchorIndex = (currentAnchorIndex + 1) % 2;

            Serial.print("주소 변경, F 초기화 ");
            F1=F2=F3=F4= 0;
            delay(Blink);
            
            transmitPoll();
            noteActivity();
        }
    }

}





