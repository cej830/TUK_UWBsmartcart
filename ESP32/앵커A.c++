#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

// connection pins
const uint8_t PIN_SCK = 18;  // SPI Clock 
const uint8_t PIN_MOSI = 23; //SPI Master Out
const uint8_t PIN_MISO = 19; //SPI Master In
const uint8_t PIN_SS = 4;    // SPI Chip Select
const uint8_t PIN_RST = 15;  // Reset
const uint8_t PIN_IRQ = 17;  //IRQ

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

// message flow state
volatile byte expectedMsgId = POLL;

// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

// protocol error state
boolean protocolFailed = false;

// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;
uint64_t timeComputedRange;

// last computed range/time
// data buffer


// 앵커 주소 설정과, 데이터 버퍼 크기 설정
#define ackAddress 1
#define LEN_DATA  20
byte data[LEN_DATA];

// 기대 메세지가 Range일경우 시간 체크용 변수
uint32_t rangeWaitStart = 0;

// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 500;

// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

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
    
    Serial.begin(115200);  //PC 디버깅용 통신

    delay(1000);
    Serial.println(F("### DW1000Ng-arduino-ranging-anchor ###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

    DW1000Ng::setDeviceAddress(1); // 내 주소=1  
    data[1] = ackAddress;

    
    DW1000Ng::setAntennaDelay(16530);   /// 안테나 딜레이 조절하면서 1m 값 정확히 맞추기 (태그는 고정 각각 앵커부터 건드리기)
    
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
    // anchor starts in receiving mode, awaiting a ranging poll message
   
    receiver();
    noteActivity();
    
    // for first time ranging frequency computation
    rangingCountPeriod = millis();
}





// 마지막 행동 시간 기록 함수
void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}


// 리셋 행동 실행 함수
void resetInactive() {
  
    // anchor listens for POLL
    expectedMsgId = POLL;   // 기대 메세지 POLL로 변경하고 리셋상태 진입

    receiver();
    noteActivity();
}



// 송신 인터럽트
void handleSent() {
    // status change on sent success

    sentAck = true;
}


// 수신 인터럽트
void handleReceived() {
    // status change on received success
    
    receivedAck = true;
}


// POLL_ACK 보내기
void transmitPollAck() {
    data[0] = POLL_ACK;
    

    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}


// Range_Report 보내기 
void transmitRangeReport(float curRange) {
    data[0] = RANGE_REPORT;
    
    
    // write final ranging result
    // curRange을 data[2..5] (float 4바이트)에 기록

    memcpy(data + 2, &curRange, 4);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}


// 거리 측정 실패  보내기
void transmitRangeFailed() {

    data[0] = RANGE_FAILED;
    
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}


// Receive 모드로 돌아가기
void receiver() {
  
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually

    // 수신 끄고 300u초 기다리고 다시 수신 시작
    delayMicroseconds(300);

    DW1000Ng::startReceive();
}


// UART 통신 함수
void sendDistance(float distance) {

    
  // PC 콘솔용 전송
  Serial.print("R: ");
  Serial.print(distance, 4);
  Serial.print(" @ ");
  Serial.println(millis());


}

// UART 통신 함수
void sendDistanceFailed() {

   
    // PC 콘솔용 전송
    Serial.print("R: ");
    Serial.print(9999);
    Serial.print(" @ ");
    Serial.println(millis());
  
  
  }


// UART 통신 함수
void sendReset() {


    // PC 콘솔용 전송
    Serial.print("R: ");
    Serial.print("nosig");
    Serial.print(" @ ");
    Serial.println(millis());
  
  
  }





void loop() {
  
    int32_t curMillis = millis();

    if (!sentAck && !receivedAck) {   // 인터럽트 둘다 FALSE -> 

        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {  //500ms 이상 응답 없으면 리셋

            sendReset();
            resetInactive();
        }
        return;
    }

    
    // continue on any success confirmation
    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];

        if (msgId == POLL_ACK) {
            timePollAckSent = DW1000Ng::getTransmitTimestamp();
            noteActivity();

        }

        DW1000Ng::startReceive();
    }
    

    if (receivedAck) {   //수신 인터럽트가 발생했을때
        receivedAck = false;

        // get message and parse  data[0] = 메세지 타입 정보 , data[1] = 타겟 주소
        DW1000Ng::getReceivedData(data, LEN_DATA);  //DW1000 데이터 ESP data 버퍼에 복사하기  



        byte targetAddr = data[1];    

        
        // 타켓 주소랑 ESP 설정 주소랑 비교
        if (targetAddr != ackAddress) { //태그와 주소 다를시 수신 상태로 돌아가기 
        
        //수신 모드로 돌아가기
           
            resetInactive();  
            return;         
        }

        
        byte msgId = data[0];

        if (msgId != expectedMsgId) {  //기대 메세지  : poll or range

            // unexpected message, start over again (except if already POLL)
            protocolFailed = true;

        }


        if (msgId == POLL) {
            // on POLL we (re-)start, so no protocol failure
            protocolFailed = false;

            timePollReceived = DW1000Ng::getReceiveTimestamp();

            expectedMsgId = RANGE;
            rangeWaitStart = millis();

            transmitPollAck();
            noteActivity();
        }
        
        else if (msgId == RANGE) {

            timeRangeReceived = DW1000Ng::getReceiveTimestamp();
            expectedMsgId = POLL;

            if (!protocolFailed) {
                timePollSent = DW1000NgUtils::bytesAsValue(data + 2, LENGTH_TIMESTAMP);
                timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 7, LENGTH_TIMESTAMP);
                timeRangeSent = DW1000NgUtils::bytesAsValue(data + 12, LENGTH_TIMESTAMP);

                // (re-)compute range as two-way ranging is done
                double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);
                /* Apply simple bias correction */

                distance = DW1000NgRanging::correctRange(distance);

                sendDistance(distance);  // 유아트로 거리 전달하기
 
                transmitRangeReport(distance);  // 태그에 레인지 리포트 전송하기
                

                // 1초에 몇변 거리 출력 되는지 계산 : samplingRate
                successRangingCount++;
                if (curMillis - rangingCountPeriod > 1000) {
                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                    rangingCountPeriod = curMillis;
                    successRangingCount = 0;
                }
            }
            else {

                sendDistanceFailed();  // 유아트로 거리출력 실패 출력
                transmitRangeFailed();
            }

            noteActivity();
        }
    }
}