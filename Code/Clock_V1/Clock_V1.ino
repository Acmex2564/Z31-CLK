#include <LedDisplay.h>
#include <Keypad.h>
#include <TimeLib.h>
#include <ACAN.h>

const byte rows = 2;
const byte cols = 2;

char buttons[rows][cols] = {
  {'S', 'R'},
  {'+', '-'}
};
byte rowPins[rows] = {5, 6};
byte colPins[cols] = {7, 8};

Keypad buttonPad = Keypad(makeKeymap(buttons), rowPins, colPins, rows, cols);

#define CAN_TX 3
#define CAN_RX 4
#define CAN_EN 1

#define CAN_BITRATE_HS 5*100*1000 //500kbit/s
#define ID_LAMBDA 0x400
#define ID_BRIGHTNESS 0x202
#define ID_TEMPERATURE 0x750
#define ID_STATUS 0x755
#define ID_SPEED 0x760

#define ledData 23
#define ledRS 22
#define ledClk 21
#define ledEn 15
#define ledRst 14

#define ledLen 8

#define TIME_HEADER  "T"   // Header tag for serial time sync message

LedDisplay disp = LedDisplay(ledData, ledRS, ledClk, ledEn, ledRst, ledLen);

int brightness = 15;

int currentPage = 0;

int TZ_ADJUST = -4;

/* Page List:
    0: Clock: "HH:MM:SS"
    1: Cal: "DD/MM/YY"
    2: AFR: "AFR XXXX"
    3: LSU Temp: "LSU YYYC"
    4: LSU State: "STA 0xZZ"
    5: RAD Temp: "RAD YY*C"
    6: FAN Speed: "FAN XXX%"
*/

uint8_t currentSecond = second();

word lambda;
float afr;
unsigned int temp = 0;
byte lsu_stat;

int tempWater;
int tempAmbient;
int pressManifold;
int pressFuel;
int vehicleSpeed;
int pwmFan;


unsigned long timeLastLambda;
unsigned long timeLastTemp;
unsigned long timeLastStatus;
unsigned long timeLastSpeed;
unsigned long timeMaxMessage = 2000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //while (!Serial);
  disp.begin();
  disp.setBrightness(brightness);

  pinMode(CAN_TX, OUTPUT);
  pinMode(CAN_EN, OUTPUT);
  pinMode(CAN_RX, INPUT_PULLUP);

  ACANSettings settingsHSCan (CAN_BITRATE_HS);


  settingsHSCan.mRxPinHasInternalPullUp = false;
  settingsHSCan.mTxPinIsOpenCollector = false;
  uint32_t errorCan = ACAN::can0.begin(settingsHSCan);

  if (0 == errorCan) {
    Serial.print("Alive. CAN OK");
  }
  else {
    disp.print("ERR 0x"); disp.print(errorCan, HEX);
    Serial.print("CAN Failure. 0x"); Serial.println(errorCan, HEX);
  }
  setSyncProvider(getTeensy3Time);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  char key = buttonPad.getKey();

  if (key) {
    Serial.println(key);
    updateDisp(currentPage, key);
  }

  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }

  if (ACAN::can0.available()) {
    CANMessage frameReceived;
    ACAN::can0.receive (frameReceived);


    switch (frameReceived.id)
    {
      case ID_BRIGHTNESS:
          if (frameReceived.data[0] == 254)
          {
              if (brightness != 15) { brightness = 15; disp.setBrightness(brightness);}
          }
          else 
          {
              if (brightness != map(frameReceived.data[0], 0, 253, 1, 15)) {brightness = map(frameReceived.data[0], 0, 253, 1, 15); disp.setBrightness(brightness);}
          }
          
        break;
      case ID_LAMBDA:
        //per spartan3 manual:
        //Data[0]: Lambda x1000 High Byte
        //Data[1]: Lambda x1000 Low Byte
        //Data[2]: LSU Temp/10
        //Data[3]: Status
        lambda = 0;
        lambda = ((int) frameReceived.data[0] << 8) | frameReceived.data[1];
        afr = ((float)lambda / 1000) * 14.7;
        temp = frameReceived.data[2] * 10;
        lsu_stat = frameReceived.data[3];
        timeLastLambda = millis();
        break;

      case ID_TEMPERATURE:
        tempWater = frameReceived.data[0] - 40;
        tempAmbient = frameReceived.data[1] - 40;
        
        timeLastTemp = millis();
        break;

      case ID_STATUS:

        pwmFan = frameReceived.data[0];
        pressManifold = frameReceived.data[1];
        pressFuel = frameReceived.data[2] * 3;
        
        timeLastStatus = millis();
        break;

      case ID_SPEED:
        vehicleSpeed = frameReceived.data[0];
        timeLastSpeed = millis();
        break;
    };
  };

refreshDisp(currentPage);

}

void updateDisp(int currentState, char keyPress) {
  switch (keyPress) {
    case 'S':
      if (currentState >= 9) {
        currentPage = 0;
      }
      else {
        currentPage = currentState + 1;
        Serial.println(currentState);
      }
      break;
    case 'R':
      currentPage = 0;
      break;
  }
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

void refreshDisp(int currentState) {
  disp.home();
  switch (currentState) {
    case 0: //clock
      if (currentSecond != second()) {
        
        disp.print(getTimeStamp());

      }
      break;
    case 1: //calendar
      
      disp.print(getDateStamp());
      break;

    case 2: //lambda
      
      disp.print("AFR:");
      if (millis() - timeLastLambda > timeMaxMessage) {
        disp.print("NDAT");
      }
      else {
        disp.print((float)lambda / 1000 , 2);
      }
      break;
    case 3: //lsu temp
      
      disp.print("LSU ");
      if (millis() - timeLastLambda > timeMaxMessage) {
        disp.print("NDAT");
      }
      else {
        disp.print(temp);
        disp.print("C");
      }
      break;
    case 4: //lsu status
      
      disp.print("STA ");
      if (millis() - timeLastLambda > timeMaxMessage) {
        disp.print("NDAT");
      }
      else {
        disp.print(lsu_stat, HEX);
      }
      break;

    case 5: //radiator temperature
      
      disp.print("RCT ");
      if (millis() - timeLastTemp > timeMaxMessage){
        disp.print("NDAT");
      }
      else {
        disp.print(tempWater);
        disp.print("C  ");
      }
      break;

    case 6:
      disp.print("FAN ");
      if (millis() - timeLastTemp > timeMaxMessage){
        disp.print("NDAT");
      }
      else {
        disp.printf("%3.0f%   ", 100 * (pwmFan/255.0));
      }
      break;
    case 7: //fuel pressure
      disp.print("FP ");
      if (millis() - timeLastStatus > timeMaxMessage){
        disp.print(" NDAT");
      }
      else {
        disp.printf("%2.1f#   ", pressFuel*3/6.895);
      }
      break;
    case 8: //manifold pressure
      disp.print("MP ");
      if (millis() - timeLastStatus > timeMaxMessage){
        disp.print(" NDAT");
      }
      else {
        disp.printf("%2.1f#   ", pressManifold/6.895);
      }
      break;

    case 9: //vehicle speed
      disp.print("SPD ");
      if (millis() - timeLastStatus > timeMaxMessage){
        disp.print(" NDAT");
      }
      else {
        disp.print(vehicleSpeed);
        disp.print("k  ");
      }
      break;
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

String getTimeStamp() {
  char timestamp[8] = "";
  sprintf(timestamp, "%02d:%02d:%02d", hour(now() + 60 * 60 * TZ_ADJUST), minute(now() + 60 * 60 * TZ_ADJUST), second(now() + 60 * 60 * TZ_ADJUST));
  return timestamp;
}

String getDateStamp() {
  char datestamp[8] = "";
  sprintf(datestamp, "%02d-%02d-%02d", day(now() + 60 * 60 * TZ_ADJUST), month(now() + 60 * 60 * TZ_ADJUST), year(now() + 60 * 60 * TZ_ADJUST) % 100);
  return datestamp;
}
