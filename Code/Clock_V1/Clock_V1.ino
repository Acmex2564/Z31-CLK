#include <LedDisplay.h>
#include <Keypad.h>
#include <TimeLib.h>
#include <ACAN.h>

#define CAN_TX 3
#define CAN_RX 4
#define CAN_EN 1

#define CAN_BITRATE_HS 5*100*1000 //500kbit/s
#define ID_LAMBDA 0x400
#define ID_BRIGHTNESS 0x202
#define ID_TEMPERATURE 0x750
#define ID_STATUS 0x755
#define ID_SPEED 0x760
#define ID_FUEL 0x720

#define ledData 23
#define ledRS 22
#define ledClk 21
#define ledEn 15
#define ledRst 14

#define ledLen 8

#define TIME_HEADER  "T"   // Header tag for serial time sync message

#define PAGE_CLOCK 0 //"HH:MM:SS"
#define PAGE_CAL 1 //"DD/MM/YY"
#define PAGE_AFR 2 //"AFR XXXX"
#define PAGE_LSU_TEMP 3 //"LSU YYYC"
#define PAGE_LSU_STATE 4 //"STA 0xZZ"
#define PAGE_RAD_TEMP 5 //"RAD YY*C"
#define PAGE_FAN_SPD 6 //"FAN XXX%"
#define PAGE_FUEL_AP 7 //
#define PAGE_MAN_AP 8 //
#define PAGE_VEH_SPD 9 //
#define PAGE_FUEL_1 10 //
#define PAGE_FUEL_2 11 //



const byte rows = 2;
const byte cols = 2;

char buttons[rows][cols] = {
  {'S', 'R'},
  {'+', '-'}
};
byte rowPins[rows] = {5, 6};
byte colPins[cols] = {7, 8};

Keypad buttonPad = Keypad(makeKeymap(buttons), rowPins, colPins, rows, cols);

LedDisplay disp = LedDisplay(ledData, ledRS, ledClk, ledEn, ledRst, ledLen);

int brightness = 15;

int currentPage = 0;

int TZ_ADJUST = -4;


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
int fuelMain;
int fuelSub;


unsigned long timeLastLambda;
unsigned long timeLastTemp;
unsigned long timeLastStatus;
unsigned long timeLastSpeed;
unsigned long timeLastFuel;
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

      case ID_FUEL:
        fuelMain = frameReceived.data[0];
        fuelSub = frameReceived.data[1];
        timeLastFuel = millis();
        break;
    };
  };

refreshDisp(currentPage);

}

void updateDisp(int currentState, char keyPress) {
  switch (keyPress) {
    case 'S':
      if (currentState >= 11) {
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
    case PAGE_CLOCK: //clock
      if (currentSecond != second()) {
        
        disp.print(getTimeStamp());

      }
      break;
    case 1: //calendar
      
      disp.print(getDateStamp());
      break;

    case PAGE_AFR: //lambda
      
      disp.print("AFR:");
      if (millis() - timeLastLambda > timeMaxMessage) {
        disp.print("NDAT");
      }
      else {
        disp.print((float)lambda / 1000 , 2);
      }
      break;
    case PAGE_LSU_TEMP: //lsu temp
      
      disp.print("LSU ");
      if (millis() - timeLastLambda > timeMaxMessage) {
        disp.print("NDAT");
      }
      else {
        disp.print(temp);
        disp.print("C");
      }
      break;
    case PAGE_LSU_STATE: //lsu status
      
      disp.print("STA ");
      if (millis() - timeLastLambda > timeMaxMessage) {
        disp.print("NDAT");
      }
      else {
        disp.print(lsu_stat, HEX);
      }
      break;

    case PAGE_RAD_TEMP: //radiator temperature
      
      disp.print("RCT ");
      if (millis() - timeLastTemp > timeMaxMessage){
        disp.print("NDAT");
      }
      else {
        disp.print(tempWater);
        disp.print("C  ");
      }
      break;

    case PAGE_FAN_SPD:
      disp.print("FAN ");
      if (millis() - timeLastTemp > timeMaxMessage){
        disp.print("NDAT");
      }
      else {
        disp.printf("%3.0f%   ", 100 * (pwmFan/255.0));
      }
      break;
    case PAGE_FUEL_AP: //fuel pressure
      disp.print("FP ");
      if (millis() - timeLastStatus > timeMaxMessage){
        disp.print(" NDAT");
      }
      else {
        disp.printf("%2.1f#   ", pressFuel*3/6.895);
      }
      break;
    case PAGE_MAN_AP: //manifold pressure
      disp.print("MP ");
      if (millis() - timeLastStatus > timeMaxMessage){
        disp.print(" NDAT");
      }
      else {
        disp.printf("%2.1f#   ", pressManifold/6.895);
      }
      break;

    case PAGE_VEH_SPD: //vehicle speed
      disp.print("SPD ");
      if (millis() - timeLastStatus > timeMaxMessage){
        disp.print(" NDAT");
      }
      else {
        disp.print(vehicleSpeed);
        disp.print("k  ");
      }
      break;

    case PAGE_FUEL_1:
      disp.print("MFL ");
      if (millis() - timeLastFuel > timeMaxMessage){
        disp.print("NDAT");
      }
      else {
        disp.printf("%2.1f", ((float)fuelMain)/10);
      }
      break;
    case PAGE_FUEL_2:
      disp.print("SFL ");
      if (millis() - timeLastFuel > timeMaxMessage){
        disp.print("NDAT");
      }
      else {
        disp.printf("%2.1f", ((float)fuelSub)/10);
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
