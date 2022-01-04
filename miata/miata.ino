#include <SPI.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>
#include <EasyNextionLibrary.h>
#include <RTClib.h>

#define refreshTempDelay 5000
#define refreshAccelDelay 5000
#define encoderSwithcDebounceTime 300 //Define how long we'll ignore clicks to debounce
#define SS_SWITCH 24
#define SS_NEOPIX 6
#define SEESAW_BASE_ADDR 0x36


Adafruit_AHTX0 aht;
Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;
EasyNex myNex(Serial1);
// create 2 encoders!
Adafruit_seesaw encoders[2];
// create 2 encoder pixels
seesaw_NeoPixel encoderPixels[2] = {
  seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800),
  seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800)
};
int32_t encoderPositions[] = {0, 0};
unsigned long encoderPress[] = {0, 0};
bool foundEncoders[] = {false, false};
unsigned long refreshCurrent;
unsigned long refreshTempLast;
unsigned long refreshAccelLast;
int32_t audioVolume = 20;
int32_t audioVolumeSave = 20;
bool audioMute = false;
bool menuPop = false;


void setup() {
  //Initialize the refresh timer  
  refreshCurrent = millis();
  refreshTempLast = refreshCurrent - refreshTempDelay;
  refreshAccelLast = refreshCurrent - refreshAccelDelay;
  
  // Start serial and wait for port to open
  Serial.begin(115200);
  while (!Serial) delay(10);
  //Delay to wait for serial monitor to begin
  delay(2000);
  //Begin AHT
  if (aht.begin()) {
      Serial.println("Found AHT20");
  } else {
      Serial.println("!! Did not find AHT20 !!");
  }
  //Begin RTC
    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }

//  if (rtc.lostPower()) {
//    Serial.println("RTC lost power, let's set the time!");
//    rtc.adjust(DateTime(2021,12,14,17,11,0));
//  }
  //Begin MPU6050
  if (mpu.begin()) {
    Serial.println("Found MPU6050");
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    Serial.print("Accel range: ");Serial.println(mpu.getAccelerometerRange());
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range: ");Serial.println(mpu.getGyroRange());
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);Serial.println(mpu.getFilterBandwidth());
    Serial.print("Filter bandwidth set to: ");
  } else {
    Serial.println("!! Did not found MPU6050 !!");
  }
  //Start seesaw encoders and pixels
  Serial.println("Seesaw Encoders:");
  for (uint8_t enc=0; enc<sizeof(foundEncoders); enc++) {
      // See if we can find encoders on this address 
      if (! encoders[enc].begin(SEESAW_BASE_ADDR + enc) || ! encoderPixels[enc].begin(SEESAW_BASE_ADDR + enc)) {
          Serial.print("Couldn't find encoder #");
          Serial.println(enc);
      } else {
          Serial.print("Found encoder + pixel #");
          Serial.println(enc);
          uint32_t version = ((encoders[enc].getVersion() >> 16) & 0xFFFF);
          if (version != 4991){
              Serial.print("Wrong firmware loaded? ");
              Serial.println(version);
              while(1) delay(10);
          }
          Serial.println("Found Product 4991");
          // use a pin for the built in encoder switch
          encoders[enc].pinMode(SS_SWITCH, INPUT_PULLUP);
          // get starting position
          encoderPositions[enc] = encoders[enc].getEncoderPosition();
          encoderPixels[enc].setBrightness(100);
          encoderPixels[enc].show();
          foundEncoders[enc] = true;
      }
  }
  Serial.println("Encoders started");
  //Start the Nextion
  myNex.begin(9600);
  delay(500);
  DateTime now = rtc.now();
  Serial.print("Boot time: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  String currenTime = "";
  currenTime = currenTime + now.hour() + ":" + now.minute();
  myNex.writeStr("gTime.txt", currenTime);   
}
void loop() {
  refreshCurrent = millis();
  myNex.NextionListen();
  processEncoder();
  processTemp();
  processAccel();
}

//Functions
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return seesaw_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return seesaw_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return seesaw_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void trigger1(){
  //bVolPlus
  if (audioVolume < 100) {
    audioVolume += 5;
    Serial.print("Volume from HMI: ");Serial.println(audioVolume);
  }
}
void trigger2(){
  //bVolMinus
  if (audioVolume > 0) {
    audioVolume -= 5;
    Serial.print("Volume from HMI: ");Serial.println(audioVolume);
  }
}

void processEncoder () {
  int32_t newPosition;
  for (uint8_t enc=0; enc<sizeof(foundEncoders); enc++) { 
    if (foundEncoders[enc] == false) continue;
    newPosition = encoders[enc].getEncoderPosition(); 
    switch (enc) {
      case 0:
        //encoder 0 for volume
        if (!audioMute) {
          if ( newPosition < encoderPositions[enc]) {
            if (audioVolume < 100) {
              audioVolume += 5;
              Serial.print("Volume: ");Serial.println(audioVolume);
              myNex.writeNum("gVol.val", audioVolume);              
            }
          } else if ( newPosition > encoderPositions[enc]) {
            if (audioVolume > 0) {
              audioVolume -= 5;
              Serial.print("Volume: ");Serial.println(audioVolume);
              myNex.writeNum("gVol.val", audioVolume);
            }
          }
        }
        break;
      case 1:
        if ( newPosition != encoderPositions[enc]) {
          Serial.print("Encoder #1 new position: ");Serial.println(newPosition);
        }
        break;
    }
    encoderPositions[enc] = newPosition;
    //Encoder press
    if (! encoders[enc].digitalRead(SS_SWITCH)) {
      if ( encoderPress[enc] == 0 ) {
        encoderPress[enc] = millis();
          switch (enc) {
            case 0:
              audioMute = !audioMute;
              if (!audioMute){
                audioVolume = audioVolumeSave;
                Serial.println("Volume: unmute");
              } else {
                audioVolumeSave = audioVolume;
                audioVolume = 0;
                Serial.println("Volume: mute");
              }
              myNex.writeNum("MainPage.gVol.val", audioVolume);
              break;
            case 1:
              menuPop = !menuPop;
              if (menuPop) {
                Serial.println("Menu opened");
              } else {
                Serial.println("Menu closed");
              }
              break;
            }
      } else if ( refreshCurrent > encoderPress[enc] + encoderSwithcDebounceTime ) {
        encoderPress[enc] = 0;
      }
    }
  }
}

void processTemp () {
  if ( refreshCurrent - refreshTempLast > refreshTempDelay) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    refreshTempLast = millis();
    // Serial.print("Fetching Temp and humidity every ");Serial.print(refreshTempDelay);Serial.println(" ms");
    // Serial.print("Uptime: ");Serial.print(refreshCurrent);Serial.println(" ms");
    // Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.println("° C");
    // Serial.print("Himidity: ");Serial.print(humidity.relative_humidity);Serial.println("%");
  }
}

void processAccel () {
  if ( refreshCurrent - refreshAccelLast > refreshAccelDelay) {
    //Serial.print("Fetching Acceleration and gyro every ");Serial.print(refreshAccelDelay);Serial.println(" ms");
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // Serial.println("Acceleration");
    // Serial.print("X: ");Serial.print(a.acceleration.x*0.101972);Serial.println(" G");
    // Serial.print("Y: ");Serial.print(a.acceleration.y*0.101972);Serial.println(" G");
    // Serial.print("Z: ");Serial.print(a.acceleration.z*0.101972);Serial.println(" G");
    // Serial.println("Rotation");
    // Serial.print("X: ");Serial.print(g.gyro.x);Serial.println(" rad/s");
    // Serial.print("Y: ");Serial.print(g.gyro.y);Serial.println(" rad/s");
    // Serial.print("Z: ");Serial.print(g.gyro.z);Serial.println(" rad/s");
    refreshAccelLast = millis();
    DateTime now = rtc.now();
    String currenTime = "";
    currenTime = currenTime + now.hour() + ":" + now.minute();
    myNex.writeStr("gTime.txt", currenTime);
    Serial.print(currenTime);
  }
}