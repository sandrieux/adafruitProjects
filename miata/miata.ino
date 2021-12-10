#include <SPI.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>
#include <EasyNextionLibrary.h> 

#define refreshTempDelay 5000
#define refreshAccelDelay 1000
#define encoderSwithcDebounceTime 200 //Define how long we'll ignore clicks to debounce
#define SS_SWITCH 24
#define SS_NEOPIX 6
#define SEESAW_BASE_ADDR 0x36

Adafruit_AHTX0 aht;
Adafruit_MPU6050 mpu;
EasyNex myNex(Serial1);
// create 2 encoders!
Adafruit_seesaw encoders[2];
// create 2 encoder pixels
seesaw_NeoPixel encoder_pixels[2] = {
  seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800),
  seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800)
};
int32_t encoder_positions[] = {0, 0};
bool found_encoders[] = {false, false};
unsigned long refreshCurrent;
unsigned long refreshTempLast;
unsigned long refreshAccelLast;


void setup() {
  //Initialize the refresh timer  
  refreshCurrent = millis();
  refreshTempLast = refreshCurrent - refreshTempDelay;
  refreshAccelLast = refreshCurrent - refreshAccelDelay;
  
  // Start serial and wait for port to open
  Serial.begin(115200);
  while (!Serial) delay(10);
  //Begin AHT
  if (aht.begin()) {
      Serial.println("Found AHT20");
  } else {
      Serial.println("!! Did not find AHT20 !!");
  }
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
  for (uint8_t enc=0; enc<sizeof(found_encoders); enc++) {
      // See if we can find encoders on this address 
      if (! encoders[enc].begin(SEESAW_BASE_ADDR + enc) || ! encoder_pixels[enc].begin(SEESAW_BASE_ADDR + enc)) {
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
          encoder_positions[enc] = encoders[enc].getEncoderPosition();
          // set not so bright!
          encoder_pixels[enc].setBrightness(90);
          encoder_pixels[enc].show();
          found_encoders[enc] = true;
      }
  }
  Serial.println("Encoders started");
  //Start the Nextion
  myNex.begin(9600);
  delay(500);
}

void loop() {  
  for (uint8_t enc=0; enc<sizeof(found_encoders); enc++) { 
    if (found_encoders[enc] == false) continue;
    int32_t new_position = encoders[enc].getEncoderPosition();
    // did we move around?
    if (encoder_positions[enc] != new_position) {
      Serial.print("Encoder #");
      Serial.print(enc);
      Serial.print(" -> ");
      Serial.println(new_position);
      encoder_positions[enc] = new_position;
      // change the neopixel color, mulitply the new positiion by 4 to speed it up
      encoder_pixels[enc].setPixelColor(0, Wheel((new_position*4) & 0xFF));
      encoder_pixels[enc].show();
    }
  }
  // Set current loop timestamp
  refreshCurrent = millis();
  // Time to refresh temperature and humiidity? Move to function...
  if ( refreshCurrent - refreshTempLast > refreshTempDelay) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    Serial.print("Fetching Temp and humidity every ");Serial.print(refreshTempDelay);Serial.println(" ms");
    Serial.print("Uptime: ");Serial.print(refreshCurrent);Serial.println(" ms");
    Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.println("° C");
    Serial.print("Himidity: ");Serial.print(humidity.relative_humidity);Serial.println("%");
    refreshTempLast = millis(); 
  }
  // Time to refresh accel and gyro? Move to function...
  if ( refreshCurrent - refreshAccelLast > refreshAccelDelay) {
    Serial.print("Fetching Acceleration and gyro every ");Serial.print(refreshAccelDelay);Serial.println(" ms");
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    /* Print out the values */
    Serial.println("Acceleration");
    Serial.print("X: ");Serial.print(a.acceleration.x*0.101972);Serial.println(" G");
    Serial.print("Y: ");Serial.print(a.acceleration.y*0.101972);Serial.println(" G");
    Serial.print("Z: ");Serial.print(a.acceleration.z*0.101972);Serial.println(" G");
    Serial.println("Rotation");
    Serial.print("X: ");Serial.print(g.gyro.x);Serial.println(" rad/s");
    Serial.print("Y: ");Serial.print(g.gyro.y);Serial.println(" rad/s");
    Serial.print("Z: ");Serial.print(g.gyro.z);Serial.println(" rad/s");
    refreshAccelLast = millis();
  }
  delay(100);
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
