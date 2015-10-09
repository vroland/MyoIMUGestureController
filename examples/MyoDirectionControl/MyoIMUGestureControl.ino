/**
 * @file   MyoIMUGestureControl.ino
 * @author Valentin Roland (webmaster at vroland.de)
 * @date   September-October 2015
 * @brief  Example sketch using leds and the hardware serial connection to show the detected gestures.
 *
 * This library provides gesture detection functionality using almost exclusively the IMU data of the Myo Armband.
 * The gestures are based on arm rotation to work with persons where distinct muscle activity is hard to detect.
 * Muscle activity is only used for starting/ending the recording of a gesture. Uses the MyoBridge Arduino Library (https://github.com/vroland/MyoBridge).
 */

//use serial debug messages
#define DEBUG_SERIAL

#include <MyoBridge.h>
#include <SoftwareSerial.h>
#include <MyoIMUGestureController.h>

///Pin for the Left-Right switch LED
#define LR_LED_PIN 6
///Pin for the Up-Down switch LED
#define UD_LED_PIN 7
///Pin for the Lock LED
#define LK_LED_PIN 8

typedef struct {
  bool up_active = false;
  bool right_active = false;
} ControlStatus_t;

ControlStatus_t controlStatus;

//SoftwareSerial connection to MyoBridge
SoftwareSerial bridgeSerial(2,3);

//initialize MyoBridge object with software serial connection, provide reset pin
MyoBridge bridge(bridgeSerial, 4);

//a function to inform us about the current state and the progess of the connection to the Myo.
void printConnectionStatus(MyoConnectionStatus status) {
  
  //print the status constant as string
  Serial.println(status);
}

/** 
 * update the output after IMU handling. 
 */
void updateControls(GestureType gesture) {

  Serial.println(gestureToString(gesture)); 
  bridge.vibrate(1); 

  if (controlStatus.up_active && (gesture == ARM_DOWN)) {
   controlStatus.up_active = false;
  } 
  if (controlStatus.right_active && (gesture == ARM_LEFT)) {
   controlStatus.right_active = false;
  }

  if (!controlStatus.right_active && (gesture == ARM_UP)) {
    controlStatus.up_active = true; 
  } 
  if (!controlStatus.up_active && (gesture == ARM_RIGHT)) {
    controlStatus.right_active = true; 
  } 

  digitalWrite(UD_LED_PIN, controlStatus.up_active);
  digitalWrite(LR_LED_PIN, controlStatus.right_active);
}

///update the output after lock change
void updateLockOutput(bool locked) {
  digitalWrite(LK_LED_PIN, !locked);
}

///set up pin mode for output pins
void pin_setup() {
  pinMode(LR_LED_PIN, OUTPUT);
  pinMode(UD_LED_PIN, OUTPUT);
  pinMode(LK_LED_PIN, OUTPUT);
}

void setup() {

  //set pins as output
  pin_setup();
  
  //initialize both serial connections
  Serial.begin(115200);
  bridgeSerial.begin(115200);

  //wait until MyoBridge has found Myo and is connected. Make sure Myo is not connected to anything else and not in standby!
  Serial.println(F("Searching for Myo..."));
  //initiate the connection with the status callback function
  bridge.begin(printConnectionStatus);
  
  Serial.println(F("connected!"));
  
  MyoIMUGestureController::begin(bridge, updateControls, updateLockOutput);
}

void loop() {
  
  //update the connection to MyoBridge
  bridge.update();
}
