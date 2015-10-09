/**
 * @file   MyoIMUGestureController.cpp
 * @author Valentin Roland (webmaster at vroland.de)
 * @date   September-October 2015
 * @brief  Implementation file for data handling.
 *
 * This library provides gesture detection functionality using almost exclusively the IMU data of the Myo Armband.
 * The gestures are based on arm rotation to work with persons where distinct muscle activity is hard to detect.
 * Muscle activity is only used for starting/ending the recording of a gesture. Uses the MyoBridge Arduino Library (https://github.com/vroland/MyoBridge).
 */

#include "MyoIMUGestureController.h"

/**
 * Member definitions
 */
/// Callback for gesture recognition
void (*MyoIMUGestureController::on_gesture)(GestureType);
/// Callback for lock status changes
void (*MyoIMUGestureController::on_lock_change)(bool);

///The MyoBridge object used
MyoBridge* MyoIMUGestureController::bridge;

///reference orientation. Set on unlock.
Matrix33 MyoIMUGestureController::inverseInitMatrix = ZERO_MATRIX;
///tell the IMU handle to save a new inverse init matrix
bool MyoIMUGestureController::refresh_init = true;

///the EMG data cache. used to smooth out EMG data
int8_t MyoIMUGestureController::emgCache[EMG_CACHE_SIZE][8] = {0};
///sum of the absolute value of all EMG data stored in the cache.
long MyoIMUGestureController::emgSum = 0;
///maximum emgSum during sync time, used as reference.
long MyoIMUGestureController::emgSync = 0;
///Is EMG synced? (reference value stored)
bool MyoIMUGestureController::isEMGSynced = false;
///Does the user currently do the lock/unlock pose? Used to toggle the lock state.
bool MyoIMUGestureController::lock_toggle = true; 
///Is the device unlocked to store data?
bool MyoIMUGestureController::locked;

/// the time in milliseconds passed since connection
unsigned long MyoIMUGestureController::timeConnected = 0;


/**
 * Initialize the gesture controller. This will change the parameters of the passed
 * MyoBridge object: It will disable sleep, enable IMU and EMG data and set its own functions
 * as callbacks. The MyoBridge object is also used to send commands like vibrations.
 * Call after MyoBridge.connect()!
 * 
 * @param myoBridge The MyoBridge object to use.
 * @param onGesture Callback for gesture recognition
 * @param onLockChange Callback for lock status changes
 */
void MyoIMUGestureController::begin(MyoBridge &myoBridge, void (*onGesture)(GestureType), void (*onLockChange)(bool)) {
  bridge = &myoBridge;
  on_gesture = onGesture;
  on_lock_change = onLockChange;

  //activate data streams
  bridge->setIMUMode(IMU_MODE_SEND_DATA);
  bridge->setEMGMode(EMG_MODE_SEND);
  bridge->disablePoseData();
  
  //set callbacks
  bridge->setIMUDataCallBack(handleIMUData);
  bridge->setEMGDataCallBack(handleEMGData);
	
  //disable Myo sleep mode
  bridge->disableSleep(); 
  
  //vibrate long to signalize start of syncing process
  bridge->vibrate(3);
}

/**
 * handle the IMU data
 */
void MyoIMUGestureController::handleIMUData(MyoIMUData& data) {
  
  //build matrix of the current absolute orientation
  Matrix33 matrix = ZERO_MATRIX; 
  Matrix33 local = ZERO_MATRIX; 
  
  unit_quaternion_to_matrix(matrix, (int16_t*)&data.orientation);
  
  //save initial orientation matrix, is reset on unlock. Used for reference
  if (refresh_init) {
    inverse_matrix(matrix, inverseInitMatrix);
    refresh_init = false;
  }
  
  multiply_matrix(matrix, inverseInitMatrix, local);
  
  float roll_angle = asin(local[1][0]);
  
  //enable locking/unlocking feature after a certain delay after sync
  if (isEMGSynced && (timeConnected + EMG_SYNC_TIME + AFTER_SYNC_WAIT < millis())) {
	
    if ((float) emgSum/ (float) emgSync < LOCK_TOGGLE_THRESHOLD) {
	  
		//discard gesture if the buffer is full -> user is probably inactive or gesture incomplete
		if ((!locked) && gestureBufferFull()) {
			
			resetGestureCache();
			// initiate re-locking
			lock_toggle = false;
		}
	
		//toggle lock
		if (!lock_toggle) {
			
			lock_toggle = true;
			locked = !locked;

			//end of unlocking gesture
			if (!locked) {
				resetGestureCache();
			}

			on_lock_change(locked);
		}
      
    } else {
      if (lock_toggle) {
         refresh_init = true;
         lock_toggle = false; 

         //begin of locking gesture
         if (!locked) {
          
          //get the recognized gesture
          GestureType gesture = processCacheData();
    
          if (gesture != ARM_UNKNOWN) {
            on_gesture(gesture);
          }
        }
      }  
    }
	
	//when recording, save angles in gesture cache
	if (!locked) {
	  updateGestureCache(local[2][1], local[2][0], roll_angle);
	}
  }
}

/**
 * Update the EMG cache. The EMG cache is used to smooth fluctuating values
 */
void MyoIMUGestureController::updateCache(int8_t* data) {
  memmove(emgCache[1], emgCache[0], (EMG_CACHE_SIZE-1)* 8 *sizeof(int8_t));
  memcpy (emgCache[0], data, 8 *sizeof(int8_t));
  emgSum = 0;
  for (int i=0; i<EMG_CACHE_SIZE; i++) {
      for (int j=0; j<8; j++)
        emgSum += abs(emgCache[i][j]);
  }
}

/**
 * Handle the EMG data. Also handles syncing.
 */
void MyoIMUGestureController::handleEMGData(int8_t data[8]) {
  //http://developerblog.myo.com/myocraft-emg-in-the-bluetooth-protocol/

  //store in EMG cache
  updateCache(data);

  //start sync
  if (timeConnected == 0) {
    //prompt sync
    timeConnected = millis();
    #ifdef DEBUG_SERIAL
    Serial.println(F("Syncing. Please perform a strong gesture to use for emg evaluation."));  
    #endif
  }

  //still syncing?
  if (timeConnected + EMG_SYNC_TIME > millis()) {

    //when syncing, remember highest EMG value
    if (emgSum > emgSync) emgSync = emgSum;  
    
  } else {
    if (!isEMGSynced) {
      isEMGSynced = true;
      #ifdef DEBUG_SERIAL
      Serial.println(F("Done."));
      #endif
      //vibrate short to signalize end of syncing process
      MyoIMUGestureController::bridge->vibrate(1);
    }  
  }
}
