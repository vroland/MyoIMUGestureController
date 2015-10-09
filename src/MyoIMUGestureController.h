/**
 * @file   MyoIMUGestureController.h
 * @author Valentin Roland (webmaster at vroland.de)
 * @date   September-October 2015
 * @brief  header file describing the library interface
 *
 * This library provides gesture detection functionality using almost exclusively the IMU data of the Myo Armband.
 * The gestures are based on arm rotation to work with persons where distinct muscle activity is hard to detect.
 * Muscle activity is only used for starting/ending the recording of a gesture. Uses the MyoBridge Arduino Library (https://github.com/vroland/MyoBridge).
 */
 
#ifndef DATAHANDLING_H
#define DATAHANDLING_H

#include <MyoBridge.h>
#include "include/gestureAnalysis.h"
#include "include/matrix.h"

///Number of EMG values to cache
#define EMG_CACHE_SIZE 10
///time to perform sync gesture
#define EMG_SYNC_TIME 3000
///Lock toggle threshold
#define LOCK_TOGGLE_THRESHOLD .5
///time to wait after sync gesture
#define AFTER_SYNC_WAIT 500

/**
 * This class provides gesture detection functionality. The gestures are based on
 * arm rotation to work with persons where distinct muscle activity is hard to detect.
 * Muscle activity is only used for starting/ending the recording of a gesture.
 * Uses the MyoBridge Arduino Library (https://github.com/vroland/MyoBridge).
 * NOTE: Use this class as "static" class. Use no or just one instance.
 *
 * Example sketch:
 * @include MyoIMUGestureControl.ino
 */
class MyoIMUGestureController {
  public:

     /**
     * Initialize the gesture controller. This will change the parameters of the passed
     * MyoBridge object: It will disable sleep, enable IMU and EMG data and set its own functions
     * as callbacks. The MyoBridge object is also used to send commands like vibrations.
     * Call after MyoBridge.connect()!
     * you can define DEBUG_SERIAL to print sync instructions to hardware serial.
     * 
     * @param myoBridge The MyoBridge object to use.
     * @param onGesture Callback for gesture recognition
     * @param onLockChange Callback for lock status changes
     */
    static void begin(MyoBridge &myoBridge, void (*onGesture)(GestureType), void (*onLockChange)(bool));

  private:
    
    /// Callback for gesture recognition
    static void (*on_gesture)(GestureType);
    /// Callback for lock status changes
    static void (*on_lock_change)(bool);

    ///The MyoBridge object used
    static MyoBridge* bridge;

    ///reference orientation. Set on unlock.
    static Matrix33 inverseInitMatrix;
    ///tell the IMU handle to save a new inverse init matrix
    static bool refresh_init;
    
    ///the EMG data cache. used to smooth out EMG data
    static int8_t emgCache[EMG_CACHE_SIZE][8];
    ///sum of the absolute value of all EMG data stored in the cache.
    static long emgSum;
    ///maximum emgSum during sync time, used as reference.
    static long emgSync;
    ///Is EMG synced? (reference value stored)
    static bool isEMGSynced;
    ///Does the user currently do the lock/unlock pose? Used to toggle the lock state.
    static bool lock_toggle; 
    ///Is the device unlocked to store data?
    static bool locked;
    
    /// the time in milliseconds passed since connection
    static unsigned long timeConnected;
     
    /**
     * handle the IMU data
     */
    static void handleIMUData(MyoIMUData& data);
    
    /**
     * Update the EMG cache. The EMG cache is used to smooth fluctuating values
     */
    static void updateCache(int8_t* data);
    
    /**
     * Handle the EMG data. Also handles syncing.
     */
    static void handleEMGData(int8_t data[8]);

};

#endif
