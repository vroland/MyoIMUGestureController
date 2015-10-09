/**
 * @file   gestureAnalysis.h
 * @author Valentin Roland (webmaster at vroland.de)
 * @date   September-October 2015
 * @brief  Header file describing gesture types.
 *
 * This library provides gesture detection functionality using almost exclusively the IMU data of the Myo Armband.
 * The gestures are based on arm rotation to work with persons where distinct muscle activity is hard to detect.
 * Muscle activity is only used for starting/ending the recording of a gesture. Uses the MyoBridge Arduino Library (https://github.com/vroland/MyoBridge).
 */

#ifndef GESTUREANALYSIS_H
#define GESTUREANALYSIS_H

#include <Arduino.h>

/// number of float values to be stored for gesture recognition. 
/// This means it will hold approx. length/(15 * 2) seconds of recorded data
/// This has to be an even number!
#define GESTURE_CACHE_SIZE 128

/// Straight movement parameters

/// maximum relation of X and Y standard deviation for straight movement
#define STRAIGHT_MAX_RELATION .5
/// minimum angle (-distance) for straight movement
#define STRAIGHT_MIN_DISTANCE .3
/// make movement in Y-direction stronger by this factor to address smaller range
#define Y_DEVIATION_CORRECTION 1.3

/// circular movement parameters

/// The number of sample points to use for diameter and center determination
#define GESTURE_CIRCLE_SAMPLES 10
/// Minimum circle diameter
#define CIRCLE_MIN_DIAMETER .65
/// Maximum standard deviation of circle radius
#define CIRCLE_MAX_DEVIATION .3
/// Maximum distance from start to end of the circle
#define MAX_ENDS_DISCANCE .4

/// rotation movement parameters

/// Maximum variance from (0,0)
#define ROTATION_MAX_VARIANCE .15
/// Minimum rotation angle
#define ROTATION_MIN_ANGLE PI/6

/// Gesture Types
typedef enum GestureType {
  ARM_UP,
  ARM_DOWN,
  ARM_LEFT,
  ARM_RIGHT,
  ARM_CIRCLE_CW,
  ARM_CIRCLE_CCW,
  ARM_ROTATE_CW,
  ARM_ROTATE_CCW,
  ARM_UNKNOWN
};

/**
 * Return the string equivalent of a GestureType constant.
 */
const char* gestureToString(GestureType type);

/**
 * Caches IMU data for gesture recognition.
 */
void updateGestureCache(float x, float y, float roll_angle);

/**
 * Resets the gesture cache.
 */
void resetGestureCache();

/**
 * The buffer will be filled at a rate of about 30 floats/second at maximum. This function resturns if it is full.
 */
bool gestureBufferFull();
 
/**
 * Processes the cached gesture data to recognize gestures.
 */
GestureType processCacheData();

#endif
