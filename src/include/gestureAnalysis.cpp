/**
 * @file   gestureAnalysis.cpp
 * @author Valentin Roland (webmaster at vroland.de)
 * @date   September-October 2015
 * @brief  Implementation file for gesture analysis.
 *
 * This library provides gesture detection functionality using almost exclusively the IMU data of the Myo Armband.
 * The gestures are based on arm rotation to work with persons where distinct muscle activity is hard to detect.
 * Muscle activity is only used for starting/ending the recording of a gesture. Uses the MyoBridge Arduino Library (https://github.com/vroland/MyoBridge).
 */

#include "gestureAnalysis.h"

/**
 * The gesture cache stores the x and y component of the pointing direction
 * while unlocked. When locked again, processCacheData tries to match a gesture
 * to the cached data.
 */
float gesture_cache[GESTURE_CACHE_SIZE];
int gesture_cache_offset = 0;
//cache the arm rotation to use later for gesture evaluation
float gesture_roll_angle = 0;

//Gesture strings
const char* const gesture_strings[] = {
  "UP",
  "DOWN",
  "LEFT",
  "RIGHT",
  "CIRCLE_CW",
  "CIRCLE_CCW",
  "ROTATE_CW",
  "ROTATE_CCW",
  "UNKNOWN"
};

inline float clip(float n, float lower, float upper) {
  return max(lower, min(n, upper));
}

inline float sqr(float a) {
  return a*a;
}

/**
 * Return the string equivalent of a GestureType constant.
 */
const char* gestureToString(GestureType type) {
  return gesture_strings[(uint8_t)type];  
}

/**
 * Resets the gesture cache.
 */
void resetGestureCache() {
	gesture_cache_offset = 0;
	gesture_roll_angle = 0;
}

/**
 * The buffer will be filled at a rate of about 30 floats/second at maximum. This function resturns if it is full.
 */
bool gestureBufferFull() {
	return (gesture_cache_offset == GESTURE_CACHE_SIZE);
}

/**
 * Caches IMU data for gesture recognition
 */
void updateGestureCache(float x, float y, float roll_angle) {

  if (gesture_cache_offset<GESTURE_CACHE_SIZE) {
    gesture_cache[gesture_cache_offset]     = asin(clip(x, -.99999, .99999));  
    gesture_cache[gesture_cache_offset + 1] = asin(clip(y, -.99999, .99999));  
    gesture_cache_offset += 2;
  }

  gesture_roll_angle = roll_angle;
}

inline float getSqrPointDist(float x1, float y1, float x2, float y2) {
  return sqr(x1-x2)+sqr(y1-y2);
}

/**
 * Processes the cached gesture data to recognize gestures.
 */
GestureType processCacheData() {

  
  //store number of points and reset cache offset
  int num_points = gesture_cache_offset / 2;
  gesture_cache_offset = 0;

  /***************************************************
   * Test for circular movement
   **************************************************/
  
  //list for storing diameters for all sample points
  float sample_point_diameters[GESTURE_CIRCLE_SAMPLES] = {0};
  
  //pick sample points
  short index_offset = (int)(num_points)/ GESTURE_CIRCLE_SAMPLES;

  float center_x = 0;
  float center_y = 0;
  
  //enough points?
  if (index_offset >= 1) {

    // find the greatest distance to another point (diameter) for all sample points
    for (int j=0;j<GESTURE_CIRCLE_SAMPLES;j++) {
      
      float point_x = gesture_cache[2 * j* index_offset];
      float point_y = gesture_cache[2 * j* index_offset + 1];

      //sum point coordinates for center determination
      center_x += point_x;
      center_y += point_y;
      for (int i=0;i<num_points;i++) {
      
        //get distance from sample point to current point
        float dist = sqrt(getSqrPointDist(point_x, point_y,
                     gesture_cache[2 * i], gesture_cache[2 * i + 1]));

        // store maximum distance
        if (dist > sample_point_diameters[j]) {
          sample_point_diameters[j] = dist;
        }
      }
    }

    // calculate circle center
    center_x /= (float) GESTURE_CIRCLE_SAMPLES;
    center_y /= (float) GESTURE_CIRCLE_SAMPLES;
  
    //Ymax, Xmin, Xmax coordinate for clockwise or counterclockwise determination
    float y_max = 0.;
    short y_max_index = 0;
    float x_max = 0.;
    float x_max_index = 0;
    float x_min = 1000.;
    float x_min_index = 0;
    
    //get average circle radius 
    float average_radius = 0;
    for (int j=0;j<GESTURE_CIRCLE_SAMPLES;j++) {
      average_radius +=sample_point_diameters[j];
    }
    
    average_radius /= (float) GESTURE_CIRCLE_SAMPLES * 2.;
	
    //get standard deviation of the radius
    float circular_deviation = 0;
    for (int i=0;i<num_points;i++) {

      float x = gesture_cache[2 * i];
      float y = gesture_cache[2 * i + 1];
    
      float dist = sqrt(getSqrPointDist(center_x, center_y, x, y));
                     
      circular_deviation += sqr(dist - average_radius);

      // track minima/maxima
      if (x>x_max) {
        x_max = x;  
        x_max_index = i;
      }
  
      if (x<x_min) {
        x_min = x;  
        x_min_index = i;
      }
      
      if (y>y_max) {
        y_max = y;  
        y_max_index = i;
      }
    }

    //calculate deviation
    circular_deviation = sqrt(circular_deviation / (float) num_points);
    
    //determine if the circle is nearly closed
    float ends_distance = sqrt(getSqrPointDist(gesture_cache[0], gesture_cache[1],
                               gesture_cache[2 * num_points - 2], gesture_cache[2 * num_points - 1]));

    //determine clockwise/counterclockwise
    bool clockwise = false;
    
    if ((x_min_index < y_max_index) && (y_max_index < x_max_index)) {
      clockwise = true;  
    }
    if ((y_max_index < x_max_index) && (x_max_index < x_min_index)) {
      clockwise = true;  
    }
    if ((x_max_index < x_min_index) && (x_min_index < y_max_index)) {
      clockwise = true;  
    }

    //are all conditions met?
    if ((average_radius * 2. >= CIRCLE_MIN_DIAMETER) && (circular_deviation <= CIRCLE_MAX_DEVIATION) && (ends_distance <= MAX_ENDS_DISCANCE)) {
      if (clockwise) {
        return ARM_CIRCLE_CW;  
      } else {
        return ARM_CIRCLE_CCW;  
      }
    }
  }

  /***************************************************
   * Gather some statistical data
   **************************************************/

  float x_total = 0.;
  float y_total = 0.;
  float x_deviation = 0;
  float y_deviation = 0;
  
  //get sum of all points and X/Y deviation
  for (int i=0;i<num_points;i++) {
    x_total += gesture_cache[2 * i];
    y_total += gesture_cache[2 * i + 1];
    
    x_deviation += sqr(gesture_cache[2 * i]);
    y_deviation += sqr(gesture_cache[2 * i + 1]);
  }

  //calculate deviations
  x_deviation /= (float) num_points;
  y_deviation /= (float) num_points;
  
  //correct y deviation because of more limited movement
  y_deviation *= Y_DEVIATION_CORRECTION;

  //deviation relation
  float relation = x_deviation/y_deviation;
  
  /***************************************************
   * Test for arm rotation
   **************************************************/
  
  //are the conditions met?
  if ((x_deviation <= ROTATION_MAX_VARIANCE) && (y_deviation <= ROTATION_MAX_VARIANCE) && (sqr(gesture_roll_angle) > sqr(ROTATION_MIN_ANGLE))) {
    if (gesture_roll_angle<0) {
      return ARM_ROTATE_CW;  
    } else {
      return ARM_ROTATE_CCW;  
    }
  }
  
  /***************************************************
   * Test for straight movement
   **************************************************/
   
  //determine the distance from (0,0)
  float distance = sqrt(getSqrPointDist(0, 0, gesture_cache[2 * num_points - 2], gesture_cache[2 * num_points - 1]));
  
  //horizontal movement?
  if ((x_deviation > y_deviation) && (relation > 1./STRAIGHT_MAX_RELATION) && (distance >= STRAIGHT_MIN_DISTANCE)) {
    if (x_total > 0) {
      return ARM_RIGHT;
    } else {
      return ARM_LEFT;  
    }
  }

  //vertical movement?
  if ((y_deviation > x_deviation) && (relation < STRAIGHT_MAX_RELATION) && (distance >= STRAIGHT_MIN_DISTANCE)) {
    if (y_total > 0) {
      return ARM_UP;
    } else {
      return ARM_DOWN;  
    }
  }


  //no gesture detected
  return ARM_UNKNOWN;
}
