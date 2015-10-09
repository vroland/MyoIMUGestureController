/**
 * @file   matrix.cpp
 * @author Valentin Roland (webmaster at vroland.de)
 * @date   September-October 2015
 * @brief  Implementation file for matrix operations.
 *
 * This library provides gesture detection functionality using almost exclusively the IMU data of the Myo Armband.
 * The gestures are based on arm rotation to work with persons where distinct muscle activity is hard to detect.
 * Muscle activity is only used for starting/ending the recording of a gesture. Uses the MyoBridge Arduino Library (https://github.com/vroland/MyoBridge).
 */

#include "matrix.h"

#include <MyoBridge.h>

//vector dot product
float dotp(float* v1, float* v2) {
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

inline float sqr(float a) {
  return a*a;
}

inline float clip(float n, float lower, float upper) {
  return max(lower, min(n, upper));
}

//squared distance of two points represented as vectors
float sqr_dist(float* v1, float* v2) {
  return sqr(v1[0]-v2[0])+sqr(v1[1]-v2[1])+sqr(v1[2]-v2[2]);
}

/**
 * print a matrix to hardware serial
 */
void print_matrix(Matrix33 matrix) {
  
  Serial.print(matrix[0][0]);
  Serial.print(" ");
  Serial.print(matrix[0][1]);
  Serial.print(" ");
  Serial.println(matrix[0][2]);
  Serial.print(matrix[1][0]);
  Serial.print(" ");
  Serial.print(matrix[1][1]);
  Serial.print(" ");
  Serial.println(matrix[1][2]);
  Serial.print(matrix[2][0]);
  Serial.print(" ");
  Serial.print(matrix[2][1]);
  Serial.print(" ");
  Serial.println(matrix[2][2]);
}

/**
 * print a vector to hardware serial
 */
void print_vector(float* vec) {

  Serial.print(vec[0]);
  Serial.print(" ");
  Serial.print(vec[1]);
  Serial.print(" ");
  Serial.println(vec[2]);
}

//multiply two matrices in an already initialized new one
//http://stackoverflow.com/questions/5670596/multiplying-two-3x3-matrices-in-c
void multiply_matrix(Matrix33 a, Matrix33 b, Matrix33 &result) {
  int i, j, k;
  for(i = 0; i < 3; i++) {
    for(j = 0; j < 3; j++) {
      result[i][j] = 0;
      for(k = 0; k < 3; k++) {
        result[i][j] +=  a[i][k] *  b[k][j];
      }
    }
  }
}

/**
 * Multiply a matrix and a vector
 */
void multiply_matrix_vector(Matrix33 a, float* v, float* result) {
     for (int i=0;i<3;i++){
        result[i] = 0;
        for (int j=0;j<3;j++){
            result[i]+=( a[i][j]*v[j]);
        }
    }
}

/**
 * write inverse matrix of in to out
 * http://stackoverflow.com/questions/983999/simple-3x3-matrix-inverse-code-c
 */
void inverse_matrix(Matrix33 in, Matrix33 &out) {
  
  float determinant = +in[0][0]*(in[1][1]*in[2][2]-in[2][1]*in[1][2])
                        -in[0][1]*(in[1][0]*in[2][2]-in[1][2]*in[2][0])
                        +in[0][2]*(in[1][0]*in[2][1]-in[1][1]*in[2][0]);
                        
  float invdet = 1./determinant;
  
  out[0][ 0] = (in[1][ 1] * in[2][ 2] - in[2][ 1] * in[1][ 2]) * invdet;
  out[0][ 1] = (in[0][ 2] * in[2][ 1] - in[0][ 1] * in[2][ 2]) * invdet;
  out[0][ 2] = (in[0][ 1] * in[1][ 2] - in[0][ 2] * in[1][ 1]) * invdet;
  out[1][ 0] = (in[1][ 2] * in[2][ 0] - in[1][ 0] * in[2][ 2]) * invdet;
  out[1][ 1] = (in[0][ 0] * in[2][ 2] - in[0][ 2] * in[2][ 0]) * invdet;
  out[1][ 2] = (in[1][ 0] * in[0][ 2] - in[0][ 0] * in[1][ 2]) * invdet;
  out[2][ 0] = (in[1][ 0] * in[2][ 1] - in[2][ 0] * in[1][ 1]) * invdet;
  out[2][ 1] = (in[2][ 0] * in[0][ 1] - in[0][ 0] * in[2][ 1]) * invdet;
  out[2][ 2] = (in[0][ 0] * in[1][ 1] - in[1][ 0] * in[0][ 1]) * invdet;
}

/**
 * convert a myo unit quaternion to a 3x3 matrix
 * http://www.cprogramming.com/tutorial/3d/quaternions.html
 */
void unit_quaternion_to_matrix(Matrix33 &matrix, int16_t* quat) {

  //convert the raw data in unit quaternion of floats. 
  float x = clip((float)quat[0] / (MYOHW_ORIENTATION_SCALE), -.999999, .999999); 
  float y = clip((float)quat[1] / (MYOHW_ORIENTATION_SCALE), -.999999, .999999); 
  float z = clip((float)quat[2] / (MYOHW_ORIENTATION_SCALE), -.999999, .999999); 
  float w = clip((float)quat[3] / (MYOHW_ORIENTATION_SCALE), -.999999, .999999); 
  
  matrix[0][0] = 1-2*y*y-2*z*z;
  matrix[0][1] = 2*x*y-2*w*z;
  matrix[0][2] = 2*x*z+2*w*y;

  matrix[1][0] = 2*x*y+2*w*z;
  matrix[1][1] = 1-2*x*x-2*z*z;
  matrix[1][2] = 2*y*z-2*w*x;

  matrix[2][0] = 2*x*z-2*w*y;
  matrix[2][1] = 2*y*z+2*w*x;
  matrix[2][2] = 1-2*x*x-2*y*y;
}
