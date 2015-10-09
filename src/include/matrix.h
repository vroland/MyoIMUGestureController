/**
 * @file   matrix.h
 * @author Valentin Roland (webmaster at vroland.de)
 * @date   September-October 2015
 * @brief  Header file defining the Matrix33 type with utility functions.
 *
 * This library provides gesture detection functionality using almost exclusively the IMU data of the Myo Armband.
 * The gestures are based on arm rotation to work with persons where distinct muscle activity is hard to detect.
 * Muscle activity is only used for starting/ending the recording of a gesture. Uses the MyoBridge Arduino Library (https://github.com/vroland/MyoBridge).
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <Arduino.h>

/// 3x3 array of zeros, used for matrix initialization.
#define ZERO_MATRIX {{0, 0, 0},{0, 0, 0},{0, 0, 0}}

/**
 * 3x3 Matrix type. 
 */
typedef float Matrix33[3][3];

/**
 * print a matrix to hardware serial
 */
void print_matrix(Matrix33 matrix);

/**
 * print a vector to hardware serial
 */
void print_vector(float* vec);

/**
 * multiply two matrices in an already initialized new one
 * http://stackoverflow.com/questions/5670596/multiplying-two-3x3-matrices-in-c
 */
void multiply_matrix(Matrix33 a, Matrix33 b, Matrix33 &result);

/**
 * Multiply a matrix and a vector
 */
void multiply_matrix_vector(Matrix33 a, float* v, float* result);

/**
 * write inverse matrix of in to out
 * http://stackoverflow.com/questions/983999/simple-3x3-matrix-inverse-code-c
 */
void inverse_matrix(Matrix33 in, Matrix33 &out);

/**
 * convert a myo unit quaternion to a 3x3 matrix
 * http://www.cprogramming.com/tutorial/3d/quaternions.html
 */
void unit_quaternion_to_matrix(Matrix33 &matrix, int16_t* quat);

#endif //MATRIX_H
