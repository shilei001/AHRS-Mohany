#pragma once
//=====================================================================================================
// MahonyAHRS.h
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//=====================================================================================================
#ifndef MahonyAhrs_h
#define MahonyAhrs_h
//----------------------------------------------------------------------------------------------------
// Variable declaration
//extern volatile float twoKp;			// 2 * proportional gain (Kp)
//extern volatile float twoKi;			// 2 * integral gain (Ki)
//extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
//---------------------------------------------------------------------------------------------------
// Function declarations
void MahonyAHRSupdate( float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float rpy[3]);
void quater2rpy(float q1, float q2, float q3, float q4, float  rpy_mohany[3]);
void MahonyAHRSupdateIMU( float ax, float ay, float az,float gx, float gy, float gz );
//float invSqrt(float x);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
