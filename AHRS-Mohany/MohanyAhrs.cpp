// AHRS-Mohany.cpp: 定义控制台应用程序的入口点。
//=====================================================================================================
// Madgwick's implementation of Mayhony's AHRS algorithm.
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//=====================================================================================================
#include "MahonyAhrs.h"
#include <math.h>
#include<stdio.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	 320.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.000f)	// 2 * integral gain
#define pi 3.141592653f
float ecompassyaw;
 volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
 float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
 float twoKi = twoKiDef;											// 2 * integral gain (Ki)
 float  q0 = 1;
 float  q1 = 0;
 float	q2 = 0;
 float	q3 = 0;

//====================================================================================================
// Functions
// AHRS algorithm update

void MahonyAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float rpy[3]) {
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy,hz, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

#if 1
	ecompassyaw = atan2(mx, my);//坐标系原因
	if (ecompassyaw<0.0)
	{
		ecompassyaw = ecompassyaw + 2 * pi;
	}   // [0-2*pi] how to cooperate yaw and rtk.hesding not in plane now .
	if (ecompassyaw>2 * pi)
	{
		ecompassyaw = ecompassyaw - 2 * pi;
	}
	   ecompassyaw = ecompassyaw * 180 / pi;
#endif
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1 / sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = 1 / sqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
		//这里相当于磁力计的三轴矢量右乘DCM  得到的是磁力计旋转到导航系（水平）下的分量
		bx = sqrt(hx * hx + hy * hy);//h  is horizon 水平
		bz = hz;
		// [0 0 g]*nCb  
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = 0.5f - q1q1 - q2q2;//hz 中的右乘的矢量wxyz  同时也是[0 0 1]*g的
		//real mag* nCb
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q2q3+ q0q1);
		halfwz = bx * (q1q3+ q0q2) + bz * (0.5f - q1q1 - q2q2);
		//根据当前四元数的姿态值来估算出各重力分量Vx，Vy，Vz和各地磁分量Wx，Wy，Wz
		//使用叉积来计算重力和地磁误差
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
		/*
		axyz是机体坐标系在参照系上的加速度计输出。
		vxyz计算输出的重力分解向量，它们都是机体坐标参照系上的重力向量。
		*/
		//把上述计算得到的重力和磁力差进行积分运算
	
#if 0
		if (twoKi > 0.0f) {//if(ex != 0.0f && ey != 0.0f && ez != 0.0f)// 在另一个函数中他这样用  因此在保密时候就可以对这些运算就行增减
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			// 用叉积误差来做PI修正陀螺零偏
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
#endif
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	
	//if(abs(gz<0.1))
	//{ gz = 0.0; }
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = 1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	//********************																										
	//if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}
//-------------------------------------------
void quater2rpy(float q1, float q2, float q3, float q4, float rpy_mohany[3])
{
	float r11, r21, r31, r32, r33;
	
	q2 = -q2;
	q3 = -q3;
	q4 = -q4;
	r11 = -1+2 * q1*q1 + 2 * q2*q2;
	r21 = 2 * (q2*q3 - q1 * q4);
	r31 = 2 * (q2*q4 + q1 * q3);
	r32 = 2 * (q3*q4 - q1 * q2);
	r33 = -1+2 * q1*q1  + 2 * q4*q4;

	rpy_mohany[0]  =  atan2(r32, r33) * (180/pi);
	rpy_mohany[1]  = -atan (r31 / sqrt(1 - r31*r31))* (180/pi);
	//rpy_mohany[2] = 2*pi-atan2(r21, r11);// *(180 / pi);
	rpy_mohany[2] =  atan2(r21, r11);// *(180 / pi);
	if (rpy_mohany[2]<0.0)
	{
		rpy_mohany[2] = rpy_mohany[2] + 2 * pi;
	}   // [0-2*pi] how to cooperate yaw and rtk.hesding not in plane now .
	if (rpy_mohany[2]>2 * pi)
	{
		rpy_mohany[2] = rpy_mohany[2] - 2 * pi;
	}
	rpy_mohany[2] = rpy_mohany[2] * 180 / pi;
}
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1/sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		//if (twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		//}
		//else {
		//	integralFBx = 0.0f;	// prevent integral windup
		//	integralFBy = 0.0f;
		//	integralFBz = 0.0f;
		//}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = 1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
// 这里使用可靠但是动态性较差的融合算法实现航姿的可靠输出。
#if 0
void ShileiAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float rpy[3]) {
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, hz, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	mx = mx - 70;
	my = my - 90;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1 / sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = 1 / sqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
		//这里相当于磁力计的三轴矢量右乘DCM  得到的是磁力计旋转到导航系（水平）下的分量
		bx = sqrt(hx * hx + hy * hy);//h  is horizon 水平
		bz = hz;
		// [0 0 g]*nCb  
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = 0.5f - q1q1 - q2q2;//hz 中的右乘的矢量wxyz  同时也是[0 0 1]*g的
									//real mag* nCb
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q2q3 + q0q1);
		halfwz = bx * (q1q3 + q0q2) + bz * (0.5f - q1q1 - q2q2);
		//根据当前四元数的姿态值来估算出各重力分量Vx，Vy，Vz和各地磁分量Wx，Wy，Wz
		//使用叉积来计算重力和地磁误差
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
		/*
		axyz是机体坐标系在参照系上的加速度计输出。
		vxyz计算输出的重力分解向量，它们都是机体坐标参照系上的重力向量。
		*/
		//把上述计算得到的重力和磁力差进行积分运算

#if 0
		if (twoKi > 0.0f) {//if(ex != 0.0f && ey != 0.0f && ez != 0.0f)// 在另一个函数中他这样用  因此在保密时候就可以对这些运算就行增减
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			// 用叉积误差来做PI修正陀螺零偏
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
#endif
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));

	//if(abs(gz<0.1))
	//{ gz = 0.0; }
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	//********************																										
	//if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}
#endif

//====================================================================================================
// END OF CODE
//====================================================================================================


/*************************************2018.2.2 SL ***********************************************/
