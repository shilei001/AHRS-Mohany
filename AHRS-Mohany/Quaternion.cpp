#include"Quaternion.h"
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x)
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 1.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.53f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0, tempq1, tempq2, tempq3;

	// 先把这些用得到的值算好
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	now = micros();  //读取时间
	if (now<lastUpdate) { //定时器溢出过了。
		halfT = ((float)(now + (0xffff - lastUpdate)) / 2000000.0f);
	}
	else {
		halfT = ((float)(now - lastUpdate) / 2000000.0f);
	}
	lastUpdate = now;	//更新时间

	norm = invSqrt(ax*ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	//把加计的三维向量转成单位向量。

	norm = invSqrt(mx*mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;

	/*
	这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
	*/
	// compute reference direction of flux
	hx = 2 * mx*(0.5f - q2q2 - q3q3) + 2 * my*(q1q2 - q0q3) + 2 * mz*(q1q3 + q0q2);
	hy = 2 * mx*(q1q2 + q0q3) + 2 * my*(0.5f - q1q1 - q3q3) + 2 * mz*(q2q3 - q0q1);
	hz = 2 * mx*(q1q3 - q0q2) + 2 * my*(q2q3 + q0q1) + 2 * mz*(0.5f - q1q1 - q2q2);
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;

	// estimated direction of gravity and flux (v and w)
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2 * bx*(0.5 - q2q2 - q3q3) + 2 * bz*(q1q3 - q0q2);
	wy = 2 * bx*(q1q2 - q0q3) + 2 * bz*(q0q1 + q2q3);
	wz = 2 * bx*(q0q2 + q1q3) + 2 * bz*(0.5 - q1q1 - q2q2);

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az * vy) + (my*wz - mz * wy);
	ey = (az*vx - ax * vz) + (mz*wx - mx * wz);
	ez = (ax*vy - ay * vx) + (mx*wy - my * wx);

	/*
	axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
	axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
	那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
	向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
	这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
	*/
	if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		// 用叉积误差来做PI修正陀螺零偏
		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;

	}

	// 四元数微分方程
	tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz)*halfT;
	tempq1 = q1 + (q0*gx + q2 * gz - q3 * gy)*halfT;
	tempq2 = q2 + (q0*gy - q1 * gz + q3 * gx)*halfT;
	tempq3 = q3 + (q0*gz + q1 * gy - q2 * gx)*halfT;

	// 四元数规范化
	norm = invSqrt(tempq0*tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}



/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getYawPitchRoll(float * angles) {
	float q[4]; //　四元数
	volatile float gx = 0.0, gy = 0.0, gz = 0.0; //估计重力方向
	IMU_getQ(q); //更新全局四元数

	angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) * 180 / M_PI; // yaw
	angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180 / M_PI; // pitch
	angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 180 / M_PI; // roll
																											   //if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}


