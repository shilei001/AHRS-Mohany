// AHRS-Mohany.cpp: 定义控制台应用程序的入口点。
//
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "MahonyAhrs.h"

#define pi 3.141592653f
extern float   q0;
extern float   q1;
extern float   q2;
extern float   q3;
extern float ecompassyaw;


int main()
{
	struct type_imu
	{
		float ax = 0.0;
		float ay = 0.0; 
	    float az = 0.0;
		float gx = 0.0; 
		float gy = 0.0; 
		float gz = 0.0;
		float mx = 0.0; 
		float my = 0.0; 
		float mz = 0.0;
		float time=0.0;
		float rpy[3] = { 0.0 };
	}imu;


	//float acc, gyro, mag;
	FILE *file1;
	FILE *file2; 

	char txtline[1024];
	char datapath1[1024] = "F:\\HG-UWBRTKINS\\组合测试和过程记录\\rawdata\\C++verify\\mpudataed2.txt";
	char datapath2[1024] = "F:\\HG-UWBRTKINS\\组合测试和过程记录\\rawdata\\C++verify";
	char datapath3[1024];
	file1 = fopen(datapath1, "r");
	if (file1 == NULL)
	{
		printf("error in reading imufile.txt");
		return 0;

	}
	strcpy(datapath3, datapath2);
	strcat(datapath3, "\\imufile-out.txt");
	file2 = fopen(datapath3, "w");
	if (file2 == NULL)
	{
		printf("error in wrighting  imuflieoutput.txt");
	}

			while (!feof(file1))
			{
				auto i = fgets(txtline, sizeof(txtline), file1);
				if (i == NULL)
				{
					printf("error in reading imufiles'line");
				}
				else
				{
					sscanf(txtline,"%f %f %f %f %f %f %f %f %f %f\r\n",
						&imu.time, &imu.ax, &imu.ay, &imu.az,&imu.gx, &imu.gy,&imu.gz,&imu.mx, &imu.my, &imu.mz);
					//sscanf("0.107 0.354", "%f %f\n",&imu.time, &imu.ax);
				}
			
			     // MahonyAHRSupdate(imu.ax, imu.ay, imu.az, imu.gx*(pi/180), imu.gy*(pi/180), imu.gz*(pi/180), imu.mx, imu.my, imu.mz,imu.rpy);
				MahonyAHRSupdateIMU(imu.ax, imu.ay, imu.az, imu.gx*(pi / 180), imu.gy*(pi / 180), imu.gz*(pi / 180));
				quater2rpy(q0, q1, q2, q3, imu.rpy);

				fprintf(file2,"%8.5f %3.2f %3.2f %3.2f  \n", imu.time,imu.rpy[0],imu.rpy[1],imu.rpy[2]);
				//printf("time=%8.5f Roll=%3.2f Pitch=%3.2f Yaw=%3.2f \n", imu.time, imu.rpy[0], imu.rpy[1], imu.rpy[2]);

			}


			return 0;
}


