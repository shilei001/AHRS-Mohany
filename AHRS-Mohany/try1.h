//#pragma once
//#ifndef __IMU_H
//#define __IMU_H
//
//#include "common.h"  //包含所有的驱动 头文件
//#include "pprz_algebra_int.h"
//#include "pprz_algebra_float.h"
//
//#include <math.h>
//
//#ifndef M_PI
//#define M_PI 3.14159265358979323846
//#endif
//
//
///** abstract IMU interface providing fixed point interface  */
//struct Imu {
//	struct Int32Rates gyro;             ///< gyroscope measurements
//	struct Int32Vect3 accel;            ///< accelerometer measurements
//	struct Int32Vect3 mag;              ///< magnetometer measurements
//	struct Int32Rates gyro_prev;        ///< previous gyroscope measurements
//	struct Int32Vect3 accel_prev;       ///< previous accelerometer measurements
//	struct Int32Rates gyro_neutral;     ///< gyroscope bias
//	struct Int32Vect3 accel_neutral;    ///< accelerometer bias
//	struct Int32Vect3 mag_neutral;      ///< magnetometer neutral readings (bias)
//	struct Int32Rates gyro_unscaled;    ///< unscaled gyroscope measurements
//	struct Int32Vect3 accel_unscaled;   ///< unscaled accelerometer measurements
//	struct Int32Vect3 mag_unscaled;     ///< unscaled magnetometer measurements
//	struct Int32Quat  body_to_imu_quat; ///< rotation from body to imu frame as a unit quaternion
//	struct Int32RMat  body_to_imu_rmat; ///< rotation from body to imu frame as a rotation matrix
//};
//
///** abstract IMU interface providing floating point interface  */
//struct ImuFloat {
//	struct FloatRates   gyro;
//	struct FloatVect3   accel;
//	struct FloatVect3   mag;
//	struct FloatRates   gyro_prev;
//	struct FloatEulers  body_to_imu_eulers;
//	struct FloatQuat    body_to_imu_quat;
//	struct FloatRMat    body_to_imu_rmat;
//	uint32_t sample_count;
//};
//
//#if !defined IMU_BODY_TO_IMU_PHI && !defined IMU_BODY_TO_IMU_THETA && !defined IMU_BODY_TO_IMU_PSI
//#define IMU_BODY_TO_IMU_PHI   0
//#define IMU_BODY_TO_IMU_THETA 0
//#define IMU_BODY_TO_IMU_PSI   0
//#endif
//
//#if !defined IMU_GYRO_P_NEUTRAL && !defined IMU_GYRO_Q_NEUTRAL && !defined IMU_GYRO_R_NEUTRAL
//#define IMU_GYRO_P_NEUTRAL 0
//#define IMU_GYRO_Q_NEUTRAL 0
//#define IMU_GYRO_R_NEUTRAL 0
//#endif
//
//#if !defined IMU_ACCEL_X_NEUTRAL && !defined IMU_ACCEL_Y_NEUTRAL && !defined IMU_ACCEL_Z_NEUTRAL
//#define IMU_ACCEL_X_NEUTRAL 0
//#define IMU_ACCEL_Y_NEUTRAL 0
//#define IMU_ACCEL_Z_NEUTRAL 0
//#endif
//
//
//#ifndef ImuScaleGyro
//#define ImuScaleGyro(_imu) {					\
//    RATES_COPY(_imu.gyro_prev, _imu.gyro);				\
//    _imu.gyro.p = ((_imu.gyro_unscaled.p - _imu.gyro_neutral.p)*IMU_GYRO_P_SIGN*IMU_GYRO_P_SENS_NUM)/IMU_GYRO_P_SENS_DEN; \
//    _imu.gyro.q = ((_imu.gyro_unscaled.q - _imu.gyro_neutral.q)*IMU_GYRO_Q_SIGN*IMU_GYRO_Q_SENS_NUM)/IMU_GYRO_Q_SENS_DEN; \
//    _imu.gyro.r = ((_imu.gyro_unscaled.r - _imu.gyro_neutral.r)*IMU_GYRO_R_SIGN*IMU_GYRO_R_SENS_NUM)/IMU_GYRO_R_SENS_DEN; \
//  }
//#endif
//
//#ifndef ImuScaleAccel
//#define ImuScaleAccel(_imu) {					\
//    VECT3_COPY(_imu.accel_prev, _imu.accel);				\
//    _imu.accel.x = ((_imu.accel_unscaled.x - _imu.accel_neutral.x)*IMU_ACCEL_X_SIGN*IMU_ACCEL_X_SENS_NUM)/IMU_ACCEL_X_SENS_DEN; \
//    _imu.accel.y = ((_imu.accel_unscaled.y - _imu.accel_neutral.y)*IMU_ACCEL_Y_SIGN*IMU_ACCEL_Y_SENS_NUM)/IMU_ACCEL_Y_SENS_DEN; \
//    _imu.accel.z = ((_imu.accel_unscaled.z - _imu.accel_neutral.z)*IMU_ACCEL_Z_SIGN*IMU_ACCEL_Z_SENS_NUM)/IMU_ACCEL_Z_SENS_DEN; \
//  }
//#endif
//
//#ifndef ImuScaleMag
//#define ImuScaleMag(_imu) {						\
//    _imu.mag.x = ((_imu.mag_unscaled.x - _imu.mag_neutral.x) * IMU_MAG_X_SIGN * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN; \
//    _imu.mag.y = ((_imu.mag_unscaled.y - _imu.mag_neutral.y) * IMU_MAG_Y_SIGN * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN; \
//    _imu.mag.z = ((_imu.mag_unscaled.z - _imu.mag_neutral.z) * IMU_MAG_Z_SIGN * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN; \
//  }
//#endif //ImuScaleMag
//
//#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
//#define IMU_MAG_X_SIGN 1
//#define IMU_MAG_Y_SIGN 1
//#define IMU_MAG_Z_SIGN 1
//#endif
//
//#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
//#define IMU_GYRO_P_SIGN   1
//#define IMU_GYRO_Q_SIGN   1
//#define IMU_GYRO_R_SIGN   1
//#endif
//#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
//#define IMU_ACCEL_X_SIGN  1
//#define IMU_ACCEL_Y_SIGN  1
//#define IMU_ACCEL_Z_SIGN  1
//#endif
//
///** default gyro sensitivy and neutral from the datasheet
//* MPU60X0 has 32.8 LSB/(deg/s) at 1000deg/s range
//* sens = 1/32.8 * pi/180 * 2^INT32_RATE_FRAC
//* sens = 1/32.8 * pi/180 * 4096 = 2.179533
//*/
//#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
//#define IMU_GYRO_P_SENS 2.179
//#define IMU_GYRO_P_SENS_NUM 2179
//#define IMU_GYRO_P_SENS_DEN 1000
//#define IMU_GYRO_Q_SENS 2.179
//#define IMU_GYRO_Q_SENS_NUM 2179
//#define IMU_GYRO_Q_SENS_DEN 1000
//#define IMU_GYRO_R_SENS 2.179
//#define IMU_GYRO_R_SENS_NUM 2179
//#define IMU_GYRO_R_SENS_DEN 1000
//#endif
//#if !defined IMU_GYRO_P_NEUTRAL & !defined IMU_GYRO_Q_NEUTRAL & !defined IMU_GYRO_R_NEUTRAL
//#define IMU_GYRO_P_NEUTRAL 0
//#define IMU_GYRO_Q_NEUTRAL 0
//#define IMU_GYRO_R_NEUTRAL 0
//#endif
//
//#if !defined IMU_MAG_X_SENS_NUM	& !defined IMU_MAG_Y_SENS_NUM & !defined IMU_MAG_Z_SENS_NUM
//#define IMU_MAG_X_SENS_NUM	  1
//#define IMU_MAG_Y_SENS_NUM	  1
//#define IMU_MAG_Z_SENS_NUM	  1
//#endif
//
//#if !defined IMU_MAG_X_SENS_DEN & !defined IMU_MAG_Y_SENS_DEN & !defined IMU_MAG_Z_SENS_DEN 
//#define IMU_MAG_X_SENS_DEN   1
//#define IMU_MAG_Y_SENS_DEN   1
//#define IMU_MAG_Z_SENS_DEN   1
//#endif
//
///** default accel sensitivy from the datasheet
//* MPU60X0 has 2048 LSB/g
//* fixed point sens: 9.81 [m/s^2] / 2048 [LSB/g] * 2^INT32_ACCEL_FRAC
//* sens = 9.81 / 2048 * 1024 = 4.905
//*/
//#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
//#define IMU_ACCEL_X_SENS 4.905
//#define IMU_ACCEL_X_SENS_NUM 4905
//#define IMU_ACCEL_X_SENS_DEN 1000
//#define IMU_ACCEL_Y_SENS 4.905
//#define IMU_ACCEL_Y_SENS_NUM 4905
//#define IMU_ACCEL_Y_SENS_DEN 1000
//#define IMU_ACCEL_Z_SENS 4.905
//#define IMU_ACCEL_Z_SENS_NUM 4905
//#define IMU_ACCEL_Z_SENS_DEN 1000
//#endif
//#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
//#define IMU_ACCEL_X_NEUTRAL 0
//#define IMU_ACCEL_Y_NEUTRAL 0
//#define IMU_ACCEL_Z_NEUTRAL 0
//#endif
//
///** global IMU state */
//extern struct Imu imu;
//
//
////Mini IMU AHRS 解算的API
//void IMU_init(void); //初始化
//void IMU_getYawPitchRoll(float * ypr); //更新姿态
//uint32_t micros(void);	//读取系统上电后的时间  单位 us 
//void Get_IMU_Sesor(void);
//#endif
//
////------------------End of File----------------------------
