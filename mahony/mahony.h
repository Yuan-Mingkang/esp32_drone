#ifndef MAHONY_H_
#define MAHONY_H_

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif