#ifndef PID_H_
#define PID_H_

#include <stdbool.h>

#define PID_ROLL_KP  5.9
#define PID_ROLL_KI  2.9
#define PID_ROLL_KD  0.0
#define PID_ROLL_INTEGRATION_LIMIT    100.0

#define PID_PITCH_KP  5.9
#define PID_PITCH_KI  2.9
#define PID_PITCH_KD  0.0
#define PID_PITCH_INTEGRATION_LIMIT   100.0

#define PID_YAW_KP  6.0
#define PID_YAW_KI  0.0
#define PID_YAW_KD  0.35
#define PID_YAW_INTEGRATION_LIMIT     50.0


#define DEFAULT_PID_INTEGRATION_LIMIT 500.0
#define DEFAULT_PID_OUTPUT_LIMIT      300.0

typedef struct {
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1;
  float delay_element_2;
} lpf2pData;

typedef struct
{
  float desired;      //< set point
  float error;        //< error
  float prevError;    //< previous error
  float integ;        //< integral
  float deriv;        //< derivative
  float kp;           //< proportional gain
  float ki;           //< integral gain
  float kd;           //< derivative gain
  float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
  float iLimit;       //< integral limit, absolute value. '0' means no limit.
  float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
  float dt;           //< delta-time dt
  lpf2pData dFilter;  //< filter for D term
  bool enableDFilter; //< filter for D term enable flag
} PidObject;

/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] desired  The initial set point.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 * @param[in] dt        Delta time since the last call
 * @param[in] samplingRate Frequency the update will be called
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
 */
 void pidInit(PidObject* pid, const float desired, const float kp,
              const float ki, const float kd, const float dt, const float iLimit, bool enableDFilter);

float pidUpdate(PidObject* pid, const float measured, const bool updateError);

#endif