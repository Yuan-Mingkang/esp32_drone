#include "pid.h"
#include <math.h>
#include <float.h>


// /**
//  * 功能： 初始化一个低通滤波器的结构体，并设置滤波器的截止频率。
//  * lpf2pData* lpfData: 这是指向低通滤波器数据结构的指针，用于保存滤波器的状态。
//  * sample_freq: 采样频率，即信号被采集的频率。
//  * cutoff_freq: 截止频率，是滤波器开始衰减信号的频率。
//  */
// void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
// {
//   if (lpfData == NULL || cutoff_freq <= 0.0f) {
//     return;
//   }

//   lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
// }
// /**
//  * 功能： 设置滤波器的系数。这里实现了一个二阶低通滤波器，通过采样频率和截止频率来计算滤波器的参数。
//  */
// void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
// {
//   float fr = sample_freq/cutoff_freq;
//   float ohm = tanf(M_PI_F/fr);
//   float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm;
//   lpfData->b0 = ohm*ohm/c;
//   lpfData->b1 = 2.0f*lpfData->b0;
//   lpfData->b2 = lpfData->b0;
//   lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
//   lpfData->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
//   lpfData->delay_element_1 = 0.0f;
//   lpfData->delay_element_2 = 0.0f;
// }
// //功能： 应用低通滤波器，处理输入的采样数据并输出过滤后的信号。
// float lpf2pApply(lpf2pData* lpfData, float sample)
// {
//   float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
//   if (!isfinite(delay_element_0)) {
//     // don't allow bad values to propigate via the filter
//     delay_element_0 = sample;
//   }

//   float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

//   lpfData->delay_element_2 = lpfData->delay_element_1;
//   lpfData->delay_element_1 = delay_element_0;
//   return output;
// }
//功能： 将一个值限制在指定的最小值和最大值之间。
float constrain(float value, const float minVal, const float maxVal)
{
  return fminf(maxVal, fmaxf(minVal,value));
}

void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt, float iLimit, bool enableDFilter)
{
  pid->error         = 0;
  pid->prevError     = 0;
  pid->integ         = 0;
  pid->deriv         = 0;
  pid->desired       = desired;
  pid->kp            = kp;
  pid->ki            = ki;
  pid->kd            = kd;
  pid->iLimit        = iLimit;
  // pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;
  pid->dt            = dt;
  pid->enableDFilter = enableDFilter;
}

float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output = 0.0f;

    if (updateError)
    {
        pid->error = pid->desired - measured;
    }

    pid->outP = pid->kp * pid->error;
    output += pid->outP;

    float deriv = (pid->error - pid->prevError) / pid->dt;
    if (pid->enableDFilter)
    {
      // pid->deriv = lpf2pApply(&pid->dFilter, deriv);
    } else {
      pid->deriv = deriv;
    }
    if (isnan(pid->deriv)) {
      pid->deriv = 0;
    }
    pid->outD = pid->kd * pid->deriv;
    output += pid->outD;

    pid->integ += pid->error * pid->dt;

    // Constrain the integral (unless the iLimit is zero)
    if(pid->iLimit != 0)
    {
    	pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
    }

    pid->outI = pid->ki * pid->integ;
    output += pid->outI;

    // Constrain the total PID output (unless the outputLimit is zero)
    if(pid->outputLimit != 0)
    {
      output = constrain(output, -pid->outputLimit, pid->outputLimit);
    }


    pid->prevError = pid->error;

    return output;
}