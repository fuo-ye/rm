
#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "includes.h"
#include "motor.h"

//#define Chassis_Use_IMU


#define Chassis_Vr_FFC_MAXOUT 800
#define Chassis_Vr_FCC_LPF 0.001
#define Chassis_Vr_C0 1
#define Chassis_Vr_C1 1
#define Chassis_Vr_C2 1
#define Change_Boundary 500

#define CHASSIS_TASK_PERIOD 2

#if INFANTRY_ID == 2 /*麦轮步兵*/
#define TwoHeading
//	#define FourHeading
#define UseAttitudeControl
#define GRAVUTYCENTER_ADJUSTMENT 1.0f;

#define POWER_GAIN 1.13f  //功率修正系数
#define LOWVOLTAGE_GAIN 0.7
#elif INFANTRY_ID == 3 /*全向轮步兵*/
//	#define TwoHeading
#define FourHeading
#define UseAttitudeControl
#define GRAVUTYCENTER_ADJUSTMENT 0.982f;

#define POWER_GAIN 1.13f  //功率修正系数
#define LOWVOLTAGE_GAIN 0.7
#elif INFANTRY_ID == 4 /*全向轮步兵*/
//	#define TwoHeading
#define FourHeading
#define UseAttitudeControl
#define GRAVUTYCENTER_ADJUSTMENT 0.982f;

#define POWER_GAIN 1.0f  //功率修正系数
#define LOWVOLTAGE_GAIN 0.7

#endif

#define ENABLE_SPINNING
// #define DISABLE_SPINNING

#define FOLLOW_DEAD_BAND 10.0f

#define VELOCITY_RATIO 5
#define RC_STICK_ROTATE_RATIO 0.72
#define RC_MOUSE_ROTATE_RATIO 0.5f

#define wheel_radius 76.00f
#define Car_body_radius 380.271f
#define pi 3.1415926f
#define Kx 0.250f
#define Ky 0.250f

#define YAW_REDUCTION_RATIO 1 / 1.0f  //YAW轴电机的传动系统的减速比

#define YAW_REDUCTION_CORRECTION_ANGLE 90.0f

#ifdef ARM_MATH_DSP
#define user_cos arm_cos_f32
#define user_sin arm_sin_f32
#else
#define user_cos cosf
#define user_sin sinf
#endif
/*未采用*/
typedef struct
{
  float slip;  /*???*/
  float V[2];  /*???? m/s*/
  float omiga; /*???? rad/s*/
  float R;     /*???? m*/

  float BodyAccel[2]; /*?????*/
} Abs_Control_t;
typedef struct
{
  float PowerScale; /*功率范围*/
  float Capacitor_Voltage; /*电容电压*/
  float Power_Limit; /*功率限制*/
  float Power_Calculation; /*限幅之前的功率*/
  float Power_Calculation_Clipping; /*限幅之后的实际功率*/
  float Power_correction_gain;  /*功率修正系数（同POWER_GAIN，防止机器人超功率）*/
  float LowVoltage_Gain;  /*电压修正系数（防止电容电压过低）*/
  float sec_PowerScale;
} Chassis_PowerControl_t;

typedef struct _Chassis_t
{
  int16_t Vx, Vy, Vr; /*XY轴速度与角速度 */
  int16_t LastVr ;
  float VxTransfer, VyTransfer; /*用于云台坐标系与底盘坐标系的转换*/
  float AngularVelocity;  /*角速度*/

  float VelocityRatio;  /*调整小陀螺在总功率中的占比*/
  float rcStickRotateRatio; /*摇杆运动与电机间的比例系数*/
  float rcMouseRotateRatio; /*鼠标运动与电机间的比例系数*/

  float V1, V2, V3, V4;

  float Vx_is_Chassis, Vy_is_Chassis; /*底盘坐标系下反解出的车速度*/
  float Vx_is_Body, Vy_is_Body; /*云台坐标系下反解出的车速度*/
  float Ax_Chassis, Ay_Chassis; /*底盘坐标系下在X,Y方向上的加速度*/
  float Ax_Body, Ay_Body; /*云台坐标系下在X,Y方向上的加速度*/
  float V1_is, V2_is, V3_is, V4_is; /*反解出四个轮子的速度*/

  float x;
  float y;

  float Chassis_Offsetangle;

  uint8_t Mode;

  int YawCorrectionScale; /*YAW修正后的范围*/
  int8_t HeadingFlag; /*0正面 1左 -1右  2背面*/ //步兵车头方向的转换
  int8_t HeadingFlag_last;
  uint32_t ChassisSwitchTick;

  float GravityCenter_Adjustment; /*云台重心修正*/
  float Heading;
  float Theta;
  float TotalTheta;
  float FollowTheta;
  float DeflectionAngle;
  
  float cap_energy;
  float energy_percentage;

  float Attitude_adjustment[4];
  float Differential_Gain[2]; // 0??? 1???

  Chassis_PowerControl_t PowerControl;
  
  Motor_t ChassisMotor[4];
  PID_t RotateFollow;
  PID_t ChassisVr;

  TD_t ChassisVxTD;
  TD_t ChassisVyTD;

  TD_t ChassisVxTD_s;
  TD_t ChassisVyTD_s;
  Abs_Control_t Abs_Control;
} Chassis_t;

enum
{
  Follow_Mode = 0,  //跟随
  Spinning_Mode,  //小陀螺
  Side_Mode,  //横着走
  Silence_Mode, //静止模式
};

void Chassis_Control(void);
void Chassis_Init(void);

extern Chassis_t Chassis;

#endif
