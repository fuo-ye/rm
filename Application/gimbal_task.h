
#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "includes.h"

#define GIMBAL_TASK_PERIOD 1

// Gimbal motor define
#if INFANTRY_ID == 1
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define YAW_MOTOR_ZERO_OFFSET 5365
#define PITCH_MOTOR_ZERO_OFFSET 2903
#endif

#if INFANTRY_ID == 2
#define YAW_MOTOR_ID 0x206
#define PITCH_MOTOR_ID 0x205
#define YAW_MOTOR_ZERO_OFFSET 4265
#define PITCH_MOTOR_ZERO_OFFSET 2335
#endif

#if INFANTRY_ID == 3
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define YAW_MOTOR_ZERO_OFFSET 6841
#define PITCH_MOTOR_ZERO_OFFSET 6533
#endif
#if INFANTRY_ID == 4
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define YAW_MOTOR_ZERO_OFFSET 6235
#define PITCH_MOTOR_ZERO_OFFSET 6533
#endif

enum
{
  Normal_Mode = 0X00,      // 0000 0000
  AimAssist_Mode = 0x01,   // 0000 0001
  Calibration_Mode = 0x02, // 0000 0010
  Tuning_Mode = 0x04,      // 0000 0100
  Sniping_Mode = 0x08,     // 0000 1000
  NONEEEE = 0x10,          // 0001 0000
  NONEEEEE = 0x20,         // 0010 0000
  NONEEEEEE = 0x40,        // 0100 0000
  NONEEEEEEE = 0x80,       // 1000 0000
};

typedef struct _GimbalControl
{
  Motor_t YawMotor;
  Motor_t PitchMotor;

  float YawAngle;
  float PitchAngle;
  float EncoderYawAngle;
  float EncoderPitchAngle; // ��Pitch�����������õ��������̬��
  float YawAngularVelocity;
  float PitchAngularVelocity;

  TD_t YawRefAngularVelocityTD;
  TD_t PitchRefAngularVelocityTD;
  TD_t YawRefAngleTD;
  TD_t PitchRefAngleTD;

  float YawRefAngularVelocity;
  float PitchRefAngularVelocity;
  float YawCtrlAngle;
  float PitchCtrlAngle;

  float YawErrSqrtK;
  float PitchErrSqrtK;

  float rcStickYawRatio;
  float rcStickPitchRatio;
  float rcMouseYawRatio;
  float rcMousePitchRatio;

  uint16_t ModeSwitchCount;

  uint8_t LaserState;
  uint8_t Mode;
  uint8_t ModeLast;

  uint16_t YawFrontEncoder;        // ��̨������ǰʱYaw��������ֵ
  uint16_t PitchParallelEncoder;   // ��̨ƽ����Yaw����ʱPitch��������ֵ
  float DepressionEncoderInDegree; // ��̨�������ʱPitch�������̶�Ӧ�Ƕ�ֵ
  float ElevationEncoderInDegree;  // ��̨�������ʱPitch�������̶�Ӧ�Ƕ�ֵ
  float DepressionIMU;             // ��̨�������ʱ��̬��
  float ElevationIMU;              // ��̨�������ʱ��̬��
} Gimbal_t;

typedef struct _GimbalDate
{
  uint8_t MiniPC_state; // 1 online
  uint8_t tgt_ID;       // 0û��Ŀ��
  uint8_t Gimbal_mode;  // 0 Normal_Mode 1 AimAssist_Mode
  uint8_t shooter_id;
  uint8_t shooter_state; // 1 fixed_shooter1 2 fixed_shooter2 0 unfixed
  uint8_t isFricOn;
  uint8_t isLidOpen;
  uint8_t myteam;
  int16_t x;
  int16_t pre_x;
  uint16_t y;
  uint16_t pre_y;
  int16_t z;
  int16_t pre_z;
} Gimbal_Date_t;

enum ColorChannels
{
  INVALID_COLOR = 0,
  BLUE = 1,
  RED = 2
};

extern Gimbal_t Gimbal;
extern Gimbal_Date_t Gimbal_Date;

void Gimbal_Init(void);
void Callback_Gimbal_Handle(Gimbal_Date_t *Gimbal_Date, uint8_t *buf);
void Callback_Aimassist_Handle(Gimbal_Date_t *Gimbal_Date, uint8_t *buf);
#endif
