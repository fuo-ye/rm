/**
 ******************************************************************************
 * @file    chassis task.c
 * @author  Wang Hongxi
 * @version V1.0.2
 * @date    2020/2/4
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "chassis_task.h"
uint8_t chassis_debug_mode = 1;
uint32_t RC_Lost = 0;
Chassis_t Chassis = {0};
static uint8_t data[6];
uint32_t Chassis_DWT_Count = 0;
uint32_t tempTime_Sprint = 0;
uint32_t time_temp = 0;
float Vxy_Coef = 1.0f;
float V_coef = 1.0f;
float CAP_flag;

float  ChassisMotorsPower;
float  ChassisMeasurePower;
float  ChassisPower;
float  ChassisPowerBuffer;
float PowerLimit;
int Speedup_signal;
float warning_power_buff=30.0f;
float Stamp;
int32_t Vx_ref = 0;
int32_t Vy_ref = 0;
int32_t last_Vx;
int32_t last_Vy;

float Amp1 = 0;
float Omega1 = 5;
float Amp2 = 0;
float Omega2 = 5;
uint8_t AMP = 50;
uint16_t VR = 600;
float MAX_RPM = 8000;

float VX_k = 900;//最大阶跃速度比
float VY_k = 900;
float VR_k = 50;
float Vr_MAX = 150;
float c[3]={1, 1, 1};

uint8_t count_press = 0;
uint8_t BIANSU_SET = 100;
float LPF_Coef = 0.05f;
static float dt = 0,
             t = 0;
float r = 1000, r_slip = 20000, r_slowstart = 35000;
float r_fast = 200000;
float h = 0.01;
float twist_speed = 400, twist_omega = 8;
float Gimbal_Position_Modification = 1.0f;
float predictvalue;

uint8_t Use_Differential = FALSE;
uint8_t Use_VelocityLimit =FALSE;
uint8_t Use_CurrentLimit = TRUE;
static void Chassis_Get_Theta(void);//获取底盘与云台偏角
static void Chassis_Set_Mode(void);//设置底盘运动模式
static void Chassis_Get_CtrlValue(void);//处理来自摇杆与键盘的控制数据
static void Chassis_Set_Control(void);//底盘运动解算以及PID计算
static void Send_Chassis_Current(void);//发送底盘电机控制电流
static float Max_4(float num1, float num2, float num3, float num4);
static void Send2CapBoard(void);/*发送裁判系统信息到电容板*/
static void abs_control(void);
static void ChassisState_Inverse_solution(void);/*计算底盘运动速度*/
static void Chassis_Attitude_Control(float *Attitude_adjustment);
static float Find_heading(void);//找头
static void Chassis_Power_Limit(void);
static void Chassis_Power_Exp(void);//计算可能的底盘功率
static void Velocity_MAXLimit(void);//最大速度限制
static void Change_Predict(void);//换头预测
void Chassis_Init(void)
{
    static float tempTheta = 360;
    Chassis.VelocityRatio = VELOCITY_RATIO;
#if INFANTRY_ID == 3|| INFANTRY_ID == 4
    Chassis.rcStickRotateRatio = 0;
    Chassis.rcMouseRotateRatio = 0;
#else
    Chassis.rcStickRotateRatio = RC_STICK_ROTATE_RATIO;
    Chassis.rcMouseRotateRatio = RC_MOUSE_ROTATE_RATIO;
#endif

    Chassis.HeadingFlag = 0;
    Chassis.Heading = 0;

    Chassis.GravityCenter_Adjustment = GRAVUTYCENTER_ADJUSTMENT;

    Chassis.PowerControl.Power_correction_gain = POWER_GAIN;
    Chassis.PowerControl.LowVoltage_Gain = LOWVOLTAGE_GAIN;

    TD_Init(&Chassis.ChassisVxTD, 1000, 0.01f);
    TD_Init(&Chassis.ChassisVyTD, 1000, 0.01f);


    for (int8_t i = 0; i < (360.0f / YAW_REDUCTION_CORRECTION_ANGLE); i++) //用于步兵启动时找头
    {
        Chassis.TotalTheta = (Gimbal.YawMotor.total_angle - Gimbal.YawMotor.zero_offset) *
                                 ENCODERCOEF * YAW_REDUCTION_RATIO +
                             i * YAW_REDUCTION_CORRECTION_ANGLE;
        Chassis.Theta = loop_float_constrain((Chassis.TotalTheta +
                                              Gimbal.YawMotor.offset_angle * ENCODERCOEF * YAW_REDUCTION_RATIO),
                                             -180, 180);
        if (fabsf(Chassis.Theta) < tempTheta)
        {
            tempTheta = fabsf(Chassis.Theta);
        }
    }

    Chassis.YawCorrectionScale = 0;

    //电机速度环PID计算
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Init(
            &Chassis.ChassisMotor[i].PID_Velocity, 16384, 16384, 0, 15, 30, 0, 500, 100, 0.005, 0, 1, Integral_Limit | OutputFilter);
        Chassis.ChassisMotor[i].Max_Out = 16384;
    }
    //底盘跟随云台PID初始化
    PID_Init(&Chassis.RotateFollow, 300, 100, 0, 6, 0, 0, 0,
             0, 0, 0, 5,
             Integral_Limit | Derivative_On_Measurement | OutputFilter | DerivativeFilter);

    //底盘Vr PID初始化
    PID_Init(&Chassis.ChassisVr, 500, 100, 0, 5, 0, 0, 300,
             100, 0, 0.005, 5,
             Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | DerivativeFilter);
}

void Chassis_Control(void)
{
    dt = DWT_GetDeltaT(&Chassis_DWT_Count);
    t += dt;
    //获取底盘与云台偏角
    Chassis_Get_Theta();
    //设置底盘运动模式
    Chassis_Set_Mode();
    //处理来自摇杆与键盘的控制数据
    Chassis_Get_CtrlValue();
    //底盘运动解算以及PID计算
    Chassis_Set_Control();
    //发送底盘电机控制电流
    Send_Chassis_Current();
    /*计算底盘运动速度*/
    ChassisState_Inverse_solution();
    /*发送裁判系统信息到电容板*/
    Send2CapBoard();

}

static void Chassis_Get_Theta(void)
{
    //底盘与云台偏角获取
    Chassis.TotalTheta = (Gimbal.YawMotor.total_angle - Gimbal.YawMotor.zero_offset) *
                             ENCODERCOEF * YAW_REDUCTION_RATIO +
                         Chassis.YawCorrectionScale * YAW_REDUCTION_CORRECTION_ANGLE;

    Chassis.Theta = loop_float_constrain((Chassis.TotalTheta +
                                          Gimbal.YawMotor.offset_angle * ENCODERCOEF * YAW_REDUCTION_RATIO),
                                         -180, 180);

    Find_heading();
    // 云台偏角异常时，重置底盘偏角
    if (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE))
    {
        Chassis.TotalTheta = 0;
        Chassis.Theta = 0;
    }
#ifdef ENABLE_SPINNING
    Chassis.FollowTheta = float_deadband(Chassis.Theta, -0.005f, 0.005f);

#else
    Chassis.FollowTheta = Chassis.TotalTheta;
#endif
}
static void Chassis_Get_CtrlValue(void)
{
    static float Temp_Vx, Temp_Vy;//速度的期望值
    static float tempVal;
    //static float RC = 0.000001f;
    static float RC = 0.00001f;
    //功率限制与实际值不符时
    if (robot_state.chassis_power_limit >= 10240)//数据包体积的最大值
        robot_state.chassis_power_limit /= 256;
    //功率限幅
    if (robot_state.chassis_power_limit > 120)//数据包的体积的最小值
        robot_state.chassis_power_limit = 120;
    //功率最低限制
    if (robot_state.chassis_power_limit == 0)
        robot_state.chassis_power_limit = 45;

    if (robot_state.chassis_power_limit != 0)
        Chassis.VelocityRatio = float_constrain(robot_state.chassis_power_limit / 10.0f + 2, 5, 10); //通过power_limit 调整机器人跑得快慢

    if (remote_control.key_code & Key_A || remote_control.key_code & Key_D)  //给到最大速度，然后梯度上升
    {
        if (remote_control.key_code & Key_D)
            Temp_Vx = 660.0f;
        if (remote_control.key_code & Key_A)
            Temp_Vx = -660.0f;
    }
    else
        Temp_Vx = 0;
    if (remote_control.key_code & Key_W || remote_control.key_code & Key_S) 
    {
        if (remote_control.key_code & Key_W)
            Temp_Vy = 660.0f;
        if (remote_control.key_code & Key_S)
            Temp_Vy = -660.0f;
    }
    else
        Temp_Vy = 0;
    
    // 获取遥控器控制量
    Temp_Vx += remote_control.ch3;
    Temp_Vy += remote_control.ch4;

    if (remote_control.switch_left == Switch_Up || remote_control.key_code & Key_SHIFT ||remote_control.key_code & Key_CTRL)
    {
        Temp_Vx *= 2;
        Temp_Vy *= 2;
    }
    if ((remote_control.key_code & Key_C))
    {
        Temp_Vx *= 0.2f;
        Temp_Vy *= 0.2f;
    }

    if (fabsf(Temp_Vx) > 1e-3f)//上升部分 1e-3f ~~ 0
    {
        if (Chassis.Vx * Temp_Vx < 0)
        {
            Chassis.Vx = 0;
        }
        tempVal = (Temp_Vx - Chassis.Vx) / (RC + dt);
        if (tempVal > VX_k)
            tempVal = VX_k;
        else if (tempVal < -VX_k)
            tempVal = -VX_k;
        Chassis.Vx += tempVal * dt;
    }
    else
    {
        tempVal = (Temp_Vx - Chassis.Vx) / (0.01f + dt);
        Chassis.Vx += tempVal * dt;
    }

    if (fabsf(Temp_Vy) > 1e-3f)
    {
        if (Chassis.Vy * Temp_Vy < 0)
            Chassis.Vy = 0;
        tempVal = (Temp_Vy - Chassis.Vy) / (RC + dt);
        if (tempVal > VY_k)
            tempVal = VY_k;
        else if (tempVal < -VY_k)
            tempVal = -VY_k;
        Chassis.Vy += tempVal * dt;
    }
    else
    {
        tempVal = (Temp_Vy - Chassis.Vy) / (0.01f + dt);
        Chassis.Vy += tempVal * dt;
    }
}

static void Chassis_Set_Mode(void)
{
    static uint16_t LastKeyCode = 0;
    static uint32_t tempTimeStamp = 0;
    static uint32_t tempTimeStamp2 = 0;

    //按住EF,静默模式
    if ((remote_control.key_code & Key_F) && (remote_control.key_code & Key_E))
    {
        Chassis.Mode = Silence_Mode;
        return;
    }

    // F切换陀螺模式和跟随模式
    if ((remote_control.key_code & Key_F) && !(LastKeyCode & Key_F))
    {
        if (Chassis.Mode != Spinning_Mode)
            Chassis.Mode = Spinning_Mode;
        else
            Chassis.Mode = Follow_Mode;
    }

    // Z切换静默模式和跟随模式
    if ((remote_control.key_code & Key_Z) && !(LastKeyCode & Key_Z))
    {
        if (Chassis.Mode != Silence_Mode)
            Chassis.Mode = Silence_Mode;
        else
            Chassis.Mode = Follow_Mode;
    }

    // E切换斜45度和跟随模式
    if ((remote_control.key_code & Key_E) && !(LastKeyCode & Key_E))
    {

        if (Chassis.Mode != Side_Mode)
            Chassis.Mode = Side_Mode;
        else
            Chassis.Mode = Follow_Mode;
    }
    if (Chassis.Mode == Side_Mode)
    {
        Chassis.Heading = 45.0f; 
    }
    else
    {
        Chassis.Heading = 0.0f;
    }

	if((is_TOE_Error(CHASSIS_MOTOR1_TOE)&&is_TOE_Error(CHASSIS_MOTOR2_TOE)&&is_TOE_Error(CHASSIS_MOTOR3_TOE)&&is_TOE_Error(CHASSIS_MOTOR4_TOE))
			||((remote_control.key_code & Key_F) && (remote_control.key_code & Key_E)))
	{
		Chassis.Mode = Silence_Mode;
	}

    //拨杆切换运动模式
    switch (remote_control.switch_right)
    {
    case Switch_Up: // 
        Chassis.Mode = Follow_Mode;
        break;

    case Switch_Down: // All from Keyboard?
        Chassis.Mode = Silence_Mode;
        //Chassis.Mode = Spinning_Mode;
        break;
    }
    LastKeyCode = remote_control.key_code;
}

static void Chassis_Set_Control(void)
{

    switch (Chassis.Mode)
    {
    case Follow_Mode:
    FOLLOW:
        if (fabsf(Chassis.FollowTheta) < 0.005f)//判断是否正对头部
        {
#ifdef Chassis_Use_IMU

#else
            Chassis.Vr = remote_control.ch1 * Chassis.rcStickRotateRatio + remote_control.mouse.x * Chassis.rcMouseRotateRatio;
        
#endif
        }
        else
        {
#ifdef Chassis_Use_IMU

#else
            if (remote_control.switch_left == Switch_Up || remote_control.key_code & Key_SHIFT||remote_control.key_code & Key_CTRL)//开启电容
                Chassis.Vr = -PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, Chassis.Heading);
            else
            {
                Chassis.Vr = -PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, Chassis.Heading) +
                             remote_control.ch1 * Chassis.rcStickRotateRatio + remote_control.mouse.x * Chassis.rcMouseRotateRatio;
                //Change_Predict( );
            }
#endif
        }
        Chassis.VxTransfer = user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx +
                             user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy;
        Chassis.VyTransfer = -user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx +
                             user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy;

        if (Chassis.Mode == Spinning_Mode || Chassis.Mode == Side_Mode)
        {
            Chassis.Vr *= 2;
            tempTime_Sprint = USER_GetTick();
        }
        break;

    case Silence_Mode:
        Chassis.Vr = 0;
        Chassis.VxTransfer = 0;
        Chassis.VyTransfer = 0;

        //静默模式中按下移动键，进入跟随模式
        if ((remote_control.key_code & Key_W) || (remote_control.key_code & Key_A) || (remote_control.key_code & Key_S) || (remote_control.key_code & Key_D))
            Chassis.Mode = Follow_Mode;
        break;

    case Spinning_Mode:
    {
        if (remote_control.key_code & Key_SHIFT||remote_control.key_code & Key_CTRL)
            Chassis.Vr = 475;
        else
        {
//            Chassis.Vr = 0.8f*float_constrain(45 * Chassis.VelocityRatio, -475, 475); // 475
						Chassis.Vr = 190; // 150
        }
        Chassis.DeflectionAngle = (Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF;
        Chassis.VxTransfer = user_cos(Chassis.DeflectionAngle) * Chassis.Vx +
                             user_sin(Chassis.DeflectionAngle) * Chassis.Vy;
        Chassis.VyTransfer = -user_sin(Chassis.DeflectionAngle) * Chassis.Vx +
                             user_cos(Chassis.DeflectionAngle) * Chassis.Vy;

        Chassis.VxTransfer *= 0.5f;
        Chassis.VyTransfer *= 0.5f;
        break;
    }

    case Side_Mode:
    {
#ifdef Chassis_Use_IMU

#else
        Chassis.Vr = -PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, Chassis.Heading) +
                     remote_control.ch1 * Chassis.rcStickRotateRatio + remote_control.mouse.x * Chassis.rcMouseRotateRatio;
#endif
    }
        Chassis.VxTransfer = user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx + user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy;
        Chassis.VyTransfer = -user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx + user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy;
        break;
    }

    if (GlobalDebugMode != 0 && GlobalDebugMode != CHASSIS_DEBUG)
    {
        Chassis.VxTransfer = 0;
        Chassis.VyTransfer = 0;
        Chassis.Vx = 0;
        Chassis.Vy = 0;
        Chassis.Vr = 0;
    }

    if (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE)) //如果YAW轴电机掉线（云台无法旋转），只有底盘旋转带动枪管旋转
    {
#ifdef Chassis_Use_IMU

#else
        Chassis.Vr = remote_control.ch1 * Chassis.rcStickRotateRatio + remote_control.mouse.x * Chassis.rcMouseRotateRatio;
#endif
    }
    
#ifdef UseAttitudeControl
    if (Use_Differential || (remote_control.key_code & Key_CTRL))
    {
        Chassis_Attitude_Control(Chassis.Attitude_adjustment);
    }
    else
    {
        //memset(Chassis.Attitude_adjustment,1, 4 * sizeof(float));
        Chassis.Attitude_adjustment[0] = 1;
        Chassis.Attitude_adjustment[1] = 1;
        Chassis.Attitude_adjustment[2] = 1;
        Chassis.Attitude_adjustment[3] = 1;
    }

#else
    memset(Chassis.Attitude_adjustment, 1, 4 * sizeof(float));
#endif
    Chassis.V1 = -(-Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[0] + Gimbal_Position_Modification * Chassis.Vr * 10;
    Chassis.V2 = -(Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[1] + Gimbal_Position_Modification * Chassis.Vr * 10;
    Chassis.V3 = -(Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[2] + Chassis.Vr * 10;
    Chassis.V4 = -(-Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[3] + Chassis.Vr * 10;

    //云台重心不一样的修正
    Chassis.V3 *= Chassis.GravityCenter_Adjustment;
    Chassis.V4 *= Chassis.GravityCenter_Adjustment;

	Velocity_MAXLimit();

    Motor_Speed_Calculate(&Chassis.ChassisMotor[0], Chassis.ChassisMotor[0].Velocity_RPM, Chassis.V1);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[1], Chassis.ChassisMotor[1].Velocity_RPM, Chassis.V2);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[2], Chassis.ChassisMotor[2].Velocity_RPM, Chassis.V3);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[3], Chassis.ChassisMotor[3].Velocity_RPM, Chassis.V4);

    if (Chassis.Vx != 0 || Chassis.Vy != 0)
        time_temp = USER_GetTick();

   Chassis_Power_Exp();//计算可能的底盘功率
    Chassis_Power_Limit();

    //功率限制
    // if (remote_control.switch_left == Switch_Up || remote_control.key_code & Key_SHIFT ||remote_control.key_code & Key_CTRL)
    // {
    //         Chassis_Power_Limit();
    // }
    
}

static void Send_Chassis_Current(void)/*发送底盘电机控制电流*/
{
    //发送裁判系统功率部分给电容板
    Send_Power_Data(&hcan1, power_heat_data_t.chassis_power_buffer, robot_state.chassis_power_limit);

    if (is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE))//RC——遥控器接收器  VTM——图传
    {
        if (Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0) == HAL_OK)
            HAL_IWDG_Refresh(&hiwdg);
        ;
    }
    else
    {
        if (Send_Motor_Current_1_4(&hcan1,
                                   Chassis.ChassisMotor[0].Output, Chassis.ChassisMotor[1].Output,
                                   Chassis.ChassisMotor[2].Output, Chassis.ChassisMotor[3].Output) == HAL_OK)
            HAL_IWDG_Refresh(&hiwdg);
        ;
    }
    //计算实际底盘功率
    Chassis.PowerControl.Power_Calculation_Clipping = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        Chassis.PowerControl.Power_Calculation_Clipping += ina226[0].Bus_Voltage * fabsf(Chassis.ChassisMotor[i].Real_Current * Chassis.ChassisMotor[i].Velocity_RPM);
    }
    Chassis.PowerControl.Power_Calculation_Clipping *= 0.0000001732f;
}
static void Send2CapBoard(void)/*发送裁判系统信息到电容板*/
{
    static uint8_t count = 0;
    uint16_t a, b;
    a = robot_state.chassis_power_limit;
    b = power_heat_data_t.chassis_power_buffer;
    data[0] = 0XAA;
    data[5] = 0XBB;
    memcpy(data + 1, &a, 2);
    memcpy(data + 3, &b, 2);
    if (count % 3 == 0)
    {
         if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX && (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == SET))
            {
                HAL_UART_Transmit_DMA(&huart1, data, 6);
            }
        
        count = 0;
    }
    count++;
   
}

static void ChassisState_Inverse_solution(void)/*计算底盘运动速度*/
{
    static uint8_t tempid[4];
    static float LastVx_Chassis = 0;
    static float LastVy_Chassis = 0;
    static float LastVx_Body = 0;
    static float LastVy_Body = 0;
    
    static int8_t chassishead;
    
    chassishead = Chassis.HeadingFlag==-1?3:Chassis.HeadingFlag;
        tempid[0] = loop_int_constrain(chassishead,0,3);
        tempid[1] = loop_int_constrain(chassishead+1,0,3);
        tempid[2] = loop_int_constrain(chassishead+2,0,3);
        tempid[3] = loop_int_constrain(chassishead+3,0,3);
    Chassis.V1_is = ((Chassis.ChassisMotor[loop_int_constrain(chassishead,0,3)].Velocity_RPM) * 2 * pi / 60.0f) * wheel_radius / 13.733f * 0.001f; //m/s
    Chassis.V2_is = ((Chassis.ChassisMotor[loop_int_constrain(chassishead+1,0,3)].Velocity_RPM) * 2 * pi / 60.0f) * wheel_radius / 13.733f * 0.001f;
    Chassis.V3_is = ((Chassis.ChassisMotor[loop_int_constrain(chassishead+2,0,3)].Velocity_RPM) * 2 * pi / 60.0f) * wheel_radius / 13.733f * 0.001f;
    Chassis.V4_is = ((Chassis.ChassisMotor[loop_int_constrain(chassishead+3,0,3)].Velocity_RPM) * 2 * pi / 60.0f) * wheel_radius / 13.733f * 0.001f;

    Chassis.Vy_is_Chassis = (Chassis.V1_is + Chassis.V2_is - Chassis.V3_is - Chassis.V4_is) * Kx * 1.414;
    Chassis.Vx_is_Chassis = (Chassis.V2_is + Chassis.V3_is - Chassis.V1_is - Chassis.V4_is) * Ky * 1.414;

    Chassis.Vy_is_Body = user_cos(Chassis.DeflectionAngle)* Chassis.Vy_is_Chassis 
                         + user_sin(Chassis.DeflectionAngle)*Chassis.Vx_is_Chassis;
    Chassis.Vx_is_Body = user_cos(Chassis.DeflectionAngle)* Chassis.Vx_is_Chassis
                         - user_sin(Chassis.DeflectionAngle)*Chassis.Vy_is_Chassis;

    Chassis.Ax_Chassis = (Chassis.Vx_is_Chassis - LastVx_Chassis) / dt;
    Chassis.Ay_Chassis = (Chassis.Vy_is_Chassis - LastVy_Chassis) / dt;

    Chassis.Ax_Body = (Chassis.Vx_is_Body - LastVx_Body) / dt;
    Chassis.Ay_Body = (Chassis.Vy_is_Body - LastVy_Body) / dt;
    
    Chassis.x += Chassis.Vx_is_Chassis * dt;
    Chassis.y += Chassis.Vy_is_Chassis * dt;
    LastVx_Chassis = Chassis.Vx_is_Chassis;
    LastVy_Chassis = Chassis.Vy_is_Chassis;
    LastVx_Body = Chassis.Vx_is_Body;
    LastVy_Body = Chassis.Vy_is_Body;
}

static float Max_4(float num1, float num2, float num3, float num4)
{
    static float max_num = 0;

    max_num = fabsf(num1);
    if (fabsf(num2) > max_num)
        max_num = fabsf(num2);
    if (fabsf(num3) > max_num)
        max_num = fabsf(num3);
    if (fabsf(num4) > max_num)
        max_num = fabsf(num4);

    return max_num;
}
static float Find_heading(void)
{
    float angle = 45;
    /*寻找此刻朝向*/
#ifdef FourHeading
    if (Chassis.Heading < 0.01f)
    {
        if ((Chassis.Mode != Silence_Mode) && ((Chassis.Theta > 45 + Chassis.Heading) && (Chassis.Theta < 135 + Chassis.Heading)))
        {
            Chassis.HeadingFlag = -1;

            Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
                                                 -180, 180);
        }
        else if (((Chassis.Mode != Silence_Mode) && (fabsf(Chassis.Theta) > 135) && (Chassis.Heading == 0)) || ((Chassis.Mode != Silence_Mode) && (Chassis.Heading != 0) && (Chassis.Theta < -135 + Chassis.Heading)))
        {

            Chassis.HeadingFlag = 2;

            Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
                                                 -180, 180);
        }
        else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta > -135 + Chassis.Heading) && (Chassis.Theta < -45 + Chassis.Heading))
        {

            Chassis.HeadingFlag = 1;

            Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
                                                 -180, 180);
        }
        else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta < 45 + Chassis.Heading) && (Chassis.Theta > -45 + Chassis.Heading))
        {
            Chassis.HeadingFlag = 0;

            Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
                                                 -180, 180);
        }
    }
    else
    {
        if ((Chassis.Mode != Silence_Mode) && ((Chassis.Theta > 45 + Chassis.Heading) && (Chassis.Theta < 135 + Chassis.Heading)))
        {
            Chassis.HeadingFlag = -1;

            Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
                                                 -180, 180);
        }
        else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta < -135 + Chassis.Heading))
        {

            Chassis.HeadingFlag = 2;

            Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
                                                 -180, 180);
        }
        else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta > -135 + Chassis.Heading) && (Chassis.Theta < -45 + Chassis.Heading))
        {

            Chassis.HeadingFlag = 1;

            Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
                                                 -180, 180);
        }
        else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta < 45 + Chassis.Heading) && (Chassis.Theta > -45 + Chassis.Heading))
        {
            Chassis.HeadingFlag = 0;

            Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
                                                 -180, 180);
        }
    }

#endif

#ifdef TwoHeading
    if ((Chassis.Mode != Silence_Mode) && fabsf(Chassis.Theta) > 135)
    {
        Chassis.HeadingFlag ^= 0x10;
        Chassis.YawCorrectionScale++;
        Chassis.TotalTheta = (Gimbal.YawMotor.total_angle - Gimbal.YawMotor.zero_offset) *
                                 ENCODERCOEF * YAW_REDUCTION_RATIO +
                             Chassis.YawCorrectionScale * YAW_REDUCTION_CORRECTION_ANGLE;

        Chassis.Theta = loop_float_constrain((Chassis.TotalTheta +
                                              Gimbal.YawMotor.offset_angle * ENCODERCOEF * YAW_REDUCTION_RATIO),
                                             -180, 180);
    }

#endif
}
static void Chassis_Attitude_Control(float *Attitude_adjustment) //差速飞坡，飞坡时后轮速度更快
{
    switch (Chassis.HeadingFlag)
    {
    case 0:
			/*pitch +*//*高导轮*/
        Attitude_adjustment[0] = 0.6 + Chassis.Differential_Gain[0];
        Attitude_adjustment[1] = 1.0 + Chassis.Differential_Gain[1];
        Attitude_adjustment[2] = 1.0 + Chassis.Differential_Gain[1];
        Attitude_adjustment[3] = 0.6 + Chassis.Differential_Gain[0];
        break;
    case -1:
			/*Row +*//*低导轮*/
        Attitude_adjustment[0] = 1.0 + Chassis.Differential_Gain[1];
        Attitude_adjustment[1] = 1.0 + Chassis.Differential_Gain[1];
        Attitude_adjustment[2] = 0.6 + Chassis.Differential_Gain[0];
        Attitude_adjustment[3] = 0.6 + Chassis.Differential_Gain[0];
        break;
    case 1:
			/*Row -*//*低导轮*/
        Attitude_adjustment[0] = 0.6 + Chassis.Differential_Gain[0];
        Attitude_adjustment[1] = 0.6 + Chassis.Differential_Gain[0];
        Attitude_adjustment[2] = 1.0 + Chassis.Differential_Gain[1];
        Attitude_adjustment[3] = 1.0 + Chassis.Differential_Gain[1];
        break;

    case 2:
			/*pitch -*//*高导轮*/
        Attitude_adjustment[0] = 1.2 + Chassis.Differential_Gain[1];
        Attitude_adjustment[1] = 0.8 + Chassis.Differential_Gain[0];
        Attitude_adjustment[2] = 0.8 + Chassis.Differential_Gain[0];
        Attitude_adjustment[3] = 1.2 + Chassis.Differential_Gain[1];
        break;
    }
}
void Chassis_Power_Limit(void)
{
    
    static float coef[4] = {1, 1,1,1};

//   //电容电压小于14.5V时
//   if (ina226[0].Bus_Voltage < 14.5f)
//   {
//     Chassis.PowerControl.Power_Limit =Chassis.PowerControl.LowVoltage_Gain * robot_state.chassis_power_limit;
//   }
//   else
//   {
//     //计算修正后的功率限制
//     Chassis.PowerControl.Power_Limit = Chassis.PowerControl.Power_correction_gain * robot_state.chassis_power_limit;

//     //缓冲能量充足时，可放宽限制
//     if (power_heat_data_t.chassis_power_buffer > 20)
//       Chassis.PowerControl.Power_Limit = 1.5f * Chassis.PowerControl.Power_correction_gain * robot_state.chassis_power_limit;
//   }
    ChassisMotorsPower = Chassis.PowerControl.Power_Calculation;
    ChassisMeasurePower = ina226[0].powerchassis;
    ChassisPower = power_heat_data_t.chassis_power;
    ChassisPowerBuffer = power_heat_data_t.chassis_power_buffer;
    if (remote_control.switch_left == Switch_Up || remote_control.key_code & Key_SHIFT ||remote_control.key_code & Key_CTRL)
    {
        Speedup_signal=1;
    }
    else
    Speedup_signal=0;
    Chassis.PowerControl.Power_Limit= Speedup_signal ? (robot_state.chassis_power_limit + 7.0f * 24.0f) : robot_state.chassis_power_limit;
    Chassis.PowerControl.Power_Limit*=0.936f;

  //底盘功率大于功率限制时
  if (ChassisPowerBuffer > warning_power_buff)
{

        Chassis.PowerControl.PowerScale = (ChassisMotorsPower > Chassis.PowerControl.Power_Limit) ? (Chassis.PowerControl.Power_Limit / ChassisMotorsPower) : 1.0f;
        Chassis.PowerControl.sec_PowerScale = (ChassisMeasurePower > Chassis.PowerControl.Power_Limit) ? (Chassis.PowerControl.Power_Limit / ChassisMeasurePower) : 1.0f;
        Chassis.PowerControl.PowerScale = Chassis.PowerControl.PowerScale *  Chassis.PowerControl.sec_PowerScale;
}
else
{
    //地盘功率小于功率限制
    
    Chassis.PowerControl.PowerScale = (ChassisPowerBuffer > 10.0f) ? (ChassisPowerBuffer / warning_power_buff) : (5.0f / warning_power_buff);
    Chassis.PowerControl.PowerScale *= 0.65f;
}
  //急停时无功率限制
  if ((USER_GetTick() - time_temp < 500 && USER_GetTick() - time_temp > 5) || (USER_GetTick() - tempTime_Sprint < 500 && USER_GetTick() - tempTime_Sprint > 5))
    Chassis.PowerControl.PowerScale = 1.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        Chassis.ChassisMotor[i].Output *= Chassis.PowerControl.PowerScale*coef[i];
    }

 
}
 //计算可能底盘功率
static void Chassis_Power_Exp(void)
{

    if (is_TOE_Error(CAP_TOE))//CAP是电容
    {
        ina226[0].Bus_Voltage = 24.0f; //电容掉线，默认为24v;
    }
    Chassis.PowerControl.Power_Calculation = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        Chassis.PowerControl.Power_Calculation += ina226[0].Bus_Voltage * fabsf(Chassis.ChassisMotor[i].Output * Chassis.ChassisMotor[i].Velocity_RPM);
    }
    Chassis.PowerControl.Power_Calculation*= 0.0000001732f;
}

static void Velocity_MAXLimit(void)
{
	  static float temp_max;
   if (Max_4(fabsf(Chassis.V1), fabsf(Chassis.V2), fabsf(Chassis.V3), fabsf(Chassis.V4)) > MAX_RPM)
    {
        temp_max = Max_4(fabsf(Chassis.V1), fabsf(Chassis.V2), fabsf(Chassis.V3), fabsf(Chassis.V4));
        Chassis.V1 *= MAX_RPM / temp_max;
        Chassis.V2 *= MAX_RPM / temp_max;
        Chassis.V3 *= MAX_RPM / temp_max;
        Chassis.V4 *= MAX_RPM / temp_max;
    }
}
static void abs_control(void)
{
}

// void Change_Predict(void)
// {
//     static float tempval;
//     static RC = 0.00001;

//     Feedforward_t HeadChange;
//     predictvalue = 0;
//     c[0] = Chassis_Vr_C0;
//     c[1] = Chassis_Vr_C1;
//     c[2] = Chassis_Vr_C2;
//     Feedforward_Init(&HeadChange, Chassis_Vr_FFC_MAXOUT, c, Chassis_Vr_FCC_LPF, 5, 5);
//     predictvalue = fabs(Feedforward_Calculate(&HeadChange, Chassis.Vr));

//     if(predictvalue > Change_Boundary)
//     {
//         if(Chassis.Vr > Vr_MAX)
//           Chassis.Vr = Vr_MAX;
//         if(Chassis.Vr < -Vr_MAX)
//           Chassis.Vr = -Vr_MAX;
//     }
//     if (Chassis.LastVr * Chassis.Vr < 0)
//             Chassis.Vr = 0;
//         tempval = (Chassis.LastVr - Chassis.Vr) / (RC + dt);
//         if (tempval > VR_k)
//             tempval = VR_k;
//         else if (tempval < -VR_k)
//             tempval = -VR_k;
//         Chassis.Vr += tempval * dt;
    
//     Chassis.LastVr = Chassis.Vr;

//  }
