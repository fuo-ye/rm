///**
//  ******************************************************************************
//  * @file    chassis power control.c
//  * @author  Liu Zhijie
//  * @version V1.0.0
//  * @date    2021/1/18
//  * @brief   
//  ******************************************************************************
//  * @attention 
//  *
//  ******************************************************************************
//  */
//#include "chassis_power_control.h"
//#include "judgement_info.h"
//#include "arm_math.h"
//#include "power_measure.h"
//#include "detect_task.h"
//#include "includes.h"

//uint8_t useCurrentLimit = 1;
//uint8_t Spinning_usePower = 0;

//float PowerLimit = 0.0f;   //��������
//float ChassisPower = 0.0f; //���̹���
//float PowerScale;          //���ʱ���
//extern uint32_t time_temp;

//#if INFANTRY_ID == 1
//float Kp_Power = 1.13f; //��������ϵ��
//#endif

//#if INFANTRY_ID == 2
//float Kp_Power = 1.13f; //��������ϵ��
//#endif

//#if INFANTRY_ID == 3
//float Kp_Power = 1.13f; //��������ϵ��
//#endif

//extern float Power_Maybe;
//extern uint32_t tempTime_Sprint;

///**
//  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
//  * @retval         none
//  */
//void Chassis_Power_Limit(void)
//{
//  //����������ʵ��ֵ����ʱ
//  if (robot_state.chassis_power_limit >= 10240)
//    robot_state.chassis_power_limit /= 256;
//  //�����޷�
//  if (robot_state.chassis_power_limit > 120)
//    robot_state.chassis_power_limit = 120;
//  //�����������
//  if (robot_state.chassis_power_limit == 0)
//    robot_state.chassis_power_limit = 60;

//  //�������͵ĵ��̹���
//  ChassisPower = Power_Maybe; //���̹���

//  //���ݵ�ѹС��14.5Vʱ
//  if (ina226[0].Bus_Voltage < 14.5f)
//  {
//#if INFANTRY_ID == 1
//    PowerLimit = 0.7f * robot_state.chassis_power_limit;
//#endif

//#if INFANTRY_ID == 2
//    PowerLimit = 0.6f * robot_state.chassis_power_limit;
//#endif

//#if INFANTRY_ID == 3
//    PowerLimit = 0.7f * robot_state.chassis_power_limit;
//#endif
//  }
//  else
//  {
//    //����������Ĺ�������
//    PowerLimit = Kp_Power * robot_state.chassis_power_limit;

//    //������������ʱ���ɷſ�����
//    if (power_heat_data_t.chassis_power_buffer > 20)
//      PowerLimit = 1.5f * Kp_Power * robot_state.chassis_power_limit;
//  }

//  //���̹��ʴ��ڹ�������ʱ
//  if (ChassisPower > PowerLimit)
//    PowerScale = PowerLimit / ChassisPower;
//  else
//    PowerScale = 1.0f;

//  //��ͣʱ�޹�������
//  if ((USER_GetTick() - time_temp < 500 && USER_GetTick() - time_temp > 5) || (USER_GetTick() - tempTime_Sprint < 500 && USER_GetTick() - tempTime_Sprint > 5))
//    PowerScale = 1.0f;

//  if (useCurrentLimit)
//  {
//    //�ȱ�����С�������ӵ�output
//    for (uint8_t i = 0; i < 4; i++)
//    {
//      Chassis.ChassisMotor[i].Output *= PowerScale;
//    }
//  }
//}
//void VelocityRPM_limit()
//{
//  
//}
