
#include "gimbal_task.h"
#include "remote_control.h"
#include "includes.h"
#include "math.h"

Gimbal_t Gimbal = {0};
Gimbal_Date_t Gimbal_Date ={0};
void Gimbal_Init(void)
{
   Gimbal.YawMotor.zero_offset = YAW_MOTOR_ZERO_OFFSET;
	Gimbal.PitchMotor.zero_offset = PITCH_MOTOR_ZERO_OFFSET;
#if INFANTRY_ID == 1

#elif INFANTRY_ID == 2 
	Gimbal.YawMotor.Direction = NEGATIVE;	
	//    Gimbal.PitchMotor.Direction = NEGATIVE;
#elif INFANTRY_ID == 3
       
//    Gimbal.PitchMotor.Direction = NEGATIVE;
				Gimbal.YawMotor.Direction = NEGATIVE;
#elif INFANTRY_ID == 4
           //    Gimbal.PitchMotor.Direction = NEGATIVE;
				Gimbal.YawMotor.Direction = NEGATIVE; 
#endif
    
	 if (robot_state.robot_id <= 7)
            Gimbal_Date.myteam = RED;
   	 else
            Gimbal_Date.myteam = BLUE;
}

void Callback_Gimbal_Handle(Gimbal_Date_t *Gimbal_Date ,uint8_t * buff)
{
		uint8_t buff_temp[3];
		 if (Gimbal_Date == NULL || buff == NULL)
    {
        return;
    }
		Gimbal_Date->tgt_ID = buff[0];
		Gimbal_Date->MiniPC_state = buff[1]&0x01;
		Gimbal_Date->isLidOpen = (buff[1]>>1)&0x01;
		Gimbal_Date->isFricOn = (buff[1]>>2)&0x01;
		Gimbal_Date->Gimbal_mode = buff[2];
		Gimbal_Date->shooter_id = buff[7]&0x03;
		Gimbal_Date->shooter_state  = (buff[7]>>2)&0x03;

}
