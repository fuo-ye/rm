#ifndef _POWER_MEASUER_H
#define _POWER_MEASUER_H

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "stdint.h"

//ʹ������IIC(ʹ��Ӳ��IICʱע�͸��д���)
#define Use_software_i2c

//ʹ��I2C1ʱ���޸�Ϊhi2c1
#define Use_hi2c3
#define ina226_i2c hi2c3

#define Shunt_ohm 10 //��λ��ŷ
#define Curren_LSB 1 //��λmA�������Ĵ�����������15λ��2^15

typedef struct
{
    uint16_t Config_Register;
    int16_t Shunt_Voltage_Register; //01h
    int16_t Bus_Voltage;
    uint16_t Power_Register;
    int16_t Current_Register;
    uint16_t Calibration_Register; // (05h)
    uint16_t ID;
} ina226_t_reg_t;

typedef struct
{
    float Bus_Voltage;   //1.25mv/msb
    float Shunt_Voltage; //2.5uv/msb
    float Power_cal_W;   //���һ����ĸ�ǵ�λ
    float Power_read_W;
    float current_mA;
    float current_A;
    float powerchassis;
    float state;
    float buck_duty;
    float Bus_Voltage_filter;//�˲������ߵ�ѹ
    float Power_cal_W_first_order_filter;//һ���˲�����
    float Power_cal_W_window_filter;//�����˲�����

} ina226_t;

#define CFG_REG 0x00 //

#define SV_REG 0x01 //������ѹ�� �˴���������Ϊ 0.1ŷ

#define BV_REG 0x02 //���ߵ�ѹ

#define PWR_REG 0x03 //��Դ����

#define CUR_REG 0x04 //����

#define CAL_REG 0x05 //У׼���趨�����̷�Χ�Լ������͹��ʲ�����

#define ONFF_REG 0x06 //���� ʹ�� �������ú�ת��׼������

#define AL_REG 0x07 //��������ѡ����������Ƚϵ��޶�ֵ

#define INA226_GET_ADDR 0XFF //����Ψһ��оƬ��ʶ��


#define INA226_ADDR1 0x80
#define INA226_ADDR2 0x81
#define INA226_ADDR3 0x88

extern ina226_t ina226[3];

void INA226_Init(uint16_t Ina226_ID);
uint16_t INA226_Read_a_Reg(uint8_t reg, uint16_t Ina226_id);
void INA226_Read_Registers(uint16_t ina226_id);
#endif
