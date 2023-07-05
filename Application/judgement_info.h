#ifndef _JUDGEMENT_INFO_1__
#define _JUDGEMENT_INFO_1__

#include "includes.h"

#define JUDGE_SOF (uint8_t)0xA5
#define GAME_DATA 0x0201
#define ROBOT_STATE_DATA 0x0001
#define POWER_HEAT_DATA 0x0202
#define ROBOT_POS_DATA 0x0203
#define ROBOT_HURT_DATA 0x0206
#define USER_DATA 0x0301

extern uint8_t Shoot_Updata;

typedef enum
{
    GAME_STATUS_CMD_ID = 0x0001,
    GAME_RESULT_CMD_ID = 0x0002,
    GAME_ROBOT_HP_CMD_ID = 0x0003,
    DART_STATUS_CMD_ID = 0x0004,
    ICRA_BUFF_DEBUFF_ZONE_STATUS_CMD_ID = 0x0005,
    EVEN_DATA_CMD_ID = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID = 0x0102,
    REFEREE_WARNING_CMD_ID = 0x0104,
    DART_REMAINING_TIME_CMD_ID = 0X0105,
    GAME_ROBOT_STATUS_CMD_ID = 0x0201,
    POWER_HEAT_DATA_CMD_ID = 0x0202,
    GAME_ROBOT_POS_CMD_ID = 0x0203,
    BUFF_MUSK_CMD_ID = 0x0204,
    ROBOT_ENERGY_CMD_ID = 0x0205,
    ROBOT_HURT_CMD_ID = 0x0206,
    SHOOT_DATA_CMD_ID = 0x0207,
    BULLET_REMAINING_CMD_ID = 0x0208,
    RFID_STATUS_CMD_ID = 0x0209,
    DART_CLIENT_CMD_ID = 0x020A,
    GROUND_ROBOT_POSITION_CMD_ID = 0x020B,
    RADAR_MARK_DATA_CMD_ID = 0x020C,
    STUDENT_INTERACTIVE_HEADER_DATA_CMD_ID = 0x0301,
    INTERACTIVE_CMD_ID = 0x0302,
    MAP_INTERACTIVITY_CMD_ID = 0x0303,
    CUSTOM_CLIENT_CMD_ID = 0x0306,
    MAP_SENTRY_CMD_ID = 0x0307,
    IDCustomData,
} referee_cmd_id_t;

typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    RED_DART = 8,
    RED_RADAR = 9,
    RED_OUTPOST = 10,
    RED_BASE = 11,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
    BLUE_DART = 108,
    BLUE_RADAR = 109,
    BLUE_OUTPOST = 110,
    BLUE_BASE = 111,
} robot_id_t;

typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;

#pragma pack(1)
typedef struct
{
    uint8_t buf[100];
    uint8_t header[5];
    uint16_t cmd;
    uint8_t data[32];
    uint8_t tail[2];
} judge_receive_t;

// 比赛状态数据0x0001
typedef struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

// 比赛结果数据0x0002
typedef struct
{
    uint8_t winner;
} ext_game_result_t;

// 比赛机器人血量数据0x0003
typedef struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

// 飞镖发射状态0x0004
typedef struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;

// 人工智能挑战赛加成与惩罚状态0x0005
typedef struct
{
    uint8_t F1_zone_status : 1;
    uint8_t F1_zone_buff_debuff_status : 3;
    uint8_t F2_zone_status : 1;
    uint8_t F2_zone_buff_debuff_status : 3;
    uint8_t F3_zone_status : 1;
    uint8_t F3_zone_buff_debuff_status : 3;
    uint8_t F4_zone_status : 1;
    uint8_t F4_zone_buff_debuff_status : 3;
    uint8_t F5_zone_status : 1;
    uint8_t F5_zone_buff_debuff_status : 3;
    uint8_t F6_zone_status : 1;
    uint8_t F6_zone_buff_debuff_status : 3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

// 场地事件数据0x0101
typedef struct
{
    uint32_t event_type;
} ext_event_data_t;

// 场地补给站动作标识数据0x0102
typedef struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

// 尚未开放
//  请求补给站补弹数据
//  typedef __packed struct //0x0103
//  {
//      uint8_t supply_projectile_id;
//      uint8_t supply_robot_id;
//      uint8_t supply_num;
//  } ext_supply_projectile_booking_t;

// 裁判警告数据0x0104
typedef struct
{
    uint8_t level;
    uint8_t offending_robot_id;
} ext_referee_warning_t;

// 飞镖发射口倒计时0x0105
typedef struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

// 机器人状态数据0X201
typedef struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;

    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

// 实时功率热量数据0x0202
typedef struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

// 机器人位置数据0x0203
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

// 机器人增益数据0x0204
typedef struct
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

// 空中机器人能量状态数据0x0205
typedef struct
{
    uint8_t attack_time;
} aerial_robot_energy_t;

// 伤害状态数据0x0206
typedef struct // 0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

// 实时射击数据0x0207
typedef struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

// 子弹剩余发送数，空中机器人以及哨兵机器人发送0x0208
typedef struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

// 机器人 RFID 状态0x0209
typedef struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

// 飞镖机器人客户端指令书0x020A
typedef struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;
// 己方机器人位置0x020B
typedef struct
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
} ground_robot_position_t;
// 对方机器人被标记进度0x020C
typedef struct
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
} radar_mark_data_t;
// 机器人间交互数据，发送方触发发送0x0301
typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

// 交互数据 机器人间通信
typedef struct
{
    uint8_t data[118];
} robot_interactive_data_t;

// 客户端删除图形
typedef struct
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;

// 自定义控制器交互数据接口，通过客户端触发发送0x0302
typedef struct
{
    uint8_t data[30];
} ext_robot_interactive_data_t;

// 客户端小地图交互数据0x0303
typedef struct
{
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;
    uint16_t target_robot_ID;
} ext_map_interactivity_t;
// 操作手可使用自定义控制器模拟键鼠操作选手端0x0306
typedef struct
{
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} custom_client_data_t;

// 哨兵机器人可向己方空中机器人选手端发送路径坐标数据，该路径会在其小地图上显示0x0307
typedef struct
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
} map_sentry_data_t;

#pragma pack()
extern ext_game_status_t game_status;
extern ext_game_result_t game_result;
extern ext_game_robot_HP_t game_robot_HP;
extern ext_dart_status_t dart_status;
extern ext_ICRA_buff_debuff_zone_status_t ICRA_buff_debuff_zone_status;
extern ext_event_data_t event_data;
extern ext_supply_projectile_action_t supply_projectile_action;
extern ext_referee_warning_t referee_warning;
extern ext_dart_remaining_time_t dart_remaining_time;
extern ext_game_robot_status_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern ext_game_robot_pos_t game_robot_pos;
extern ext_buff_musk_t buff_musk;
extern aerial_robot_energy_t robot_energy;
extern ext_robot_hurt_t robot_hurt;
extern ext_shoot_data_t shoot_data;
extern ext_rfid_status_t rfid_status;
extern ext_bullet_remaining_t bullet_remaining;
extern ext_dart_client_cmd_t dart_client_cmd;
extern ground_robot_position_t ground_robot_position;
extern radar_mark_data_t radar_mark_data;
extern ext_student_interactive_header_data_t student_interactive_header_data;
extern ext_robot_interactive_data_t robot_interactive_data;
extern ext_map_interactivity_t map_interactivity;
extern custom_client_data_t custom_client_data;
extern map_sentry_data_t map_sentry_data;

extern UART_HandleTypeDef *JudgeUSART;
extern uint8_t Shoot_Updata;

extern HAL_StatusTypeDef IT_DMA_Begain(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern judge_receive_t judgement_receive;
void Judge_Control_Init(UART_HandleTypeDef *huart);
void judgement_info_handle(void);
void unpack_fifo_handle(uint8_t *prxbuf);
void judgement_info_updata(void);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void judgement_data_decode(void);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

extern unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);

#endif
