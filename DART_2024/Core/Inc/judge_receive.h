#ifndef _JUDGERECEIVE_H_
#define _JUDGERECEIVE_H_

/** Include Header Files **/
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"

#define REFEREE_DMA_SIZE (130)//一帧最长128字节（见通信协议），留两个字节防炸
#define UART6_MAX_RECEIVE_BUFFER_LENGTH 300
#define REFEREE_HUART huart6//改成自己兵种配置的串口

/*
Version:2024.01.22
修改日期：2024.01.23
修改人：Zou
注意：
命令码0X0301的子命令码0X0110
存在一个graphic_data_struct_t是协议中未定义的，
所以该子命令码整体注释，以后版本更新后记得修改
*/

/*--------------CmdID(2-Byte)----------------*/
#define GAME_STATUS             0X0001          //比赛状态数据，1Hz 周期发送
#define GAME_RESULT             0X0002          //比赛结果数据，比赛结束后发送
#define ROBOT_HP                0X0003          //比赛机器人血量数据，1Hz周期发送
//0X0005        //人工智能挑战赛加成与惩罚状态，1Hz周期发送。暂时用不到，就没写
#define EVENT_DATA               0X0101          //场地事件数据，事件改变后发送
#define SUPPLY_PROJECTILE_ACTION 0X0102          //场地补给站动作标识数据，动作改变后发送
//0X0103        //请求补给站补弹数据，由参赛队发送，上限 10Hz。RM对抗赛尚未开放，就没写
#define REFEREE_WARNING         0X0104          //裁判警告数据，警告发生后发送 
#define DART_INFO               0X0105          //飞镖发射口倒计时，1Hz 周期发送 
#define ROBOT_STATUS            0X0201          //机器人状态数据，10Hz 周期发送
#define POWER_HEAT_DATA         0X0202          //实时功率热量数据，50Hz 周期发送
#define ROBOT_POS               0X0203          //机器人位置数据，1Hz 发送
#define BUFF                    0X0204          //机器人增益数据，3Hz
#define AIR_SUPPORT_DATA        0X0205          //空中支援时间数据，1Hz周期发送，只有空中机器人主控发送
#define HURT_DATA               0X0206          //伤害状态数据，伤害发生后发送
#define SHOOT_DATA              0X0207          //实时射击数据，子弹发射后发送
#define PROJECTILE_ALLOWANCE    0X0208          //子弹剩余发送数，英雄、步兵、哨兵、空中发送，10Hz周期发送
#define RFID_STATUS             0X0209          //机器人 RFID 状态，3Hz 周期发送
#define DART_CLIENT_CMD         0X020A          //飞镖机器人客户端指令数据，3Hz周期发送
#define GROUND_ROBOT_POS        0X020B          //地面机器人位置数据，发给哨兵，1Hz周期发送
#define RADAR_MARK_DATA         0X020C          //雷达标定进度数据，1Hz周期发送
#define SENTRY_INFO             0X020D          //哨兵自主决策信息同步，1Hz周期发送
#define RADAR_INFO              0X020E          //雷达自主决策信息同步，1Hz周期发送
#define ROBOT_INTERACTION_DATA  0X0301          //机器人间交互数据，发送方触发发送，上限10Hz
#define CUSTOM_ROBOT_DATA       0X0302          //自定义控制器交互数据接口，通过客户端触发发送，上限30Hz//
#define MAP_COMMAND             0X0303          //客户端小地图交互数据，触发发送//
#define REMOTE_CONTROL          0X0304          //键盘、鼠标信息，通过图传串口发送，30Hz
#define MAP_ROBOT_DATA          0X0305          //小地图接收雷达数据，10Hz
#define CUSTOM_CLIENT_DATA      0X0306          //自定义控制器和选手端交互数据，30Hz
#define MAP_DATA                0X0307          //小地图接收哨兵数据，1Hz
#define CUSTOM_INFO             0X0308          //小地图接收机器人数据，3Hz
/*--------------CmdID(2-Byte)----------------*/

/*--------------DataSize----------------*/
#define GAME_STATUS_DATA_SIZE           (11)//0X0001
#define GAME_RESULT_DATA_SIZE           (1)//0X0002
#define ROBOT_HP_DATA_SIZE              (32)//0X0003
#define EVENTDATA_DATA_SIZE             (4) //0X0101
#define SUPPLY_PROJECTILE_ACTION_DATA_SIZE (4) //0X0102
#define REFEREE_WARNING_DATA_SIZE       (3) //0X0104
#define DART_INFO_DATA_SIZE             (3) //0X0105
#define ROBOT_STATUS_DATA_SIZE          (13)//0X0201
#define POWER_HEAT_DATA_SIZE            (16)//0X0202
#define ROBOT_POS_DATA_SIZE             (12)//0X0203
#define BUFF_SIZE                       (6)//0X0204
#define AIR_SUPPORTDATA_DATA_SIZE       (1)//0X0205
#define ROBOT_HURT_DATA_SIZE            (1)//0X0206
#define SHOOTDATA_DATA_SIZE             (7)//0X0207
#define PROJECTILE_ALLOWANCE_DATA_SIZE  (6)//0X0208
#define RFID_STATUS_DATA_SIZE            (4)//0X0209
#define DART_CLIENT_CMD_DATA_SIZE       (6)//0X020A
#define GROUND_ROBOT_POS_DATA_SIZE      (40)// 0X020B        
#define RADAR_MARK_DATA_SIZE            (6) //0X020C        
#define SENTRY_INFO_DATA_SIZE           (4) //0X020D         
#define RADAR_INFO_DATA_SIZE            (1) //0X020E      
#define ROBOT_INTERACTIONDATA_DATA_SIZE (118)//0X0301  
//0X0301子内容长度：
#define INTERACTION_LAYER_DELETE_DATA_SIZE   (2)  //0X0100 
#define INTERACTION_FIGURE_DATA_SIZE         (15) //0X0101      
#define INTERACTION_FIGURE_2_DATA_SIZE       (30) //0X0102     
#define INTERACTION_FIGURE_3_DATA_SIZE       (75) //0X0103     
#define INTERACTION_FIGURE_4_DATA_SIZE       (105)//0X0104      
#define CILENT_CUSTOM_CHARATER_DATA_SIZE     (45) //0X0110     
#define SENTRY_CMD_DATA_SIZE                 (4)  //0X0120
#define RADAR_CMD_DATA_SIZE                  (1)  //0X121
//结束
#define CUSTOM_ROBOT_DATA_SIZE          (23)//长度30以内自定,可改长度 0X0302
#define MAP_COMMAND_DATA_SIZE           (11)//0X0303
#define REMOTE_CONTROL_DATA_SIZE        (12)//0X0304        
#define MAP_ROBOT_DATA_SIZE             (10)//0X0305          
#define CUSTOM_CLIENTDATA _DATA_SIZE    (8)//0X0306          
#define MAP_DATA_DATA_SIZE              (103)//0X0307        
#define CUSTOM_INFO_DATA_SIZE           (34)//0X0308          


#define INTERACTIVEHEADER_DATA_SIZE(n) (n + 9)
#define JUDGE_DATA_LENGTH(n) (n + 9)
/*--------------DataSize----------------*/

/*--------------偏移位置----------------*/
//接收数据
#define JUDGE_SOF_OFFSET (0)
#define JUDGE_DATALENGTH_OFFSET (1)
#define JUDGE_SEQ_OFFSET (3)
#define JUDGE_CRC8_OFFSET (4)
#define JUDGE_CMDID_OFFSET (5)
#define JUDGE_DATA_OFFSET (7)
#define JUDGE_CRC16_OFFSET(n) (n + JUDGE_DATA_OFFSET)
//发送数据
#define TRAMSINIT_LENGTH 128
#define TRAMSINIT_HEAD_OFFSET 0
#define TRAMSINIT_SENDID_OFFSET 2
#define TRAMSINIT_CLIENT_OFFSET 4
/*--------------偏移位置----------------*/

#pragma pack(1)
//1.	比赛机器人状态(0x0001) 1Hz
typedef struct {
    /*
    0-3 bit：比赛类型
    • 1：RoboMaster 机甲大师赛；
    • 2：RoboMaster 机甲大师单项赛；
    • 3：ICRA RoboMaster 人工智能挑战赛
    • 4：RoboMaster 联盟赛3V3
    • 5：RoboMaster 联盟赛1V1
    */
    uint8_t game_type: 4;

    /*
    4-7 bit：当前比赛阶段
    • 0：未开始比赛；
    • 1：准备阶段；
    • 2：自检阶段；
    • 3：5s倒计时；
    • 4：对战中；
    • 5：比赛结算中
    */
    uint8_t game_progress: 4;

    /*
    当前阶段剩余时间，单位 s
    */
    uint16_t stage_remain_time;

    /*
    机器人接收到该指令的精确Unix时间，当机载端收到有效的NTP服务器授时后生效
    */
    uint64_t SyncTimeStamp;

} ext_game_status_t;

//2.比赛结果数据：0x0002。发送频率：比赛结束后发送
typedef struct {
    /*0 平局 1 红方胜利 2 蓝方胜利*/
    uint8_t winner;
} ext_game_result_t;

//3. 机器人血量数据：0x0003。发送频率：3Hz
typedef struct {
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

//4.人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz周期发送，发送范围：所有机器人
//暂时用不到，就没写

//5.场地事件数据：0x0101。发送频率：1Hz
typedef struct {
    /*
    bit 0-2：
    bit 0：己方补给站前补血点的占领状态，1 为已占领
  bit 1：己方补给站内部补血点的占领状态，1 为已占领
  bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）

    bit 3-5：己方能量机关状态：
    bit 3为打击点占领状态，1为占领；
    bit 4为小能量机关激活状态，1为已激活；
    bit 5为大能量机关激活状态，1为已激活；

   bit 6-11：己方高地占领状态
 bit 6-7：己方环形高地的占领状态，1 为被己方占领，2 为被对方占领
 bit 8-9：己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
 bit 10-11：己方梯形高地的占领状态，1为被己方占领，2为被对方占领

   bit 12-18：己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）

   bit 19-27：飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0

   bit 28-29：飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，
  1为击中前哨站，2为击中基地固定目标，3为击中基地随机目标
 
   bit 30-31：中心增益点的占领情况，0为未被占领，1为被己方占领，2为被
对方占领，3为被双方占领。（仅 RMUL 适用）
    */
    uint32_t event_type;
} ext_event_data_t;

//6. 补给站动作标识：0x0102。发送频率：动作改变后发送。发送范围：己方机器人
typedef struct {

    uint8_t reserved;
    /*
    补弹机器人 ID：
 0：当前无机器人补弹
 1：红方英雄机器人补弹
 3/4/5：红方步兵机器人补弹
 101：蓝方英雄机器人补弹
 103/104/105：蓝方步兵机器人补弹
    */
    uint8_t supply_robot_id;

    /*
    出弹口开闭状态：
 0：关闭
 1：弹丸准备中
 2：弹丸释放*/

    uint8_t supply_projectile_step;

    /*
    补弹数量：
    50：50 颗子弹；
    100：100 颗子弹；
    150：150 颗子弹；
    200：200 颗子弹。
    */
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//7. 裁判警告信息：cmd_id (0x0104)。发送频率：裁判警告数据，己方判罚/判负时
//触发发送，其余时间以 1Hz 频率发送，发送范围：己方机器人。 
typedef struct {
    /*
    己方最后一次受到判罚的等级：
 1：双方黄牌
 2：黄牌
 3：红牌
 4：判负
    */
    uint8_t level;
    /*
      己方最后一次受到判罚的违规机器人 ID。（如红 1 机器人 ID 为 1，蓝
      1 机器人 ID 为 101）
 判负和双方黄牌时，该值为 0
    */
    uint8_t offending_robot_id;
    /*
      己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。(开局默认为0)
    */
    uint8_t count;
} ext_referee_warning_t;

//8. 飞镖状态信息：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。 
typedef struct {
    /*
      己方飞镖发射剩余时间，单位：秒
    */
    uint8_t dart_remaining_time;   //15s 倒计时 
    /*
    bit 0-1：
    最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中
    基地固定目标，3 为击中基地随机目标
    bit 2-4：
    对方最近被击中的目标累计被击中计数，开局默认为 0，至多为 4
    bit 5-6：
    飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基
    地固定目标为 1，选中基地随机目标为 2
bit 7-15：保留
    */
    uint16_t dart_info;
} ext_dart_info_t;

//9.比赛机器人状态：0x0201。发送频率：10Hz
typedef struct {
    /*
    机器人 ID：
    1：红方英雄机器人；
    2：红方工程机器人；
    3/4/5：红方步兵机器人；
    6：红方空中机器人；
    7：红方哨兵机器人；
    8：红方飞镖机器人；
    9：红方雷达站；  
    101：蓝方英雄机器人；
    102：蓝方工程机器人；
    103/104/105：蓝方步兵机器人；
    106：蓝方空中机器人；
    107：蓝方哨兵机器人。
    108：蓝方飞镖机器人； 
    109：蓝方雷达站。 
    */
    uint8_t robot_id;

    /*
    机器人等级
    */
    uint8_t robot_level;

    /*
    机器人当前血量
    */
    uint16_t current_HP;

    /*
    机器人上限血量
    */
    uint16_t maximum_HP;

    /*
    机器人枪口热量每秒冷却值
    */
    uint16_t shooter_barrel_cooling_value;

    /*
    机器人枪口热量上限
    */
    uint16_t shooter_barrel_heat_limit;

    /*
    机器人底盘功率上限
    */
    uint16_t chassis_power_limit;
    /*
   电源管理模块的输出情况：
 bit 0：gimbal 口输出：0 为无输出，1 为 24V 输出
 bit 1：chassis 口输出：0 为无输出，1 为 24V 输出
 bit 2：shooter 口输出：0 为无输出，1 为 24V 输出
 bit 3-7：保留
    */
    uint8_t power_management_gimbal_output: 1;
    uint8_t power_management_chassis_output: 1;
    uint8_t power_management_shooter_output: 1;
} ext_game_robot_status_t;

//10.实时功率热量数据：0x0202。发送频率：50Hz
typedef struct {
    //底盘输出电压 单位 毫伏
    uint16_t chassis_volt;
    //底盘输出电流 单位 毫安
    uint16_t chassis_current;
    //底盘输出功率 单位 W 瓦
    float chassis_power;
    //底盘功率缓冲 单位 J 焦耳
    uint16_t chassis_power_buffer;
    //1号17mm 枪口热量
    uint16_t shooter_id1_17mm_cooling_heat;
    //2号17mm枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;
    //42mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

//11.机器人位置：0x0203。发送频率：1Hz
typedef struct {
    float x;   //位置 x 坐标，单位 m
    float y;   //位置 y 坐标，单位 m
    float angle;   //本机器人测速模块的朝向，单位：度。正北为 0 度
} ext_robot_pos_t;

//12. 机器人增益：0x0204。发送频率：3Hz
typedef struct {
    /*
    机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    */
    uint8_t recovery_buff;
    /*
  机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）
   */
    uint8_t cooling_buff;
    /*
   机器人防御增益（百分比，值为 50 表示 50%防御增益）
   */
    uint8_t defence_buff;
    /*
   机器人负防御增益（百分比，值为 30 表示-30%防御增益）
   */
    uint8_t vulnerability_buff;
    /*
   机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
   */
    uint16_t attack_buff;
} ext_buff_t;

//13. 空中机器人能量状态：0x0205。发送频率：1Hz
typedef struct {
    /*
     空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）
     */
    uint8_t airforce_status;
    /*
   此状态的剩余时间（单位为：秒，向下取整，即冷却时间剩余 1.9 秒时，此
值为 1）若冷却时间为 0，但未呼叫空中支援，则该值为 0
   */
    uint8_t time_remain;
} ext_air_support_data_t;


//14. 伤害状态：0x0206。发送频率：伤害发生后发送
typedef struct {
    /*
    bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
	该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    */
    uint8_t armor_id: 4;
    /*bit 4-7：血量变化类型
    0：装甲模块被弹丸攻击导致扣血
    1：裁判系统重要模块离线导致扣血
    2：射击初速度超限导致扣血
    3：枪口热量超限导致扣血
    4：底盘功率超限导致扣血
    5：装甲模块受到撞击导致扣血
    */
    uint8_t HP_deduction_reason: 4;
} ext_hurt_data_t;

//15. 实时射击信息：0x0207。发送频率：射击后发送
typedef struct {
    /*子弹类型: 1：17mm弹丸 2：42mm弹丸*/
    uint8_t bullet_type;
    /*
    发射机构ID：
    1：1号17mm发射机构
    2：2号17mm发射机构
    3：42mm 发射机构
    */
    uint8_t shooter_number;
    /*子弹射频 单位 Hz*/
    uint8_t launching_frequency;
    /*子弹射速 单位 m/s*/
    float initial_speed;
} ext_shoot_data_t;

//16. 子弹剩余发射数：0x0208。发送频率：10Hz周期发送，所有机器人发送 
typedef struct {
    uint16_t bullet_remaining_num_17mm;//17mm子弹剩余发射数目
    uint16_t bullet_remaining_num_42mm;//42mm子弹剩余发射数目
    uint16_t remaining_gold_coin;//剩余金币数量
} ext_projectile_allowance_t;

//17. 机器人 RFID 状态：0x0209。发送频率：3Hz，发送范围：单一机器人。 
typedef struct {
    /*
   bit 位值为 1/0 的含义：是否已检测到该增益点 RFID 卡
 bit 0：己方基地增益点
 bit 1：己方环形高地增益点
 bit 2：对方环形高地增益点
 bit 3：己方 R3/B3 梯形高地增益点
 bit 4：对方 R3/B3 梯形高地增益点
 bit 5：己方 R4/B4 梯形高地增益点
 bit 6：对方 R4/B4 梯形高地增益点
 bit 7：己方能量机关激活点
 bit 8：己方飞坡增益点（靠近己方一侧飞坡前）
 bit 9：己方飞坡增益点（靠近己方一侧飞坡后）
 bit 10：对方飞坡增益点（靠近对方一侧飞坡前）
 bit 11：对方飞坡增益点（靠近对方一侧飞坡后）
 bit 12：己方前哨站增益点
 bit 13：己方补血点（检测到任一均视为激活）
 bit 14：己方哨兵巡逻区
   bit 15：对方哨兵巡逻区
 bit 16：己方大资源岛增益点
 bit 17：对方大资源岛增益点
 bit 18：己方兑换区
 bit 19：中心增益点（仅 RMUL 适用）
 bit 20-31：保留
注：基地增益点、高地增益点、飞坡增益点、前哨站增益点、资源岛增益点、
补血点、兑换区、中心增益点（仅适用于 RMUL）和哨兵巡逻区的 RFID 卡
仅在赛内生效。在赛外，即使检测到对应的 RFID 卡，对应值也为0。
   */
    uint32_t rfid_status;
} ext_rfid_status_t;

//18. 飞镖机器人客户端指令数据：0x020A。发送频率：3Hz，发送范围：单一机器人
typedef struct {
    /*
   当前飞镖发射站的状态：
 1：关闭
 2：正在开启或者关闭中
 0：已经开启
    */
    uint8_t dart_launch_opening_status;
    //保留
    uint8_t reserved;
    /*
   切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为0。
    */
    uint16_t target_change_time;
    /*最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为0。*/
    uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;
//19. 地面机器人位置数据：0x020B。发送频率：1Hz，发送范围：哨兵机器人
typedef struct {

    //己方英雄机器人位置 x 轴坐标，单位：m
    float hero_x;

    //己方英雄机器人位置 y 轴坐标，单位：m
    float hero_y;

    //己方工程机器人位置 x 轴坐标，单位：m
    float engineer_x;

    //己方工程机器人位置 y 轴坐标，单位：m
    float engineer_y;

    //己方 3 号步兵机器人位置 x 轴坐标，单位：m
    float standard_3_x;

    //己方 3 号步兵机器人位置 y 轴坐标，单位：m
    float standard_3_y;

    //己方 4 号步兵机器人位置 x 轴坐标，单位：m
    float standard_4_x;

    //己方 4 号步兵机器人位置 y 轴坐标，单位：m
    float standard_4_y;

    //己方 5 号步兵机器人位置 x 轴坐标，单位：m
    float standard_5_x;

    //己方 5 号步兵机器人位置 y 轴坐标，单位：m
    float standard_5_y;

} ext_ground_robot_position_t;
//20. 雷达标记进度数据：0x020C。发送频率：1Hz，发送范围：单一机器人
typedef struct {
    //对方英雄机器人被标记进度：0-120
    uint8_t mark_hero_progress;
    //对方工程机器人被标记进度：0-120
    uint8_t mark_engineer_progress;
    //对方 3 号步兵机器人被标记进度：0-120
    uint8_t mark_standard_3_progress;
    //对方 4 号步兵机器人被标记进度：0-120
    uint8_t mark_standard_4_progress;
    //对方 5 号步兵机器人被标记进度：0-120
    uint8_t mark_standard_5_progress;
    //对方哨兵机器人被标记进度：0-120
    uint8_t mark_sentry_progress;

} ext_radar_mark_data_t;
//21.哨兵自主决策信息同步:0X020D。发送频率：1Hz，发送范围：单一机器人
typedef struct {
    /*
    bit 0-10：除远程兑换外，哨兵成功兑换的发弹量，开局为 0，在哨兵成功兑
换一定发弹量后，该值将变为哨兵成功兑换的发弹量值。
bit 11-14：哨兵成功远程兑换发弹量的次数，开局为 0，在哨兵成功远程兑
换发弹量后，该值将变为哨兵成功远程兑换发弹量的次数。
bit 15-18：哨兵成功远程兑换血量的次数，开局为 0，在哨兵成功远程兑换
血量后，该值将变为哨兵成功远程兑换血量的次数。
bit 19-31：保留
    */
    uint32_t sentry_info;
} ext_sentry_info_t;

//22.雷达自主决策信息同步:0X020E。发送频率：1Hz，发送范围：单一机器人
typedef struct {
    /*
    bit 0-1：雷达是否拥有触发双倍易伤的机会，开局为 0，数值为雷达拥有触发
双倍易伤的机会，至多为 2
bit 2：对方是否正在被触发双倍易伤
 0：对方未被触发双倍易伤
 1：对方正在被触发双倍易伤
bit 3-7：保留
    */
    uint8_t radar_info;
} ext_radar_info_t;

/*23.机器人交互数据:0X0301。发送频率：发送方触发发
送，频率上限为 10Hz 发送范围：单一机器人*/
typedef struct {
    //子内容 ID
    uint16_t data_cmd_id;
    //发送者 ID
    uint16_t sender_id;
/*
	接收者 ID
    仅限己方通信需为规则允许的多机通讯接收者
    若接收者为选手端，则仅可发送至发送者对应的选手端 ID 编号详见附录
*/
    uint16_t receiver_id;
    uint8_t user_data[113];
} ext_robot_interaction_data_t;
//以下是子内容的ID:

//子内容 ID：0x0100
typedef struct {
    /*删除操作
 0：空操作
 1：删除图层
 2：删除所有*/
    uint8_t delete_type;
    //图层数 图层数：0~9
    uint8_t layer;
} ext_interaction_layer_delete_t;
//子内容 ID：0x0101
typedef struct {
    //图形名 在图形删除、修改等操作中，作为索引
    uint8_t figure_name[3];
    /*
    图形配置1
bit 0-2：图形操作
 0：空操作
 1：增加
 2：修改
 3：删除
bit 3-5：图形类型
 0：直线
 1：矩形
 2：正圆
 3：椭圆
 4：圆弧
 5：浮点数
 6：整型数
 7：字符
bit 6-9：图层数（0~9）
bit 10-13：颜色
 0：红/蓝（己方颜色）
 1：黄色
 2：绿色
 3：橙色
 4：紫红色
 5：粉色
 6：青色
 7：黑色
 8：白色
bit 14-31：根据绘制的图形不同，含义不同，详见“表 2-26 图形细节参数说明”
    */
    uint32_t operate_tpye: 3;
    uint32_t figure_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t details_a: 9;
    uint32_t details_b: 9;
/*
   图形配置2
   bit 0-9：线宽，建议字体大小与线宽比例为 10：1
   bit 10-20：起点/圆心 x 坐标
   bit 21-31：起点/圆心 y 坐标
*/
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
//图形配置3 根据绘制的图形不同，含义不同，详见“表 2-26 图形细节参数说明”   
    uint32_t details_c: 10;
    uint32_t details_d: 11;
    uint32_t details_e: 11;
} ext_interaction_figure_t;
/*表 2-26 图形细节参数说明
类型 details_a  details_b  details_c   details_d      details_e
直线     -          -        -         终点x坐标      终点y坐标
矩形     -          -        -       对角顶点x坐标   对角顶点y坐标
正圆     -          -       半径          -               -
椭圆     -          -        -         x半轴长度       y半轴长度
圆弧   起始角度   终止角度   -         x半轴长度       y半轴长度
浮点数 字体大小   无作用     (      该值除以1000即实际显示值  )
整型数 字体大小     -        (     32位整型数，int32_t        )
字符   字体大小   字符长度   -            -               -


角度值含义为：0°指12点钟方向，顺时针绘制；
屏幕位置：（0,0）为屏幕左下角（1920，1080）为屏幕右上角；
浮点数：整型数均为 32 位，对于浮点数，实际显示的值为输入的值/1000，
如在details_c、details_d、details_e 对应的字节输入 1234，
选手端实际显示的值将为 1.234。
即使发送的数值超过对应数据类型的限制,图形仍有可能显示，
但此时不保证显示的效果
*/
//子内容 ID：0x0102
typedef struct {
    /*图形 1 与 0x0101 的数据段相同
      图形 2 与 0x0101 的数据段相同*/
    ext_interaction_figure_t interaction_figure[2];
} ext_interaction_figure_2_t;
//子内容 ID：0x0103
typedef struct {
    /*
      图形 1 与 0x0101 的数据段相同
      图形 2 与 0x0101 的数据段相同
      图形 3 与 0x0101 的数据段相同
      图形 4 与 0x0101 的数据段相同
      图形 5 与 0x0101 的数据段相同
    */
    ext_interaction_figure_t interaction_figure[5];
} ext_interaction_figure_3_t;
//子内容 ID：0x0104
typedef struct {
    /*
    图形 1 与 0x0101 的数据段相同
    图形 2 与 0x0101 的数据段相同
    图形 3 与 0x0101 的数据段相同
    图形 4 与 0x0101 的数据段相同
    图形 5 与 0x0101 的数据段相同
    图形 6 与 0x0101 的数据段相同
    图形 7 与 0x0101 的数据段相同
    */
    ext_interaction_figure_t interaction_figure[7];
} ext_interaction_figure_4_t;
//子内容 ID：0x0110
//相比串口协议有修改
typedef struct {
    ext_interaction_figure_t character_figure;
    uint8_t char_data[30];
} ext_interaction_character_t;

//子内容 哨兵自主决策指令：0x0120
typedef struct {
/*
bit 0：哨兵机器人是否确认复活
 0 表示哨兵机器人确认不复活，即使此时哨兵的复活读条已经完成
 1 表示哨兵机器人确认复活，若复活读条完成将立即复活
bit 1：哨兵机器人是否确认兑换立即复活
 0 表示哨兵机器人确认不兑换立即复活；
 1 表示哨兵机器人确认兑换立即复活，若此时哨兵机器人
	符合兑换立即复活的规则要求，则会立即消耗金币兑换立即复活
bit 2-12：
	哨兵将要兑换的发弹量值，开局为0，修改此值后，哨兵在补血点
	即可兑换允许发弹量。此值的变化需要单调递增，否则视为不合法。
示例：此值开局仅能为0，此后哨兵可将其从0修改至X，则消
    耗X金币成功兑换 X 允许发弹量。此后哨兵可将其从X修改至X+Y，以此类推。
bit 13-16：
	哨兵远程兑换发弹量的请求次数，开局为0，修改此值即可请求远程兑换发弹量。
    此值的变化需要单调递增且每次仅能增加 1，否则视为不合法。
	
示例：此值开局仅能为0，此后哨兵可将其从0修改至1，则消
      耗金币远程兑换允许发弹量。此后哨兵可将其从1修改至2，以此类推。
bit 17-20：哨兵远程兑换血量的请求次数，开局为0，修改此
           值即可请求远程兑换血量。此值的变化需要单调递增
	       且每次仅能增加 1，否则视为不合法。
示例：此值开局仅能为0，此后哨兵可将其从0修改至1，则消
耗金币远程兑换血量。此后哨兵可将其从1修改至 2，以此类推。

在哨兵发送该子命令时，服务器将按照从相对低位到相对高位
的原则依次处理这些指令，直至全部成功或不能处理为止。

示例：若队伍金币数为 0，此时哨兵战亡，“是否确认复活”的
值为 1，“是否确认兑换立即复活”的值为 1，“确认兑换的允
许发弹量值”为 100。（假定之前哨兵未兑换过允许发弹量）由
于此时队伍金币数不足以使哨兵兑换立即复活，则服务器将会
忽视后续指令，等待哨兵发送的下一组指令。
bit 21-31：保留
*/
    uint32_t sentry_cmd;
} ext_sentry_cmd_t;

//子内容 雷达自主决策指令：0x0121
typedef struct {
    /*
    雷达是否确认触发双倍易伤
    开局为 0，修改此值即可请求触发双倍易伤，若此时雷达拥有
触发双倍易伤的机会，则可触发。此值的变化需要单调递增且
每次仅能增加 1，否则视为不合法。

示例：此值开局仅能为0，此后雷达可将其从0修改至1，
若雷达拥有触发双倍易伤的机会，则触发双倍易伤。此后雷达可将
其从1修改至2，以此类推。若雷达请求双倍易伤时，
双倍易伤正在生效，则第二次双倍易伤将在第一次双倍易伤结束后生效。
    */
    uint8_t radar_cmd;
} ext_radar_cmd_t;
//子内容结束


//24.选手端小地图交互数据:0X0302。发送频率：发送方触发发送，
//频率上限为 30Hz 发送范围：单一机器人
typedef struct {
    //自定义数据 可改长度
    uint8_t data[23];
} ext_custom_robot_data_t;

//25.选手端小地图交互数据:0X0303。发送频率：选手端触发发送 发送范围：单一机器人
typedef struct {
    /*
    目标位置x轴坐标，单位m。备注：当发送目标机器人ID 时，该值为0
    */
    float target_position_x;
    /*
    目标位置y轴坐标，单位m。备注：当发送目标机器人ID 时，该值为0
    */
    float target_position_y;
    /*
    云台手按下的键盘按键通用键值。备注：无按键按下，则为0
    */
    uint8_t cmd_keyboard;
    //对方机器人ID 备注：当发送坐标数据时，该值为 0
    uint8_t target_robot_id;
    //信息来源ID 备注：信息来源的ID，ID对应关系详见附录
    uint8_t cmd_source;
} ext_map_command_t;
//26.选手端小地图接收雷达数据:0X0305。发送频率：10Hz 发送范围：己方所有选手端
typedef struct {
    /*
    目标机器人 ID
    */
    uint16_t target_robot_id;
    /*
    目标x位置坐标，单位：m 备注：当x、y超出边界时则不显示
    */
    float target_position_x;
    /*
    目标y位置坐标，单位：m 备注：当x、y超出边界时则不显示
    */
    float target_position_y;
} ext_map_robot_data_t;

//25.选手端小地图接收哨兵数据:0X0307。发送频率：1Hz 发送范围：对应操作手选手端
typedef struct {
    /*1：到目标点攻击
      2：到目标点防守
      3：移动到目标点*/
    uint8_t intention;
    //路径起点 x 轴坐标，单位：dm
    uint16_t start_position_x;
    /*note:小地图左下角为坐标原点，水平向右为 X 轴正方向，竖直向上为 Y 轴正方向。
显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界的位置将在边界处显示*/

    //路径起点 y 轴坐标，单位：dm
    uint16_t start_position_y;
    //路径点 x轴增量数组，单位：dm
    int8_t delta_x[49];
    //路径点 y轴增量数组，单位：dm
    /*note:增量相较于上一个点位进行计算，共 49 个新点位，X 与 Y 轴增量对应组成点位*/
    int8_t delta_y[49];
    //发送者ID，备注：需与自身 ID 匹配，ID 编号详见附录
    uint16_t sender_id;
} ext_map_data_t;

//27.选手端小地图接收机器人数据，:0X0308。发送频率：3Hz 发送范围：己方选手端
typedef struct {
    /*发送者的ID。备注：需要校验发送者的ID正确性*/
    uint16_t sender_id;
    /*接收者的ID。备注：需要校验接收者的ID正确性，仅支持发送己方选手端*/
    uint16_t receiver_id;
    /*字符 备注：以utf-16格式编码发送，支持显示中文。编码发送时请注意数
据的大小端问题*/
    uint8_t user_data[30];
} ext_custom_info_t;

//28.图传遥控信息标识：0x0304。发送频率：30Hz。
typedef struct {
    /*鼠标x轴移动速度，负值标识向左移动*/
    int16_t mouse_x;
    /*鼠标y轴移动速度，负值标识向下移动*/
    int16_t mouse_y;
    /*鼠标滚轮移动速度，负值标识向后滚动*/
    int16_t mouse_z;
    /*鼠标左键是否按下：0为未按下；1为按下*/
    int8_t left_button_down;
    /*鼠标右键是否按下：0为未按下，1为按下*/
    int8_t right_button_down;
    /*
    键盘信息
	键盘按键信息，每个bit对应一个按键，0为未按下，1为按下：
    bit 0：键盘W是否按下
    bit 1：键盘S是否按下
    bit 2：键盘A是否按下
    bit 3：键盘D是否按下
    bit 4：键盘SHIFT是否按下
    bit 5：键盘CTRL是否按下
    bit 6：键盘Q是否按下
    bit 7：键盘E是否按下
    bit 8：键盘R是否按下
    bit 9：键盘F是否按下
    bit 10：键盘G是否按下
    bit 11：键盘Z是否按下
    bit 12：键盘X是否按下
    bit 13：键盘C是否按下
    bit 14：键盘V是否按下
    bit 15：键盘B是否按下
    */
    uint16_t keyboard_value;
    /*保留位*/
    uint16_t reserved;
} ext_remote_control_t;

//29.自定义控制器与选手端交互数据:0x0306，发送方触发发送，频率上限为 30Hz
typedef struct {
    /*键盘键值：
 bit 0-7：按键1键值
 bit 8-15：按键2键值*/
    uint16_t key_value;
    /*bit 0-11：鼠标X轴像素位置
 bit 12-15：鼠标左键状态*/
    uint16_t x_position: 12;
    uint16_t mouse_left: 4;
    /*bit 0-11：鼠标Y轴像素位置
 bit 12-15：鼠标右键状态*/
    uint16_t y_position: 12;
    uint16_t mouse_right: 4;
    //保留位
    uint16_t reserved;
} ext_custom_client_data_t;
#pragma pack()


extern ext_game_status_t ext_game_status;//0X0001
extern ext_game_result_t ext_game_result;//0X0002
extern ext_game_robot_HP_t ext_game_robot_HP;//0X0003
extern ext_event_data_t ext_event_data;//0X0101
extern ext_supply_projectile_action_t ext_supply_projectile_action;//0X0102
extern ext_referee_warning_t ext_referee_warning;//0X0104
extern ext_dart_info_t ext_dart_info;//0X0105
extern ext_game_robot_status_t ext_game_robot_status;//0X0201
extern ext_power_heat_data_t ext_power_heat_data;//0X0202
extern ext_robot_pos_t ext_robot_pos;//0X0203
extern ext_buff_t ext_buff;//0X0204
extern ext_air_support_data_t ext_air_support_data;//0X0205
extern ext_hurt_data_t ext_hurt_data;//0X0206
extern ext_shoot_data_t ext_shoot_data;//0X0207
extern ext_projectile_allowance_t ext_projectile_allowance;//0X0208
extern ext_rfid_status_t ext_rfid_status;//0X0209
extern ext_dart_client_cmd_t ext_dart_client_cmd;//0X020A
extern ext_ground_robot_position_t ext_ground_robot_pos;//0X020B
extern ext_radar_mark_data_t ext_radar_mark_data;//0X020C
extern ext_sentry_info_t ext_sentry_info;//0X020D
extern ext_radar_info_t ext_radar_info;//0X020E
extern ext_robot_interaction_data_t ext_robot_interaction_data;//0X0301
extern ext_map_command_t ext_map_command;//0X0303
extern ext_map_robot_data_t ext_map_robot_data;//0X0305
extern ext_map_data_t ext_map_data;//0X0307
extern ext_custom_info_t ext_custom_info;//0X0308
extern ext_custom_robot_data_t ext_custom_robot_data;//0X0302
extern ext_remote_control_t ext_remote_control;//0X0304
extern ext_custom_client_data_t ext_custom_client_data;//0X0306
extern TickType_t ext_judge_last_receive_time;


void RefereeReceive(uint8_t judge_receive_counter, uint8_t *judge_receive_buffer);

void Referee_Receive_Data_Processing(uint8_t SOF, uint16_t CmdID, uint8_t *judge_receive_buffer);

#endif

#ifndef _REFEREE_CRC_H__
#define _REFEREE_CRC_H__

/*-------------------------------------------CRC校验---------------------------------------------------*/
/**
  * @brief  裁判系统数据校验
  * @param  __RECEIVEBUFFER__：  接收到的裁判系统数据头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval 1：                  校验正确
  * @retval 0：                  校验错误
  * @note	None
  */
#define Verify_CRC_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__) (Verify_CRC8_Check_Sum(__RECEIVEBUFFER__, JUDGE_CRC8_OFFSET + 1) && Verify_CRC16_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__ + JUDGE_DATA_LENGTH(0)))

/**
  * @brief  裁判系统添加校验
  * @param  __TRANSMITBUFFER__： 发送到裁判系统的数据中头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval None
  * @note	None
  */
#define Append_CRC_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__)                       \
do                                                                                     \
{                                                                                      \
    Append_CRC8_Check_Sum(__TRANSMITBUFFER__, JUDGE_CRC8_OFFSET + 1);                  \
    Append_CRC16_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__ + JUDGE_DATA_LENGTH(0)); \
} while (0U)

/*--------------------------------------------------校验函数--------------------------------------------------*/
void judge_Reset();

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);


/*--------------------------------------------------校验函数--------------------------------------------------*/

extern uint8_t judge_rx_buffer[2][UART6_MAX_RECEIVE_BUFFER_LENGTH];
#endif
