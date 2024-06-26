/**
 * @file ita_chariot.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef TSK_INTERACTION_H
#define TSK_INTERACTION_H

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"
#include "drv_tim.h"
#include "dvc_imu.h"
#include "alg_fsm.h"
#include "crt_chassis.h"
#include "dvc_ER08.h"
#include "dvc_SIM900A.h"
#include "dvc_Tjc011.h"
/* Exported macros -----------------------------------------------------------*/
class Class_Chariot;
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台Pitch状态枚举
 *
 */
enum Enum_Pitch_Control_Status 
{
    Pitch_Status_Control_Free = 0, 
    Pitch_Status_Control_Lock ,
};

enum Enum_MinPC_Aim_Status
{
    MinPC_Aim_Status_DISABLE = 0,
    MinPC_Aim_Status_ENABLE,
};

/**
 * @brief 摩擦轮状态
 *
 */
enum Enum_Fric_Status :uint8_t
{
    Fric_Status_CLOSE = 0,
    Fric_Status_OPEN,
};


/**
 * @brief 弹舱状态类型
 *
 */
enum Enum_Bulletcap_Status :uint8_t
{
    Bulletcap_Status_CLOSE = 0,
    Bulletcap_Status_OPEN,
};


/**
 * @brief 底盘通讯状态
 *
 */
enum Enum_Chassis_Status
{
    Chassis_Status_DISABLE = 0,
    Chassis_Status_ENABLE,
};

/**
 * @brief 云台通讯状态
 *
 */
enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE = 0,
    Gimbal_Status_ENABLE,
};

/**
 * @brief DR16控制数据来源
 *
 */
enum Enum_DR16_Control_Type
{
    DR16_Control_Type_REMOTE = 0,
    DR16_Control_Type_KEYBOARD,
};

/**
 * @brief 机器人控制有限自动机
 *
 */
class Class_FSM_Chariot_Control : public Class_FSM
{
public:
    Class_Chariot *Chariot;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief 控制对象
 *
 */
class Class_Chariot
{
public:

    Class_Chassis Chassis;
    Class_ER08 ER08;
    Class_SIM900A SIM900A;
    Class_Tjc011 Tjc011;
    friend class Class_FSM_Chariot_Control;

    void Init();
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();

protected:

};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
