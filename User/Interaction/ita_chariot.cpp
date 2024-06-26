/**
 * @file ita_chariot.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"
#include "drv_math.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
void Get_Burry(uint16_t __time_cnt);
void Put_Burry(uint16_t __time_cnt);
/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init()
{
    ER08.Init(&huart1);
    Tjc011.Init(&huart2);

    Chassis.IMU.Init();
    Chassis.Init();
}

/**
 * @brief 计算回调函数
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
}

/**
 * @brief 在线判断回调函数
 *
 */
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
}

/**
 * @brief 有限状态机控制回调函数
 *
 */
uint16_t time_cnt=0;
void Class_FSM_Chariot_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
    case 0:
        /*等待状态*/
        // 扫描模块轮询扫描
        // x:0 y:0
        // 夹爪初始位置
        Get_Burry(Status[Now_Status_Serial].Time);
        if (Status[Now_Status_Serial].Time >= 3000)
        {
			// 设置目标点
            Chariot->Chassis.Set_Target_Position_X(1);
            Chariot->Chassis.Set_Target_Position_Y(1);
            Chariot->Chassis.Set_Target_Angle(0);

            // 到达目标点 延时1s 跳转到下一个状态			
            if(fabs(Chariot->Chassis.Get_Now_Position_X() - Chariot->Chassis.Get_Target_Position_X()) < 0.1 && 
                (fabs(Chariot->Chassis.Get_Now_Position_Y() - Chariot->Chassis.Get_Target_Position_Y()) < 0.1))
            {
                time_cnt++;
                if(time_cnt >= 1000)
                Set_Status(1);                
            }
        }

        break;
            
        case 1:
            /*信息录入状态*/
            Put_Burry(Status[Now_Status_Serial].Time);
            if(Status[Now_Status_Serial].Time >= 3000)
            {
                //收回机械臂
                Get_Burry(Status[Now_Status_Serial].Time);
                // 设置目标点
                Chariot->Chassis.Set_Target_Position_X(0);
                Chariot->Chassis.Set_Target_Position_Y(0);
                Chariot->Chassis.Set_Target_Angle(0);
            }
            //Set_Status(0);
            break;

        case 2:
            /*导航前往状态*/
            break;

        case 3:
            /*夹爪放件状态*/
            break;

        case 4:
            /*导航返回状态*/
            break;

        case 5:
            /*取件/放件类型判断状态*/
            break;

        case 6:
            /*取件-夹爪取出货状态*/
            break;

        case 7:
            /*取件码判断状态*/
            break;
    }

    // Chariot->Chassis.Position_Y_PID.Set_Target(Chariot->Chassis.Get_Target_Position_Y());
    // Chariot->Chassis.Position_Y_PID.Set_Now(Chariot->Chassis.Get_Now_Position_Y());
    // Chariot->Chassis.Position_Y_PID.TIM_Adjust_PeriodElapsedCallback();
    // Chariot->Chassis.Set_Target_Velocity_Y(Chariot->Chassis.Position_Y_PID.Get_Out());

    // Chariot->Chassis.Position_X_PID.Set_Target(Chariot->Chassis.Get_Target_Position_X());
    // Chariot->Chassis.Position_X_PID.Set_Now(Chariot->Chassis.Get_Now_Position_X());
    // Chariot->Chassis.Position_X_PID.TIM_Adjust_PeriodElapsedCallback();
    // Chariot->Chassis.Set_Target_Velocity_X(-1.0f * Chariot->Chassis.Position_X_PID.Get_Out());

    // Chariot->Chassis.Position_Yaw_PID.Set_Target(Chariot->Chassis.Get_Target_Angle());
    // Chariot->Chassis.Position_Yaw_PID.Set_Now(Chariot->Chassis.Get_Now_Angle());
    // Chariot->Chassis.Position_Yaw_PID.TIM_Adjust_PeriodElapsedCallback();
    // Chariot->Chassis.Set_Target_Omega(Chariot->Chassis.Position_Yaw_PID.Get_Out());
}

// 1号 tim14 ch1  1000   	800
// 2号 tim13 ch1  1250 		400
// 3号 tim5  ch3  1550 		1200
// 4号 tim5  ch4  1250 		450

// 夹取 1 3 4 2
// 放回 2 4 3 1

void Get_Burry(uint16_t __time_cnt)
{
    // 默认先松开夹爪
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);
    if (__time_cnt >= 50) // 夹取
    {
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 800);
    }
    if (__time_cnt >= 450) // 抬起大臂 500ms
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1200);
    }
    if (__time_cnt >= 720) // 旋转yaw 500ms
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 450);
    }
    if (__time_cnt >= 920) // 折叠小臂 50ms
    {
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 400);
    }
}

void Put_Burry(uint16_t __time_cnt)
{
    // 默认先加紧夹爪
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 800);
    if (__time_cnt >= 50) // 抬起小臂 500ms
    {
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 1250);
    }
    if (__time_cnt >= 260) // 旋转yaw 500ms
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 1250);
    }
    if (__time_cnt >= 660) // 伸长大臂 500ms
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1550);
    }
    if (__time_cnt >= 1260) // 松开夹爪 50ms
    {
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
