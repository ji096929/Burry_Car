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
void Init_Position();
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

    Now_Cargo_Number = 0;
    memcpy(Cargo,0,10*sizeof(Struct_Cargo));
}

void Class_Chariot::Get_Cargo_Data()
{
    if(Now_Cargo_Number >= 10)
        return;
    for (auto i = 0; i < Now_Cargo_Number; i++)
    {
        if (ER08.Get_x() == Cargo[i].Position_X && ER08.Get_y() == Cargo[i].Position_Y)
        {
            return;
        }
    }
    Cargo[Now_Cargo_Number].Position_X = ER08.Get_x();
    Cargo[Now_Cargo_Number].Position_Y = ER08.Get_y();
    memcpy(Cargo[Now_Cargo_Number].Code,ER08.Get_code(),4*sizeof(uint8_t));
    memcpy(Cargo[Now_Cargo_Number].Phone_Number,ER08.Get_tel(),11*sizeof(uint8_t));
    memcpy(&Now_Cargo,&Cargo[Now_Cargo_Number],sizeof(Struct_Cargo));
    Now_Cargo_Number++;
}

uint8_t Class_Chariot::Jundge_Cargo()
{
    if(Now_Cargo_Number == 0)
        return 0;
    uint8_t tmp[4];
    for (auto i = 0; i < Now_Cargo_Number; i++)
    {
        memcpy(tmp,Tjc011.Get_Input_Number(),4*sizeof(uint8_t));
        if (memcmp(tmp,Cargo[i].Code,4*sizeof(uint8_t)) == 0)
        {
            memcpy(&Now_Cargo,&Cargo[i],sizeof(Struct_Cargo));
            return 1;
        }
    }
    return 0;   
}

void Class_Chariot::Set_Control_Status(Enum_Chariot_Control_Status status)
{
    Control_Status = status;
}

Enum_Chariot_Control_Status Class_Chariot::Get_Control_Status()
{
    return Control_Status;
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
        {
            /*等待状态*/
            // 夹爪初始位置
            Init_Position();

            // 设置目标点 x:0 y:0
            Chariot->Chassis.Set_Target_Position_X(0);
            Chariot->Chassis.Set_Target_Position_Y(0); 

            //目前未收到任务
            Chariot->Set_Control_Status(Chariot_Disable_Status);

            // 获取货物信息 如果更新就获取信息 转到前往状态
            if(Chariot->ER08.Updata_Flag)
            {
                Chariot->Get_Cargo_Data();
                Chariot->ER08.Updata_Flag = 0;
                Chariot->Set_Control_Status(Chariot_Put_Cargo_Status);
                Set_Status(1);
            }

            // 获取取件码信息 如果更新就获取信息 转到取件状态
            if(Chariot->Tjc011.Input_data_Flag)
            {
                if(Chariot->Jundge_Cargo())  //如果取件码正确 就转到前往状态
                {
                    Chariot->Set_Control_Status(Chariot_Get_Cargo_Status);
                    Set_Status(1);                    
                }
                Chariot->Tjc011.Input_data_Flag = 0;
            }
        }
        break;
        case 1:
           /*导航前往状态*/

            // 夹取货物
            Get_Burry(Status[Now_Status_Serial].Time);

            //夹取完成后延时1s
            if (Status[Now_Status_Serial].Time >= 4000)
            {
                // 设置目标点
                Chariot->Chassis.Set_Target_Position_X(Chariot->Now_Cargo.Position_X);
                Chariot->Chassis.Set_Target_Position_Y(Chariot->Now_Cargo.Position_Y);
                Chariot->Chassis.Set_Target_Angle(0);

                // 到达目标点 跳转到下一个状态			
                if(fabs(Chariot->Chassis.Get_Now_Position_X() - Chariot->Chassis.Get_Target_Position_X()) < 0.02 && 
                    (fabs(Chariot->Chassis.Get_Now_Position_Y() - Chariot->Chassis.Get_Target_Position_Y()) < 0.02))
                {
                    Set_Status(2);           
                }
            }
        break;
        case 2:
            /*放件/取件状态*/
            //先延时1s
            if(Status[Now_Status_Serial].Time>=1000)
            {
                //取件
                if(Chariot->Get_Control_Status()==Chariot_Get_Cargo_Status)
                {
                    //夹取货柜的货物

                }
                //放件
                else if(Chariot->Get_Control_Status()==Chariot_Put_Cargo_Status)
                {
                    //放置货物
                    
                    //发送短信
                    Chariot->SIM900A.Sim900a_Send_Data((char *)Chariot->Now_Cargo.Code,(char *)Chariot->Now_Cargo.Phone_Number);    
                }
            }
            //整个操作5s后返回
            if(Status[Now_Status_Serial].Time>5000)
            {
                Set_Status(3);
            }
        break;
        case 3:
            /*导航返回状态*/

            // 设置目标点
            Chariot->Chassis.Set_Target_Position_X(0);
            Chariot->Chassis.Set_Target_Position_Y(0);
            Chariot->Chassis.Set_Target_Angle(0);

            // 到达目标点 跳转到下一个状态			
            if(fabs(Chariot->Chassis.Get_Now_Position_X() - Chariot->Chassis.Get_Target_Position_X()) < 0.02 && 
                (fabs(Chariot->Chassis.Get_Now_Position_Y() - Chariot->Chassis.Get_Target_Position_Y()) < 0.02))
            {
                Set_Status(2);           
            }       
        break;

        case 4:
            /*返回类型判断状态*/
            //先延时1s
            if(Status[Now_Status_Serial].Time>=1000)
            {
                //取件返回
                if(Chariot->Get_Control_Status()==Chariot_Get_Cargo_Status)
                {
                    //返回初始状态
                    Set_Status(0); 
                }
                //放件返回
                else if(Chariot->Get_Control_Status()==Chariot_Put_Cargo_Status)
                {
                    // 夹爪放件 
                    Put_Burry(Status[Now_Status_Serial].Time-1000);
                    if(Status[Now_Status_Serial].Time>5000)
                    Set_Status(0);                    
                }
            }
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

void Init_Position()
{
    // 默认先松开夹爪
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 800);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1200);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 450);
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 400);
}

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
