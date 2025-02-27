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
#define NEW_CAR
//#define OLD_CAR
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

// 1号 tim14 ch1  1000   	800
// 2号 tim13 ch1  1250 		400
// 3号 tim5  ch3  1550 		1200
// 4号 tim5  ch4  1250 		450

// 夹取 1 3 4 2
// 放回 2 4 3 1
// 舵机输出轴方向看过去，逆时针旋转
void Class_Chariot::Init_Position()
{
    // 默认先松开夹爪

    // Servo[0].Set_Angle(20);
    Servo[1].Set_Angle(10);
    // Servo[2].Set_Angle(60);
    // Servo[3].Set_Angle(30);
    //__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);

    // if (__time_cnt >= 450) // 抬起大臂 500ms
    // {
    //     Servo[1].Set_Angle(0);
    // }
    // if (__time_cnt >= 720) // 旋转yaw 500ms
    // {
    //     Servo[2].Set_Angle(0);
    // }
    // if (__time_cnt >= 920) // 折叠小臂 50ms
    // {
    //     Servo[3].Set_Angle(0);
    // }
}

float hi_angle = 45;
float low_angle = 15;
float delta = 0.01;
float test_angle_1 = -50;
float test_angle_2 = 0;
float test_angle_3 = 60;
float test_angle_4 = 30;
void Class_Chariot::Scan_Burry(uint16_t __time_cnt)
{
    static float delta_angle = hi_angle;

	  delta_angle -= delta;
    if (delta_angle > hi_angle || delta_angle < low_angle)
        delta *= -1.0f;
	#ifdef OLD_CAR
    Servo[0].Set_Angle(test_angle_1);
    Servo[1].Set_Angle(delta_angle);
    Servo[2].Set_Angle(60);
    Servo[3].Set_Angle(45);
    #elif defined(NEW_CAR)
    Servo[0].Set_Angle(-50);
    Servo[1].Set_Angle(delta_angle);
    Servo[2].Set_Angle(-40);
    #endif
}
// 有货夹取放车上
// 1 3 4 2
//  -5 -45 90 90

// 有货放置到货架上
// 2 4 3 1
//-45 30 -75 -30

// 无货去货架夹取 记得最后夹取
// 2 4 3 1
//-45 30 -75 -5

// 无货 取货回来放置 夹爪保持放置
// 1 2 3 4
//-5 初始 初始 初始

bool Class_Chariot::Burry_Input_Cargo_1(uint16_t __time_cnt)
{
    #ifdef OLD_CAR
    if (__time_cnt >= 50)
    {
        Servo[1].Set_Angle(-10);
    }
    if (__time_cnt >= 250)
    {
        Servo[0].Set_Angle(-5);
    }
    if (__time_cnt >= 560)
    {
        Servo[2].Set_Angle(-45);
    }
    if (__time_cnt >= 1060)
    {
        Servo[3].Set_Angle(90);
    }
    if (__time_cnt >= 1460)
    {
        Servo[1].Set_Angle(90);
        return true;
    }
    return false;
    #elif defined(NEW_CAR)
    if(__time_cnt >= 50)
    {
        Servo[0].Set_Angle(10); //夹子夹取
    }
    if(__time_cnt >= 260)
    {
        Servo[2].Set_Angle(40);
    }
    if(__time_cnt >= 460)
    {
        Servo[1].Set_Angle(-50);
        return true;
    }
    return false;
    #endif
}

bool Class_Chariot::Burry_Input_Cargo_2(uint16_t __time_cnt)
{
    #ifdef OLD_CAR
    if (__time_cnt >= 50)
    {
        Servo[1].Set_Angle(-45);
    }
    if (__time_cnt >= 260)
    {
        Servo[3].Set_Angle(45);
    }
    if (__time_cnt >= 660)
    {
        Servo[2].Set_Angle(-75);
    }
    if (__time_cnt >= 1560)
    {
        Servo[0].Set_Angle(-55);
        return true;
    }
    return false;
    #elif defined(NEW_CAR)
    if(__time_cnt >= 50)
    {
        Servo[2].Set_Angle(60);
    }
    if(__time_cnt >= 260)
    {
        Servo[1].Set_Angle(45);
    }
    if(__time_cnt >= 1060)
    {
        Servo[0].Set_Angle(-50); //最后缓慢松开夹爪
        return true;
    }
    return false;
    #endif
}

bool Class_Chariot::Burry_Output_Cargo_1(uint16_t __time_cnt)
{
    #ifdef OLD_CAR
    Servo[0].Set_Angle(-55);
    Servo[1].Set_Angle(-45);
    if(__time_cnt >= 400)
    {
        Servo[3].Set_Angle(45);
    }
    if(__time_cnt >= 1400)
    {
        Servo[2].Set_Angle(-75);
    }
    if(__time_cnt >= 2000)
    {
        Servo[0].Set_Angle(-5);
        return ture;
    }
    return false;
    #elif defined(NEW_CAR)
    if(__time_cnt < 1260)
    {
        Servo[0].Set_Angle(-50);//先默认打开夹爪
    }
    //Servo[0].Set_Angle(-50);  
    if(__time_cnt >= 100)
    {
        Servo[2].Set_Angle(60);
    }
    if(__time_cnt >= 660)
    {
        Servo[1].Set_Angle(45);
    }
    if(__time_cnt >= 1260)
    {
        Servo[0].Set_Angle(10);  //最后缓慢关闭夹爪
        return true;
    }
    return false;
    #endif
}

bool Class_Chariot::Burry_Output_Cargo_2(uint16_t __time_cnt)
{
    #ifdef OLD_CAR
    Servo[1].Set_Angle(-25);
    if(__time_cnt >= 400)
    {
            Servo[3].Set_Angle(45);
    }
    if(__time_cnt >= 1400)
    {
            Servo[2].Set_Angle(60);
    }
    if(__time_cnt >= 2000)
    {
            Servo[0].Set_Angle(-55);
            return true;
    }
    return false;
    #elif defined(NEW_CAR)
    if(__time_cnt >= 50)
    {
        Servo[1].Set_Angle(30);
    }
    if(__time_cnt >= 260)
    {
        Servo[2].Set_Angle(-40);
    }
    if(__time_cnt >= 1060)
    {
        Servo[0].Set_Angle(-50); //最后缓慢松开夹爪
        return true;
    }
    return false;
    #endif
}

bool Class_Chariot::Midlle_Position(uint16_t __time_cnt)
{
    #ifdef OLD_CAR
    if (__time_cnt >= 50)
    {
        Servo[2].Set_Angle(-45);
    }
    if (__time_cnt >= 360)
    {
       Servo[3].Set_Angle(90);
    }
    if (__time_cnt >= 660)
    {
        Servo[1].Set_Angle(90); 
        return true;
    }
    return false;
    #elif defined(NEW_CAR)
    if (__time_cnt >= 50)
    {
        Servo[2].Set_Angle(40);
    }
    if (__time_cnt >= 250)
    {
        Servo[1].Set_Angle(-60);
        return true;
    }
    return false;
    #endif
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

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init()
{
    ER08.Init(&huart1);
    Tjc011.Init(&huart2);

    // 舵机初始化
    #ifdef NEW_CAR

    Servo[0].Init(&htim14, TIM_CHANNEL_1);
    Servo[1].Init(&htim13, TIM_CHANNEL_1);
    Servo[2].Init(&htim5, TIM_CHANNEL_3);  

    #else if (OLD_CAR)
    {
        Servo[0].Init(&htim14, TIM_CHANNEL_1);
        Servo[1].Init(&htim13, TIM_CHANNEL_1);
        Servo[2].Init(&htim5, TIM_CHANNEL_3);
        Servo[3].Init(&htim5, TIM_CHANNEL_4);   

        Servo[0].Set_Angle(20);
        Servo[1].Set_Angle(0);
        Servo[2].Set_Angle(60);
        Servo[3].Set_Angle(30);         
    }
    #endif


    Chassis.IMU.Init();
    Chassis.Init();

    Cargo_List.Init();

    Now_Cargo_Number = 0;
    Control_Status = Chariot_Disable_Status; //控制状态
    //memset(&Now_Cargo, 0, 10 * sizeof(Struct_Cargo));
}

uint8_t Class_Chariot::Get_Cargo_Data()
{
    if(Cargo_List.Exist_Cargo((uint8_t *)ER08.Get_code())!=NULL)
    {
        return 0;
    }
    Cargo_List.Add_Cargo(ER08.Get_x(), ER08.Get_y(), (uint8_t *)ER08.Get_tel(), (uint8_t *)ER08.Get_code());
    Now_Cargo.Position_X = ER08.Get_x();
    Now_Cargo.Position_Y = ER08.Get_y();
    memcpy(Now_Cargo.Code, ER08.Get_code(), 4 * sizeof(uint8_t));
    memcpy(Now_Cargo.Phone_Number, ER08.Get_tel(), 11 * sizeof(uint8_t));
    Now_Cargo_Number++;
    return 1;
}

uint8_t Class_Chariot::Jundge_Cargo()
{
    Struct_Cargo *tmp = Cargo_List.Exist_Cargo((uint8_t *)Tjc011.Get_Input_Number());
    if(tmp!=NULL)
    {
        memcpy(&Now_Cargo, tmp, sizeof(Struct_Cargo));
        return 1;
    }
    return 0;
}

void Class_Chariot::Output_Cargo()
{
    Cargo_List.Delete_Cargo(Now_Cargo.Code);
    Now_Cargo_Number--;
    if(Now_Cargo_Number < 0)
    {
        Now_Cargo_Number = 0;
    }
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
void Light_Work_Control(uint16_t work_time)
{
    static int8_t flash_cnt = 1;
    if(work_time%150==0)
    {
        flash_cnt*=-1; 
    }
    if(flash_cnt == 1)
    {
        rgb_SetAllColor(YELLOW);
    }
    else if(flash_cnt == -1)
    {
        rgb_SetAllColor(BLACK);
    }
}


/**
 * @brief 有限状态机控制回调函数
 *
 */
uint16_t time_cnt = 0;
uint8_t false_flag = 0;
void Class_FSM_Chariot_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    static uint8_t light_nums = 0;
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
    case 0:
    {
        /*等待状态*/
        //rgb_SetAllColor(GREEN);   
        if(false_flag == 1)
        {
            light_nums = 0;
            static uint8_t false_nums = 0;
            if(Status[Now_Status_Serial].Time % 50 == 0)
            {
                false_nums++;
                if(false_nums%2 == 0)
                {
                    rgb_SetAllColor(RED);
                }
                else
                {
                    rgb_SetAllColor(BLACK);
                }
            }
            if(false_nums > 6)
            {
                false_flag = 0;
                false_nums = 0;
            }
        }
        else
        {
            if(Status[Now_Status_Serial].Time % 300 == 0)
            {   
                light_nums++;
                if(light_nums == 0)
                {
                    rgb_SetAllColor(BLACK);
                }
                if(light_nums == 11)
                {
                    light_nums = 0;
                    rgb_SetAllColor(BLACK);
                }
                RGB_GREEN(0,light_nums);
            }
        }
        // 夹爪初始位置
        Chariot->Scan_Burry(Status[Now_Status_Serial].Time);

        // 设置目标点 x:0 y:0
        Chariot->Chassis.Set_Target_Position_X(0);
        Chariot->Chassis.Set_Target_Position_Y(0);

        // 目前未收到任务
        Chariot->Set_Control_Status(Chariot_Disable_Status);

        // 获取货物信息 如果更新就获取信息 转到前往状态
        if (Chariot->ER08.Updata_Flag)
        {
            Chariot->ER08.Updata_Flag = 0;            
            //添加当前货物信息入链表  如果货物信息不重复
            if(Chariot->Get_Cargo_Data())
            {
                // 发送短信
                //Chariot->SIM900A.Sim900a_Send_Data((char *)Chariot->Now_Cargo.Code, (char *)Chariot->Now_Cargo.Phone_Number);
                Chariot->Set_Control_Status(Chariot_Input_Cargo_Status);
                Chariot->Servo[1].Set_Angle(-10);
                Set_Status(1);                
            }
        }

        // 获取取件码信息 如果更新就获取信息 转到取件状态
        if (Chariot->Tjc011.Input_data_Flag)
        {
            if (Chariot->Jundge_Cargo()) // 如果取件码正确 并且更新当前货物信息 转到前往状态
            {
                Chariot->Set_Control_Status(Chariot_Output_Cargo_Status);
                Set_Status(1);
            }
            else
            {
                false_flag = 1;
            }
            Chariot->Tjc011.Input_data_Flag = 0;
        }
    }
    break;
    case 1:
        /*导航前往状态*/
        Light_Work_Control(Status[Now_Status_Serial].Time);  
        // 夹取货物
        Chariot->Burry_Input_Cargo_1(Status[Now_Status_Serial].Time);

        // 夹取完成后延时1s
        if (Status[Now_Status_Serial].Time >= 4000)
        {
            // 设置目标点
            Chariot->Chassis.Set_Target_Position_X(-1.0f*(Chariot->Now_Cargo.Position_X-48));
            Chariot->Chassis.Set_Target_Position_Y(-1.0f*(Chariot->Now_Cargo.Position_Y-48));          
            Chariot->Chassis.Set_Target_Angle(0);

            // 到达目标点 跳转到下一个状态
            if (fabs(Chariot->Chassis.Get_Now_Position_X() - Chariot->Chassis.Get_Target_Position_X()) < 0.02 &&
                (fabs(Chariot->Chassis.Get_Now_Position_Y() - Chariot->Chassis.Get_Target_Position_Y()) < 0.02))
            {
                Set_Status(2);
            }
            // if(Status[Now_Status_Serial].Time > 10000)
            // {
            //     Set_Status(2);
            // }
        }
        break;
    case 2:
        /*放件/取件状态*/
        Light_Work_Control(Status[Now_Status_Serial].Time);   
        // 先延时1s
        if (Status[Now_Status_Serial].Time >= 4000)
        {
            // 取件
            if (Chariot->Get_Control_Status() == Chariot_Output_Cargo_Status)
            {
                // 夹取货柜的货物
                Chariot->Burry_Output_Cargo_1(Status[Now_Status_Serial].Time - 4000);
            }
            // 放件
            else if (Chariot->Get_Control_Status() == Chariot_Input_Cargo_Status)
            {
                // 放置货物
                Chariot->Burry_Input_Cargo_2(Status[Now_Status_Serial].Time - 4000);
            }
        }
        // 整个操作5s后返回
        if (Status[Now_Status_Serial].Time > 8000)
        {
            Set_Status(3);
        }
        break;
    case 3:
        /*导航返回状态*/
        Light_Work_Control(Status[Now_Status_Serial].Time);     
	    Chariot->Midlle_Position(Status[Now_Status_Serial].Time);

        if(Status[Now_Status_Serial].Time > 2000)
        {
            // 设置目标点
            Chariot->Chassis.Set_Target_Position_X(0);
            Chariot->Chassis.Set_Target_Position_Y(0);
            Chariot->Chassis.Set_Target_Angle(0);
            // 到达目标点 跳转到下一个状态
            if (fabs(Chariot->Chassis.Get_Now_Position_X() - Chariot->Chassis.Get_Target_Position_X()) < 0.02 &&
                (fabs(Chariot->Chassis.Get_Now_Position_Y() - Chariot->Chassis.Get_Target_Position_Y()) < 0.02))
            {
                Set_Status(4);
            }
        }
        break;

    case 4:
        /*返回类型判断状态*/
        light_nums = 0;
        Light_Work_Control(Status[Now_Status_Serial].Time); 
        // 先延时1s
        if (Status[Now_Status_Serial].Time >= 2000)
        {
            // 取件返回
            if (Chariot->Get_Control_Status() == Chariot_Output_Cargo_Status)
            {
                //链表删除当前货物信息
                Chariot->Output_Cargo();
                // 返回初始状态
                Chariot->Burry_Output_Cargo_2(Status[Now_Status_Serial].Time - 2000);
                if (Status[Now_Status_Serial].Time > 5000)
                {
                    Set_Status(0);                // 返回初始状态
                    rgb_SetAllColor(BLACK);
                }
                    
            }
            // 放件返回
            else if (Chariot->Get_Control_Status() == Chariot_Input_Cargo_Status)
            {
                Set_Status(0);
                rgb_SetAllColor(BLACK);
            }
        }
        break;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
