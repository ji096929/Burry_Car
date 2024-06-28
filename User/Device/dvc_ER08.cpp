#include "dvc_ER08.h"
#include "string.h"



void Class_ER08::Init(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
    //    else if (huart->Instance == UART7)
    //    {
    //        UART_Manage_Object = &UART7_Manage_Object;
    //    }
    //    else if (huart->Instance == UART8)
    //    {
    //        UART_Manage_Object = &UART8_Manage_Object;
    //    }
	
  }

/**
 * @brief 数据处理过程
 *
 */
void Class_ER08::Data_Process()
{
    // 获取当前原始值数据
    memcpy(&Now_UART_Rx_Data, UART_Manage_Object->Rx_Buffer, 19 * sizeof(uint8_t));
    // 数据处理过程
    Struct_ER08_UART_Data *tmp_buffer = (Struct_ER08_UART_Data *)UART_Manage_Object->Rx_Buffer;

    x = tmp_buffer->x;
    y = tmp_buffer->y;
    
    memcpy(tel, tmp_buffer->tel, 11 * sizeof(uint8_t));
    memcpy(code, tmp_buffer->code,4 * sizeof(uint8_t));

    Updata_Flag = 1;
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_ER08::UART_RxCpltCallback(uint8_t *Rx_Data)
{

    Data_Process(); 
    
  
}