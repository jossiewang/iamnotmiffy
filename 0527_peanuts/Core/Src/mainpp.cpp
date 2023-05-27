/*
 * mainpp.cpp
 *
 *  Created on: Mar 23, 2023
 *      Author: kch93
 */
#include "mainpp.h"
#include "ros.h"
#include "STM32Hardware.h"
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;
int n=0;
geometry_msgs::Twist speed;
ros::Publisher pub("speed_fromSTM",&speed);
void vel_callback(const geometry_msgs::Twist &msg)
{
	Vx = msg.linear.x;
	Vy = msg.linear.y;
	W=msg.angular.z;
}

void realspeed()
{
	speed.linear.x=rVx;
	speed.linear.y=rVy;
	speed.angular.z=rW;
	pub.publish(&speed);
	n++;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmdvel_toSTM",vel_callback);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 {
nh.getHardware()->flush();
 }
 //void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 //{
 //nh.getHardware()->reset_rbuf();
 //}
 void setup(void)
 {
 nh.initNode();
 nh.subscribe(sub);
 nh.advertise(pub);
 }
 void loop(void)
 {
 //pub.publish(&speed);
 nh.spinOnce();
 }

 /* UART Communication */
 void Error_Handler(void)
 {
   /* USER CODE BEGIN Error_Handler_Debug */
   /* User can add his own implementation to report the HAL error return state */
   __disable_irq();
   while (1)
   {
   }
   /* USER CODE END Error_Handler_Debug */
 }

 static void MX_USART3_UART_Init(void)
 {

   /* USER CODE BEGIN USART3_Init 0 */

   /* USER CODE END USART3_Init 0 */

   /* USER CODE BEGIN USART3_Init 1 */

   /* USER CODE END USART3_Init 1 */
   huart3.Instance = USART3;
   huart3.Init.BaudRate = 57600;
   huart3.Init.WordLength = UART_WORDLENGTH_8B;
   huart3.Init.StopBits = UART_STOPBITS_1;
   huart3.Init.Parity = UART_PARITY_NONE;
   huart3.Init.Mode = UART_MODE_TX_RX;
   huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart3.Init.OverSampling = UART_OVERSAMPLING_16;
   huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
   huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
   huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
   if (HAL_UART_Init(&huart3) != HAL_OK)
   {
     Error_Handler();
   }
   if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
   {
     Error_Handler();
   }
   if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
   {
     Error_Handler();
   }
   if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
   {
     Error_Handler();
   }
   /* USER CODE BEGIN USART3_Init 2 */

   /* USER CODE END USART3_Init 2 */

 }
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 {
     if (huart == &huart3)
     {
         rVx = 0.0;
         rVy = 0.0;
         rW = 0.0;
         HAL_UART_DeInit(&huart3);
         MX_USART3_UART_Init();
         nh.getHardware()->init();
     }
 }
