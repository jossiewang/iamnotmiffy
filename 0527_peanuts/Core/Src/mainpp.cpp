/*
 * mainpp.cpp
 *
 *  Created on: Mar 23, 2023
 *      Author: kch93
 */
#include "mainpp.h"
//#include "main.c"
#include "ros.h"
#include "stm32h7xx_hal.h"
#include "geometry_msgs/Twist.h"
float Vx, Vy, W;
float rVx, rVy, rW;
void vel_callback(const geometry_msgs::Twist &msg)
{
	Vx = msg.linear.x;
	Vy = msg.linear.y;
	W=msg.angular.z;
}
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmdvel_toSTM", vel_callback);
geometry_msgs::Twist speed;
ros::Publisher pub("speed_fromSTM",&speed);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    nh.getHardware()->flush();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    nh.getHardware()->reset_rbuf();
}
void setup(void)
{
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);
}
void loop(void)
{
    nh.spinOnce();
}
void errcallback(void) {
	nh.getHardware()->init();
}

void realspeed(void)
{
	geometry_msgs::Twist speed_;
	speed_.linear.x=rVx;
	speed_.linear.y=rVy;
	speed_.angular.z=rW;

	static bool flag = false;
	if(flag) pub.publish(&speed_);
	flag = true;
}


//void inverse_kinematics_model();
//void Encoder();
//void PID_PWM();
//void kinematics_model();

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim->Instance == TIM3){
//		inverse_kinematics_model();
//		Encoder();
//		PID_PWM();
//		kinematics_model();
//
//	}
//	if(htim->Instance == TIM5){
//		//rVx = 1;
//		//rVy = 1;
//		//rW = 1;
//		realspeed();
////		nnn++;
//	}
//}
