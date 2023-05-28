/*
 * mainpp.cpp
 *
 *  Created on: Mar 23, 2023
 *      Author: kch93
 */
#include "mainpp.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"

void vel_callback(const geometry_msgs::Twist &msg)
{
	Vx = msg.linear.x;
	Vy = msg.linear.y;
	W=msg.angular.z;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmdvel_toSTM", vel_callback);
float Vx, Vy, W;
float rVx, rVy, rW;

geometry_msgs::Twist speed;
ros::Publisher pub("speed_toSTM",&speed);

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
	speed.linear.x=rVx;
	speed.linear.y=rVy;
	speed.angular.z=rW;
	pub.publish(&speed);
}
