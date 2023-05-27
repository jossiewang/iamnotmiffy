/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

//extern float x,y,theta;
extern double Vx,Vy,W, rVx, rVy, rW;
void setup(void);
void loop(void);
void realspeed(void);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
