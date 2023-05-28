/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
extern "C"
{
#endif

void setup(void);
void loop(void);
void errcallback(void);
extern double Vx, Vy, W;

#ifdef __cplusplus
}
#endif

#endif /* MAINPP_H_ */
