#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"

void xrunstep(int dxstep); 
void yrunstep(int dxstep);

void multi_angle_control(float angle1, float angle2, float angle3);

void read_angle_1(void);
void read_angle_2(void);


#endif
