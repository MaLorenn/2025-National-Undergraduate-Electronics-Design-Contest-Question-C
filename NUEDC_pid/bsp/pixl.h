#ifndef _PIXL_H_
#define _PIXL_H_

#include "main.h"

#define DIS_X 505			// 屏幕分辨率是640  480  面积是29.7cm*21cm 假设云台到屏幕距离是100，那么此处的值是100/29.7*640
#define DIS_Y 100

#define M_PI 3.141592 

void Move_pixl_Coord(int x_dat, int y_dat);

void Tripod_Draw_Circle(uint16_t X_coord, uint16_t Y_coord, uint16_t Circle_center);
void Draw_Sine_Wave(int centerX, int centerY, int amplitude, int wavelength, int numPoints);

#endif
