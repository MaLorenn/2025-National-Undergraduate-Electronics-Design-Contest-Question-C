#include "pixl.h"
#include "motor.h"
#include <math.h>


uint16_t X_OFFSET = 75;
uint16_t Y_OFFSET = 50;
int record_pixl[2] = {75,50}; 
//x y 轴坐标动作   到达指定坐标
void Move_pixl_Coord(int x_dat, int y_dat)
{
	float temp_x;
	float temp_y;
	float x_len;
	
	temp_x = (float)x_dat;
	temp_x =  X_OFFSET-temp_x;
	temp_x = atan2f(temp_x,DIS_X);
	temp_x = temp_x * 57.29577f;  //弧度转角度
	
	temp_y = (float)y_dat;//
	temp_y = temp_y - Y_OFFSET ;//
	x_len = hypot((record_pixl[0] - X_OFFSET), DIS_X);  //旋转角度矫正
	temp_y = atan2f(temp_y,x_len);
	temp_y = temp_y * 57.29577f;

	multi_angle_control(temp_x,temp_y,0);
	
	record_pixl[0] = x_dat;
	record_pixl[1] = y_dat;
}

//激光笔画圆
void Tripod_Draw_Circle(uint16_t X_coord, uint16_t Y_coord, uint16_t Circle_center)
{
	float num_cos[15] = {1.000 ,0.995 ,0.978 ,0.951 ,0.914 ,0.866 ,0.809 ,0.743 ,0.669 ,0.588 ,0.500 ,0.407 ,0.309 ,0.208 ,0.105};
	float num_sin[15] = {0.000 ,0.105 ,0.208 ,0.309 ,0.407 ,0.500 ,0.588 ,0.669 ,0.743 ,0.809 ,0.866 ,0.914 ,0.951 ,0.978 ,0.995};
	int coord[120];
	int i;
	
	for(i = 0;i < 15;i ++)
	{
		coord[i*2] = X_coord - Circle_center*num_cos[i];
		coord[i*2+1] = Y_coord + Circle_center*num_sin[i];
	}
	
	for(i = 0;i < 15;i ++)
	{
		coord[30+(i*2)] = X_coord + Circle_center*num_sin[i];
		coord[30+(i*2+1)] = Y_coord + Circle_center*num_cos[i];
	}
	
	for(i = 0;i < 15;i ++)
	{
		coord[60+(i*2)] = X_coord + Circle_center*num_cos[i];
		coord[60+(i*2+1)] = Y_coord - Circle_center*num_sin[i];
	}
	
	for(i = 0;i < 15;i ++)
	{
		coord[90+(i*2)] = X_coord - Circle_center*num_sin[i];
		coord[90+(i*2+1)] = Y_coord - Circle_center*num_cos[i];
	}
	
	for(i = 0;i < 60;i ++)
	{

		Move_pixl_Coord(coord[i*2],coord[i*2+1]);
	}
	
}



/*
 * 参数：
 * centerX：正弦波的X中心坐标
 * centerY：正弦波的Y中心坐标
 * amplitude：振幅
 * wavelength：波长
 * numPoints：点的数量
 * 返回值：无
 */
void Draw_Sine_Wave(int centerX, int centerY, int amplitude, int wavelength, int numPoints) 
{
    double xScale = (double)wavelength / numPoints; // X轴上的缩放因子
    double phaseShift = 0.0; // 相位移动，可以设置为任意值来改变波的起始位置
 
    for (int i = 0; i < numPoints; i++) 
    {
        double x = i * xScale;
        double y = centerY + amplitude * sin(2 * M_PI * x / wavelength + phaseShift);
 
        int intX = (int)(centerX + x); // 将x坐标转换为整数
        int intY = (int)(y); // 将y坐标转换为整数，注意这里可能需要进行一些舍入处理
 
        // 移动激光笔到计算出的坐标点
		Move_pixl_Coord(intX,intY);
    }
}
