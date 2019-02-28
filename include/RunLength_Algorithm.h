#ifndef RunLength_Algorithm_h
#define RunLength_Algorithm_h

#include <stdio.h>
#include "math.h"
#include "string.h"
#include "stdlib.h"

typedef struct
{
	float s_Area;
	float s_PeakLight;
	float s_MaxHeight;
	float s_MaxWidth ;
	float s_MeanLight;
	//float s_StdLight_Big;
	//float s_StdLight_Small;
	int s_C_Row;
	int s_C_Col;
	//float s_EntryBi;
    short	  s_C_fangweiPixel;			//存放像素值
	short	  s_C_fuyangPixel;			//存放像素值
	unsigned short     Frame_Code;

}TARFEATURE;


typedef struct
{
	short Row;
	short CStart;
	short CEnd;
	short Label;
	short MaxGray;
	short MeanGray;
}RUNLENGTHCODE;

//目标游程检测
void RunLengthDetect(int search_x,int search_y,int search_width,int search_height,unsigned char * mask);

void AutoTarDetect(unsigned short StartRow,unsigned short EndRow,unsigned short StartCol,unsigned short EndCol);

int Max_To_Min_TargetArea(void);

void Maxarea_target(unsigned short * tar_x, unsigned short * tar_y);

void Init_RunLengthParameter(void);
void UnInit_RunLengthParameter(void);



#endif

