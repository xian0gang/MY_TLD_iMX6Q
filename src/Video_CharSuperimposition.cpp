#include "stdlib.h"
#include "Font.h" 
#include <string.h>
#include <stdio.h>
#include "Video_CharSuperimposition.h"


//******************************************************************	 
//画点
//x,y:坐标
//Ppicture:图像指针
//width：图像宽度
//******************************************************************

void LCD_DrawPoint(unsigned short x, unsigned short y,unsigned char *Ppicture, unsigned short width)
{
	#if 0
	//白点 Gray灰度图
		Ppicture[y*width+x] = 0xFF;
	#else
	//绿点 NV12
		Ppicture[y*width+x] = 0xFF;
		if(width == 1920)
		{
			Ppicture[1920*1080 + y/2*width + x/2*2] = 0x36;
			Ppicture[1920*1080 + y/2*width + x/2*2 + 1] = 0x22;
		}
		else if(width == 720)
		{
			Ppicture[720*576 + y/2*width + x/2*2] = 0x36;
			Ppicture[720*576 + y/2*width + x/2*2 + 1] = 0x22;
		}
		else if(width == 960)
		{
			Ppicture[960*544 + y/2*width + x/2*2] = 0x36;
			Ppicture[960*544 + y/2*width + x/2*2 + 1] = 0x22;
		}
		else if(width == 640)
		{
			Ppicture[640*512+ y/2*width + x/2*2] = 0x36;
			Ppicture[640*512 + y/2*width + x/2*2 + 1] = 0x22;
		}
		else
			Ppicture[y*width+x] = 0xFF;
	#endif
}	

//******************************************************************	 
//画点
//x,y:坐标
//Ppicture:图像指针
//width：图像宽度
//******************************************************************

void LCD_DrawPoint_Back(unsigned short x, unsigned short y,unsigned char *Ppicture, unsigned short width)
{
	Ppicture[y*width+x] = 0x40;
}	

//******************************************************************
//画线
//x1,y1:起点坐标
//x2,y2:终点坐标  
//Ppicture:图像指针
//Resolution：图像分辨率
//******************************************************************
void LCD_DrawLine(unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2,unsigned char *Ppicture,unsigned int Resolution)
{
	unsigned short t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	unsigned short width = 1920,height = 1080;
	switch(Resolution)                                         //判断具体图像分辨率
	{
		case 1080: width = 1920;height = 1080; break;
		case 544: width = 960; height = 544;  break;
		case 576: width = 720; height = 576;  break;
		case 512: width = 640;height = 512;  break;
		default: break;
	}
	if((x1>=width)||(x2>=width)||(y1>=height)||(y2>=height))   //判断边界问题
	{
		printf("LCD_DrawLine 划线超过边界***********\n");
		return ;
	}
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)
		incx=1; //设置单步方向 
	else if(delta_x==0)
		incx=0;//垂直线 
	else 
	{
		incx=-1;
		delta_x=-delta_x;
	} 
	if(delta_y>0)
		incy=1; 
	else if(delta_y==0)
		incy=0;//水平线 
	else
	{
		incy=-1;
		delta_y=-delta_y;
	} 
	if( delta_x>delta_y)
		distance=delta_x; //选取基本增量坐标轴 
	else 
		distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		LCD_DrawPoint(uRow,uCol,Ppicture,width);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}    

//******************************************************************
//画矩形	  
//(x1,y1),(x2,y2):矩形的对角坐标
//Ppicture:图像指针
//Resolution：图像分辨率
//mode:1为实心矩形框，0为矩形边框
//******************************************************************
void LCD_DrawRectangle(unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2,unsigned char *Ppicture,unsigned int Resolution,unsigned char mode)
{
	if(mode)                 //填充矩形区域
	{
		int i;
		unsigned short x = 0;
		if(x1>x2)
		{
			x = x1-x2;
			for(i=0;i<x;i++)
				LCD_DrawLine(x2+i,y1,x2+i,y2,Ppicture,Resolution);
		}
		else
		{
			x = x2-x1;
			for(i=0;i<=x;i++)
				LCD_DrawLine(x1+i,y1,x1+i,y2,Ppicture,Resolution);
		}
	}
	else
	{
		LCD_DrawLine(x1,y1,x2,y1,Ppicture,Resolution);
		LCD_DrawLine(x1,y1,x1,y2,Ppicture,Resolution);
		LCD_DrawLine(x1,y2,x2,y2,Ppicture,Resolution);
		LCD_DrawLine(x2,y1,x2,y2,Ppicture,Resolution);
	}
}


//******************************************************************
//在指定位置画一个指定大小的圆
//(x,y):中心点
//r    :半径
//Ppicture:图像指针
//Resolution：图像分辨率
//******************************************************************
void LCD_Draw_Circle(unsigned short x0, unsigned short y0, unsigned char r, unsigned char* Ppicture, unsigned int Resolution)
{
	int a,b;
	int di;
	unsigned short width = 1920,height = 1080;

	switch(Resolution)                                         //判断具体图像分辨率
	{
		case 1080: width = 1920;height = 1080; break;
		case 544: width = 960; height = 544;  break;
		case 576: width = 720; height = 576;  break;
		case 512: width = 640;height = 512;  break;
		default: break;
	}
	
	if((x0+r)>=width || (x0-r)<0 || (y0-r)<0 || (y0+r)>=height)
	{
		printf("LCD_Draw_Circle划圆超过边界***********\n");
		return;
	}
	a=0;b=r;	  
	di=3-(r<<1);             //判断下个点位置的标志
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b,Ppicture,width);             //5
 		LCD_DrawPoint(x0+b,y0-a,Ppicture,width);             //0           
		LCD_DrawPoint(x0+b,y0+a,Ppicture,width);             //4               
		LCD_DrawPoint(x0+a,y0+b,Ppicture,width);             //6 
		LCD_DrawPoint(x0-a,y0+b,Ppicture,width);             //1       
 		LCD_DrawPoint(x0-b,y0+a,Ppicture,width);             
		LCD_DrawPoint(x0-a,y0-b,Ppicture,width);             //2             
  		LCD_DrawPoint(x0-b,y0-a,Ppicture,width);             //7     	         
		a++;
		//使用Bresenham算法画圆     
		if(di<0)
			di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
} 		

//******************************************************************
//在指定位置显示一个字符
//x,y:起始坐标
//num:要显示的字符:" "--->"~"
//size:字体大小 12/24/36
//******************************************************************
void LCD_ShowChar(unsigned short x,unsigned short y,unsigned char num,unsigned char* Ppicture,unsigned short width,unsigned short height,unsigned char size)
{  							  
    unsigned char temp,t1,t;
	unsigned short x0;
	x0 = x ;
	unsigned char csize=((size/2)/8+(((size/2)%8)?1:0))*size;		//得到字体一个字符对应点阵集所占的字节数	
 	num=num-' ';//得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）
 	if((y+size<height) && ((x+size/2)<width))   //判断是否字符超出边界
 	{
		for(t=0; t<csize; t++)
		{   
			if(size==12)
				temp=ASCII_1206[num][t];    //调用1206字体
			else if(size==16)
				temp=ASCII_1608[num][t];	//调用1608字体
			else if(size==24)
				temp=ASCII_2412[num][t];	//调用2412字体
			else if(size==32)
				temp=ASCII_3216[num][t];	//调用3216字体
			else
			{
				printf("LCD_ShowChar没有相关 英文 字库**********\n");
				return;						//没有的字库	
			}
			for(t1=0; t1<8; t1++)
			{		
				if(temp&0x80)              //只叠字符部分,背景不叠
					LCD_DrawPoint(x0,y,Ppicture,width);
				//else
					//LCD_DrawPoint_Back(x0,y,Ppicture,width);
				temp<<=1;
				x0++;	
			}  
			if((x0-x)==8*((size/2)/8+(((size/2)%8)?1:0)))        //判断一行结束转到下一行
			{
				x0 = x;
				y++;
			}
		} 
 	}
	else
		printf("LCD_ShowChar叠加ASCII字符超出边界 *******%c******\n",num);
}   


//m^n函数
//返回值:m^n次方.
unsigned long LCD_Pow(unsigned char m, unsigned char n)
{
	unsigned long result=1;	 
	while(n--)
		result*=m;    
	return result;
}

//显示数字,mode = 1.高位为0,则显示
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//color:颜色 
//num:数值(0~4294967295);	

void LCD_ShowNum(unsigned short x,unsigned short y,unsigned long num, unsigned char len,unsigned char *Ppicture, unsigned int Resolution, unsigned char size ,unsigned char mode)
{         	
	unsigned char t,temp;
	unsigned char enshow=0;		
	unsigned short width = 1920,height = 1080;
	
	switch(Resolution)                                         //判断具体图像分辨率
	{
		case 1080: width = 1920;height = 1080; break;
		case 544: width = 960; height = 544;  break;
		case 576: width = 720; height = 576;  break;
		case 512: width = 640;height = 512;  break;
		default: break;
	}				   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				if(mode)
					LCD_ShowChar(x+(size/2)*t,y,'0',Ppicture,width,height,size);  
				else
					LCD_ShowChar(x+(size/2)*t,y,' ',Ppicture,width,height,size);   
 				continue;
			}
			else 
				enshow=1;
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',Ppicture,width,height,size);
	}
} 

//******************************************************************
//显示数字,高位为0,还是显示
//x,y:起点坐标
//num:数值(0~999999999);	 
//len:长度(即要显示的位数)
//size:字体大小
//mode：1高位补零。0高位缺省
//Ppicture:原始图像
//Resolution:图像分辨率
//******************************************************************
void LCD_ShowxNum(unsigned short x,unsigned short y,unsigned long num, unsigned char len,unsigned char *Ppicture, unsigned int Resolution, unsigned char size,unsigned char mode)
{  
	unsigned char t,temp;
	unsigned char enshow=0;		
	unsigned short width = 1920,height = 1080;
	switch(Resolution)                                         //判断具体图像分辨率
	{
		case 1080: width = 1920;height = 1080; break;
		case 544: width = 960; height = 544;  break;
		case 576: width = 720; height = 576;  break;
		case 512: width = 640;height = 512;  break;
		default: break;
	}
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				if(mode)
					LCD_ShowChar(x+(size/2)*t,y,'0',Ppicture,width,height,size);  
				else
					LCD_ShowChar(x+(size/2)*t,y,' ',Ppicture,width,height,size);   
 				continue;
			}
			else 
				enshow=1;
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',Ppicture,width,height,size);
	}
} 


//******************************************************************
//显示字符串
//Horizon = x,  Vertical = y:起点坐标 
//size:字体大小
//*Pchar:字符串起始地址		
//Resolution 图像分辨率
//******************************************************************
void LCD_ShowString(unsigned short x, unsigned short y, char *Pchar, unsigned char *Ppicture, unsigned int Resolution, unsigned char size)
{         
	unsigned short width = 1920,height = 1080;
	switch(Resolution)                                         //判断具体图像分辨率
	{
		case 1080: width = 1920;height = 1080; break;
		case 544: width = 960; height = 544;  break;
		case 576: width = 720; height = 576;  break;
		case 512: width = 640;height = 512;  break;
		default: break;
	}
    while((*Pchar<='~')&&(*Pchar>=' '))//判断是不是非法字符!
    {       
        LCD_ShowChar(x,y,*Pchar,Ppicture,width,height,size);
        x+=size/2;
        Pchar++;
    }  
}


//汉字叠加
//******************************************************************
//在指定位置显示一个字符
//x,y:起始坐标
//num:要显示的字符:" "--->"~"
//size:字体大小 12/24/36
//******************************************************************
void LCD_Show_Chinese_Char(unsigned short x,unsigned short y,unsigned char num,unsigned char* Ppicture,unsigned int Resolution,unsigned char size)
{  							  
    unsigned char temp,t1,t;
	unsigned short x0;
	x0 = x;
	unsigned char csize=(size/8+((size%8)?1:0))*size;		//得到字体一个字符对应点阵集所占的字节数	
	unsigned short width = 1920,height = 1080;
	switch(Resolution)                                         //判断具体图像分辨率
	{
		case 1080: width = 1920;height = 1080; break;
		case 544: width = 960; height = 544;  break;
		case 576: width = 720; height = 576;  break;
		case 512: width = 640;height = 512;  break;
		default: break;
	}
 	if((x+size<width) && ((y+size)<height))   //判断是否字符超出边界
 	{
		for(t=0; t<csize; t++)
		{   
			if(size==16)
				temp=Chinese_Char16x16[num][t];    //调用24x24字体
			else if(size==24)
				temp=Chinese_Char24x24[num][t];    //调用24x24字体
			else if(size==32)
				temp=Chinese_Char32x32[num][t];	   //调用32x32字体
			else
			{
				printf("LCD_Show_Chinese_Char没有相关中文字库**********\n");
				return;						//没有的字库
			}
			for(t1=0; t1<8; t1++)
			{		
				if(temp&0x80)              //只叠字符部分,背景不叠
					LCD_DrawPoint(x0,y,Ppicture,width);
				//else
					//LCD_DrawPoint_Back(x0,y,Ppicture,width);
				temp<<=1;
				x0++;	
			} 
			if((x0-x) == size)           //判断一行结束转到下一行
			{
				x0 = x;
				y++;
			}
		} 
 	}
	else
		printf("LCD_Show_Chinese_Char叠加中文字符超出边界*******\n");
} 


//***********************************************
//1080P高清图像刻度带叠加
//***********************************************
void DisPlay_ScaleBond_Fangwei(unsigned short x, unsigned short y, unsigned short fangwei, unsigned char *Ppicture)
{
	unsigned char temp;
	unsigned short x0;
	unsigned short i,j,t1;
	unsigned short fangwei_Char = 0;
	unsigned char fangwei_Bit = 0;
	fangwei_Char = fangwei/8;         
	fangwei_Bit = fangwei%8;
	for(i=0; i<100; i++)   
	{
		x0 = x;              //重新回到行首位置
		//**********************************************************
		for(j=0;j<1;j++)                    //字符前半部分
		{
			temp = Fangwei_Scaleband[i*500 + fangwei_Char-30 +j];
			for(t1=fangwei_Bit; t1<8; t1++)
			{		 
				if((temp>>t1) & 0x01)              //只叠字符部分,背景不叠
					LCD_DrawPoint(x0,y,Ppicture,1920);
				x0++;
			}
		}
		for(j=1; j<60; j++)  //刻度带宽度
		{
			temp = Fangwei_Scaleband[i*500 + fangwei_Char-30 +j];
			for(t1=0; t1<8; t1++)
			{		
				if(temp & 0x01)              //只叠字符部分,背景不叠
					LCD_DrawPoint(x0,y,Ppicture,1920);
				temp >>= 1;
				x0++;
			}
		}
		for(j=60;j<61;j++)                   //字符后半部分
		{
			temp = Fangwei_Scaleband[i*500 + fangwei_Char-30 +j];
			for(t1=0; t1<fangwei_Bit; t1++)
			{		
				if(temp & 0x01)              //只叠字符部分,背景不叠
					LCD_DrawPoint(x0,y,Ppicture,1920);
				temp >>= 1;
				x0++;
			}
		}
		y++;
	}
}

void DisPlay_ScaleBond_Fuyang(unsigned short x, unsigned short y, unsigned short fuyang, unsigned char *Ppicture)
{
	unsigned char temp;
	unsigned short x0;
	unsigned short i,j,t1;
	
	for(i=fuyang-100; i<fuyang+100; i++) //俯仰行数
	{
		x0 = x;
		for(j=0; j<12; j++)  //列
		{
			temp = Fuyang_Scaleband[i*12 + j];
			for(t1=0; t1<8; t1++)
			{		
				if(temp & 0x01)              //只叠字符部分,背景不叠
					LCD_DrawPoint(x0,y,Ppicture,1920);
				temp >>= 1;
				x0++;
			}
		}
		y++;
	}
}

int DisPlay_Target_Bomen(unsigned short x, unsigned short y,unsigned char *Ppicture,unsigned int Resolution)
{
	unsigned short width = 1920,height = 1080,bomen = 16;
	switch(Resolution)                                         //判断具体图像分辨率
	{
		case 1080: width = 1920; height = 1080; bomen = 32; break;
		case 544: width = 960; height = 544; bomen = 16;  break;
		case 576: width = 720; height = 576; bomen = 16;  break;
		case 512: width = 640; height = 512; bomen = 16;  break;
		default: break;
	}
	if((x>width)||(y>height))
	{
		printf("Target station beyond the size of Picture **********\n");
		return -1;
	}
	//判断目标位置*********************************
	if(x < bomen)
		x = bomen;
	if(x > (width-1 - bomen))
		x = width-1 - bomen;
	if(y < bomen)
		y = bomen;
	if(y > (height-1 - bomen))
		y = height-1 - bomen;
	//*********************************************
	//上沿线
	LCD_DrawLine(x-bomen, y-bomen, x-bomen/2, y-bomen,Ppicture,Resolution);
	LCD_DrawLine(x+bomen/2, y-bomen, x+bomen, y-bomen,Ppicture,Resolution);
	//下沿线
	LCD_DrawLine(x-bomen, y+bomen, x-bomen/2, y+bomen,Ppicture,Resolution);
	LCD_DrawLine(x+bomen/2, y+bomen, x+bomen, y+bomen,Ppicture,Resolution);
	//左沿线
	LCD_DrawLine(x-bomen, y-bomen, x-bomen, y-bomen/2,Ppicture,Resolution);
	LCD_DrawLine(x-bomen, y+bomen/2, x-bomen, y+bomen,Ppicture,Resolution);
	//右沿线
	LCD_DrawLine(x+bomen, y-bomen, x+bomen, y-bomen/2,Ppicture,Resolution);
	LCD_DrawLine(x+bomen, y+bomen/2, x+bomen, y+bomen,Ppicture,Resolution);

	//上飞线
	if((y < (bomen+bomen/2))&&(y>=bomen))
		LCD_DrawLine(x, 0, x, y-bomen,Ppicture,Resolution);
	else
		LCD_DrawLine(x, y-bomen-bomen/2, x, y-bomen,Ppicture,Resolution);
	//右飞线
	if((y > (height-1 - bomen - bomen/2))&&(y <= (height-1 - bomen)))
		LCD_DrawLine(x, y+bomen, x, height-1,Ppicture,Resolution);
	else
		LCD_DrawLine(x, y+bomen, x, y+bomen+bomen/2,Ppicture,Resolution);

	//左飞线
	if((x < (bomen+bomen/2))&&(x>=bomen))
		LCD_DrawLine(0, y, x-bomen, y,Ppicture,Resolution);
	else
		LCD_DrawLine(x-bomen-bomen/2, y, x-bomen, y,Ppicture,Resolution);
	//右飞线
	if((x > (width-1 - bomen - bomen/2))&&(x <= (width-1 - bomen)))
		LCD_DrawLine(x+bomen, y, width-1, y,Ppicture,Resolution);
	else
		LCD_DrawLine(x+bomen, y, x+bomen+bomen/2, y,Ppicture,Resolution);
	return 0;
}


