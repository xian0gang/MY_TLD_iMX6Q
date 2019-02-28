#ifndef __VIDEO_CHARSUPERIMPOSITION_H
#define __VIDEO_CHARSUPERIMPOSITION_H


//x,y为图像中的位置，Ppicture为图像指针，Resolution为图像分辨率，mode:1为填充，size为字体大小
void LCD_DrawPoint(unsigned short x, unsigned short y,unsigned char *Ppicture, unsigned short width);
void LCD_DrawPoint_Back(unsigned short x, unsigned short y,unsigned char *Ppicture, unsigned short width);

void LCD_DrawLine(unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2,unsigned char *Ppicture,unsigned int Resolution);
void LCD_DrawRectangle(unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2,unsigned char *Ppicture,unsigned int Resolution,unsigned char mode);
void LCD_Draw_Circle(unsigned short x0, unsigned short y0, unsigned char r, unsigned char* Ppicture, unsigned int Resolution);
void LCD_ShowChar(unsigned short x,unsigned short y,unsigned char num,unsigned char* Ppicture,unsigned short width,unsigned short height,unsigned char size);
unsigned long LCD_Pow(unsigned char m, unsigned char n);
void LCD_ShowNum(unsigned short x,unsigned short y,unsigned long num, unsigned char len,unsigned char *Ppicture, unsigned int Resolution, unsigned char size,unsigned char mode);
void LCD_ShowxNum(unsigned short x,unsigned short y,unsigned long num, unsigned char len,unsigned char *Ppicture, unsigned int Resolution, unsigned char size,unsigned char mode);
void LCD_ShowString(unsigned short x, unsigned short y, char *Pchar, unsigned char *Ppicture, unsigned int Resolution, unsigned char size);
void LCD_Show_Chinese_Char(unsigned short x,unsigned short y,unsigned char num,unsigned char* Ppicture,unsigned int Resolution,unsigned char size);

void DisPlay_ScaleBond_Fangwei(unsigned short x, unsigned short y, unsigned short fangwei, unsigned char *Ppicture);
void DisPlay_ScaleBond_Fuyang(unsigned short x, unsigned short y, unsigned short fuyang, unsigned char *Ppicture);
int DisPlay_Target_Bomen(unsigned short x, unsigned short y,unsigned char *Ppicture,unsigned int Resolution);


#endif
