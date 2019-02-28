#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "File_Queue.h"
#include "Init_dev.h"
#include "Communication.h"


#pragma pack(1)

#define PI 3.1415926

//**********************************************************************
#define BUF_MAX2 10              //接收长度
#define BUF_MAX1 64              //接收长度

SqQueue         exPayLoadRXFrameBuff2;
SqQueue         exPayLoadRXFrameBuff1;
char            TaskRx2_DATA[BUF_MAX2];
char            TaskRx1_DATA[BUF_MAX1];
char            PL_exPayLoadRXFrameBuff2[2*BUF_MAX2];
char            PL_exPayLoadRXFrameBuff1[2*BUF_MAX1];
//**********************************************************************


void PayLoad_Init1(void) 
{
	exPayLoadRXFrameBuff1.base=PL_exPayLoadRXFrameBuff1;
	QueueInit(&exPayLoadRXFrameBuff1, 2*sizeof(PL_exPayLoadRXFrameBuff1));
} 

void PayLoad_Init2(void)
{
	exPayLoadRXFrameBuff2.base=PL_exPayLoadRXFrameBuff2;
	QueueInit(&exPayLoadRXFrameBuff2, 2*sizeof(PL_exPayLoadRXFrameBuff2));
}

//***************跟踪目标相关参数****************************
extern unsigned char mubiaodiushi;
extern unsigned char PAL_SDI_Track;
extern unsigned char Track_auto_flag ;  //跟踪手动标志位
extern unsigned char Track_auto_statue; //跟踪手动状态

extern unsigned char SDI_BeginTrack_flag;
extern unsigned char PAL_BeginTrack_flag;



void PayLoad_TaskRx1(int uartname)
{
 	char   Uart1Buff[BUF_MAX1];
	char   dataBuff[BUF_MAX1];
	char   data_recv_catched = 0;
	char   plFrameHead[2] = {0x7e,0x16};        
	char   plFrameTail[1] = {0};             
	unsigned short  crcvalue = 0,checksum = 0;
	int uart1_read_len=0;
 	uart1_read_len= read(uartname, Uart1Buff,sizeof(Uart1Buff)); 
	QueuePushBack(&exPayLoadRXFrameBuff1, Uart1Buff, uart1_read_len);

    while (QueueLength(&exPayLoadRXFrameBuff1)>= BUF_MAX1)     
    {        
		uart1_read_len=  QueueLength(&exPayLoadRXFrameBuff1); 
        if (FrameCompare(&exPayLoadRXFrameBuff1, plFrameHead, 2, plFrameTail, 0, uart1_read_len))
        {
			QueuePopFront(&exPayLoadRXFrameBuff1,TaskRx1_DATA,BUF_MAX1,1);
			data_recv_catched = 1;
		}		
        else
        {
			QueuePopFront(&exPayLoadRXFrameBuff1,0,1,1);
        }
		
    }
	if(data_recv_catched)
	{
		PayLoad1_Decode(TaskRx1_DATA, uartname);
	}
}


void PayLoad_TaskRx2(int uartname)
{
	char   Receive_Buff[BUF_MAX2];
	char   Data_Buff[BUF_MAX2];
	char   Data_recv_catched = 0;
	char   plFrameHead[2] = {0x55,0x11};        
	char   plFrameTail[1] = {0};                
	int    Receive_Len=0;
	
	//从串口读出字符，压入队列***********************************************
   	Receive_Len= read(uartname, Receive_Buff, sizeof(Receive_Buff)); 
	QueuePushBack(&exPayLoadRXFrameBuff2, Receive_Buff, Receive_Len);
	//等待队列中的数据量达到需要接收到的指令长度*****************************
    while(QueueLength(&exPayLoadRXFrameBuff2) >= BUF_MAX2)     
    {	
    	//比较数据的头部 和 尾部 数据是否一致*******************************
    	//if(FrameCompare(&exPayLoadRXFrameBuff2, plFrameHead, 2, &plFrameTail, 0, BUF_MAX2))
		//将队列中的0位开始的2个数据与plFrameHead[]作比对，判断指令头部是否一致
	    if(QueueCompare(&exPayLoadRXFrameBuff2, 0, plFrameHead, 2))
	    {     
	    	//是否进行校验码校验********************************************
			#if 0
	    	//将环形队列中BUF_MAX2的数据"复制"到Data_Buff*******************
			QueuePopFront(&exPayLoadRXFrameBuff2, Data_Buff, BUF_MAX2, 0); //注意0不擦除，即复制
		    int i;
			unsigned short  CRC_Value = 0,CRC_Checksum = 0;
			//累加数据与校验和比对******************************************
			//且注意指令校验和是哪些数据累加和******************************
			for(i=0; i<BUF_MAX2-1; i++)
				CRC_Value += Data_Buff[i];
			
			CRC_Value    = CRC_Value & 0x00ff;
			CRC_Checksum = Data_Buff[BUF_MAX2-1];
			
			#if 0
			int i;
			printf("接收到的指令为：\n");
			for(i=0; i<64; i++)
				printf(" 0x%0x ",Data_Buff[i]);
			printf(" \n");
			#endif
			
            if (CRC_Checksum != CRC_Value) 
            {
            	//如果校验和不对，则环形队列头部向后推两位，即数据挤掉前两位
		  	    QueuePopFront(&exPayLoadRXFrameBuff2, 0, 2, 1);
           		continue; 
            } 			
			#endif
			//将环形队列中BUF_MAX2长度的数据移入UART2_Receive_DATA中*********
			QueuePopFront(&exPayLoadRXFrameBuff2, TaskRx2_DATA, BUF_MAX2, 1); //注意1擦除，即剪切
			Data_recv_catched = 1;
		}		
        else
			//环形队列头部向后推一位，即数据挤掉前一位***********************
			QueuePopFront(&exPayLoadRXFrameBuff2, 0, 1, 1);
    }
	if(Data_recv_catched)
	{
		//指令UART2_Receive_DATA解析*****************************************
		PayLoad2_Decode(TaskRx2_DATA, sizeof(TaskRx2_DATA), uartname);
	}
}

//***************************************************************************
unsigned char SENSOR_IR_CCD = IR_SENSOR ;    //0：红外； 1：可见光 
unsigned char SENSOR_STATE  = TRACK_STATE;   //0：跟踪； 1: 搜索

extern unsigned char IR_Track_Begin_Flag ;
extern unsigned char CCD_Track_Begin_Flag;
extern short Camera_Resolution;              //VPU设置图像分辨率

void PayLoad2_Decode(char *pBuff, int pBuffLen, int uartname)
{	
	MsgRevSerialCmd *pMsg = (MsgRevSerialCmd*)malloc(sizeof(MsgRevSerialCmd));
	memcpy(pMsg, pBuff, pBuffLen);
	printf("Work_Mode :0 wait, 1 track; Field_Of_View :1 2 3 CCD, 0x11 0x10 IR; SelfTest :0 normal, 1 error \n");
	printf("MSG: Msg_Head = 0x%04x, Work_Mode = 0x%02x, Field_Of_View = 0x%02x, SelfTest = 0x%02x\n",pMsg->Msg_Head, pMsg->Work_Mode, pMsg->Field_Of_View, pMsg->SelfTest);
	if(!pMsg->SelfTest)
	{
		switch(pMsg->Field_Of_View)
		{
			case 0x01:
			case 0x02:
			case 0x03:
				SENSOR_IR_CCD = CCD_SENSOR; //CCD
				break;
			case 0x11:
			case 0x10:
				SENSOR_IR_CCD = IR_SENSOR; //IR
				break;
			default:
				break;
		}
		#if 0
		unsigned char ACK_IR[11]  = {0,0,0,0,0,0,0,0,0,0,0};
		unsigned char ACK_CCD[11] = {1,1,1,1,1,1,1,1,1,1,1};
		if((SENSOR_IR_CCD == IR_SENSOR)&&(Current_Cmd_State != IR_Track))
		{
			//quitflag = 1;
			//write(UART2_fd, ACK_IR, 11);
			Current_Cmd_State = IR_Track;
		}
		else if((SENSOR_IR_CCD == CCD_SENSOR)&&(Current_Cmd_State != CCD_Track))
		{
			//quitflag = 1;
			//write(UART2_fd, ACK_CCD, 11);
			Current_Cmd_State = CCD_Track;
		}
		else
		{
			
		}
		
		if(pMsg->Work_Mode == 0) //待命
		{	
			IR_Track_Begin_Flag = 0;
			CCD_Track_Begin_Flag = 0;
		}
		else                    //跟踪
		{
			IR_Track_Begin_Flag = 1;
			CCD_Track_Begin_Flag = 1;
		}
		#endif
	}
	free(pMsg);
}

void PayLoad1_Decode(char *pBuff, int uartname)
{
	
}







