#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

//***********************************************************
#define IR_Track 0
#define IR_Alarm 1
#define CCD_Track 2
#define CCD_Alarm 3

#define NCC_Algorithm_Track 0
#define QSImage_Algorithm_Track 1
#define TLD_Algorithm_Track 2

extern unsigned char Current_Cmd_State ;//当前跟踪搜索状态

#define IR_SENSOR 0
#define CCD_SENSOR 1
#define TRACK_STATE 0
#define ALARM_STATE 1

extern unsigned char SENSOR_IR_CCD ;    //0：红外； 1：可见光 
extern unsigned char SENSOR_STATE ;     //0：跟踪； 1: 搜索


typedef struct 
{
	unsigned short Msg_Head;                //0x1155
	unsigned char  Work_Mode;               //工作模式     00待命；02跟踪
	unsigned char  Field_Of_View;           //CCD:大视场-0x01、中视场-0x02、小视场-0x03; 红外:大视场-0x11、小视场-0x10
	unsigned char  Cross;                   //0x00 为黑十字信息 0x01 为白十字信息
	unsigned char  SelfTest;                //自检：    0 为系统正常、 1 为系统故障
	unsigned char  Cross_CheckAxis;
	unsigned char  Cross_CheckAxis_Direction;
	unsigned char  Horizontal_Adjust;
	unsigned char  Vertical_Adjust;
}MsgRevSerialCmd;


typedef struct 
{
	unsigned short Msg_Head;                //0x2255
	unsigned short Target_x;                //低在前，高在后          小端模式
	unsigned short Target_y;
	unsigned char  Tracking_Status;         //跟踪状态：   0 为正常、 1 为丢失
	unsigned char  SelfTest;                //自检：    0 为系统正常、 1 为系统故障
	unsigned char  Reserve[3];              //保留
	unsigned char  CRC_Checksum;            //校验和
}MsgSendSerialCmd;



//响应
//***********************************************************

void PayLoad_Init1(void);
void PayLoad_Init2(void);


void PayLoad_TaskRx1(int uartname);
void PayLoad_TaskRx2(int uartname);


void PayLoad2_Decode(char *pBuff, int pBuffLen, int uartname);

void PayLoad1_Decode(char *pBuff, int uartname);



#endif
