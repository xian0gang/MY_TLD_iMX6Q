#include "Variable_Function.h"
#include <ipu.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>

#include <sys/time.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <sys/select.h> 
#include <unistd.h>

#include "qsImgLib.h"
#include "RunLength_Algorithm.h"

extern "C" {

#include "NCC_Track.h"

}


#pragma pack(1)

extern int TCP_Accept_CMD_fd;
extern int TCP_Accept_VIDEO_fd;
extern int UART3_fd;

typedef struct BoardMsgBase_s
{
	unsigned int MsgSize;//全部的消息结构的大小
	unsigned short MsgCode;//消息码
}BoardMsgBase;

enum EMsgID
{
	eMsgH264Data=0,//h264视频流数据
	eMsgSearchJPEG,//搜索图像数据
	eMsgSerialCmd,//串口消息透传
	eMsgTrckerRsp,//跟踪器反馈
	eMsgSwitchTrackVideo, //切换视频
	eMsgShutterCorrection, //校正快门
	eMsgIRSetParma
};

typedef struct 
{
	unsigned int MsgSize;//全部的消息结构的大小
	unsigned short MsgCode;//消息码
	int debug;
	int nChannel;//通道 0:红外 1:可见光
	int nDataLen;//h264数据长度
	char data[4];//数据站位,实际长度不是4,是nDataLen
}MsgH264Data;

typedef struct 
{
	unsigned int MsgSize;//全部的消息结构的大小
	unsigned short MsgCode;//消息码
	int debug;
	int nChannel;//通道 0:红外 1:可见光
	unsigned int nFrameOffset;//帧偏移
	int aimInfoCmdLen;//告警器反馈命令实际长度
	char aimInfoCmd[64];//告警器反馈命令
	int nOrder;//帧号
	int nDataLen;//h264数据长度
	char data[4];//数据站位,实际长度不是4,是nDataLen
}MsgSearchJPEG;


typedef struct //跟板子发的指令都通过这个消息包装
{
	unsigned int MsgSize;//全部的消息结构的大小
	unsigned short MsgCode;//消息码
	int debug;
	int nCmdLen;//指令长度
	char cmd[256];//指令
}MsgSerialCmd;


typedef struct //跟板子发的指令都通过这个消息包装
{
	unsigned int MsgSize;//全部的消息结构的大小
	unsigned short MsgCode;//消息码
	int debug;
	int bIrOrCCD;//0:ir 1:ccd
}MsgSwitchTrackVideo;

typedef struct //跟板子发的指令都通过这个消息包装
{
	unsigned int MsgSize;//全部的消息结构的大小
	unsigned short MsgCode;//消息码
	int debug;
}MsgShutterCorrection;


typedef struct //跟板子发的指令都通过这个消息包装
{
	unsigned int MsgSize;//全部的消息结构的大小
	unsigned short MsgCode;//消息码
	int debug;
	int nlen;
	char cmd[32];
}MsgIRSetParma;

int TCP_Accept_VIDEO_fd = 0;
int TCP_Accept_CMD_fd =0;
void Send_H264_Frame(char *pH264FrameData,int nH264FrameLen,int channel)//发送h264帧数据,每次发一帧
{
	int ret;
	MsgH264Data *pMsg=(MsgH264Data*)malloc(sizeof(MsgH264Data)-4 + nH264FrameLen);
	//printf("sizeof(MsgH264Data) = %d ******s\n",sizeof(MsgH264Data));
	pMsg->MsgSize=sizeof(MsgH264Data)-4 + nH264FrameLen;
	pMsg->MsgCode=eMsgH264Data;
	pMsg->nDataLen=nH264FrameLen;
	pMsg->nChannel=channel;
	pMsg->debug = 12345;
	memcpy(pMsg->data,pH264FrameData,nH264FrameLen);
	//SendMsg(pMsg,pMsg->MsgSize);//直接发送tcp socket
	if(TCP_Accept_VIDEO_fd >= 0)
	{
		ret  = send(TCP_Accept_VIDEO_fd, (MsgH264Data *)pMsg, pMsg->MsgSize,0);
		if(ret !=  pMsg->MsgSize)
		{
			printf(" TCP :Send_H264_Frame ERROR  !!!********************\n");
		}
	}
	free((MsgH264Data *)pMsg);
}


void Send_JPG_Frame(char *pJPGHeadData,int nJPGHeadLen,char *pJPGFrameData,int nJPGFrameLen,int Frame_NUM,int channel)//发送JPG帧数据,每次发一帧
{
	int ret;
	MsgSearchJPEG *pMsg=(MsgSearchJPEG*)malloc(sizeof(MsgSearchJPEG)-4 + nJPGFrameLen + nJPGHeadLen);
	pMsg->MsgSize=sizeof(MsgSearchJPEG)-4 + nJPGFrameLen + nJPGHeadLen;
	pMsg->MsgCode=eMsgSearchJPEG;
	pMsg->nDataLen=nJPGFrameLen + nJPGHeadLen;
	pMsg->nChannel=channel;
	pMsg->debug = 12345;
	pMsg->nOrder=Frame_NUM;
	//pMsg->aimInfoCmd=0;
	pMsg->aimInfoCmdLen = 0;
	memcpy(pMsg->data,pJPGHeadData,nJPGHeadLen);
	memcpy(pMsg->data + nJPGHeadLen,pJPGFrameData,nJPGFrameLen);
	//SendMsg(pMsg,pMsg->MsgSize);//直接发送tcp socket
	if(TCP_Accept_VIDEO_fd >= 0)
	{
		ret = send(TCP_Accept_VIDEO_fd, (MsgSearchJPEG *)pMsg, pMsg->MsgSize, 0);
		if(ret !=  pMsg->MsgSize)
		{
			printf(" TCP :Send_JPG_Frame ERROR  !!!********************\n");
		}
	}
	//printf("pMsg->MsgSize = %x = %d; nJPGFrameLen = %d\n",pMsg->MsgSize,pMsg->MsgSize,nJPGFrameLen);
	free((MsgSearchJPEG *)pMsg);
}


void Send_CMD(char *pCMDData,int nCMDLen)
{
	MsgSerialCmd *pMsg = (MsgSerialCmd *)malloc(sizeof(MsgSerialCmd));
	pMsg->MsgSize = sizeof(MsgSerialCmd);
	pMsg->MsgCode = eMsgSerialCmd;
	pMsg->nCmdLen = nCMDLen;
	pMsg->debug = 12345;
	memcpy(pMsg->cmd,pCMDData,nCMDLen);
	if(TCP_Accept_CMD_fd >= 0)
		send(TCP_Accept_CMD_fd, (MsgSerialCmd *)pMsg, pMsg->MsgSize, 0);
	else
	{
		//printf("Send_CMD TCP_Accept_CMD_fd not connect ******\n");
	}
	free((MsgSerialCmd *)pMsg);
}

int Receive_CMD(char *pCMDData,int nCMDLen)
{
	if((nCMDLen!= (sizeof(MsgSerialCmd)))&&(nCMDLen!= (sizeof(MsgSwitchTrackVideo)))&&(nCMDLen!= (sizeof(MsgShutterCorrection)))&&(nCMDLen!= (sizeof(MsgIRSetParma))))
	{
		printf("Receive_CMD len ERROR *********\n");
		int i;
		printf("nCMDLen = %d",nCMDLen);
		for(i=0; i<nCMDLen; i++)
			printf("%02x",pCMDData[i]);
		return -1;
	}
	if(nCMDLen == (sizeof(MsgSerialCmd)))
	{
		MsgSerialCmd *pMsg = (MsgSerialCmd *)malloc(sizeof(MsgSerialCmd));
		memcpy(pMsg,pCMDData,nCMDLen);
		//Command_Analysis(TCP_Accept_CMD_fd, pMsg->cmd, pMsg->nCmdLen);	
		free((MsgSerialCmd *)pMsg);
	}
	else if(nCMDLen == (sizeof(MsgSwitchTrackVideo)))
	{
		MsgSwitchTrackVideo *pMsg = (MsgSwitchTrackVideo *)malloc(sizeof(MsgSwitchTrackVideo));
		memcpy(pMsg,pCMDData,nCMDLen);
		//Command_Analysis_SwitchVideo(pMsg->bIrOrCCD);
		free((MsgSwitchTrackVideo *)pMsg);
	}
	else if(nCMDLen == (sizeof(MsgIRSetParma)))
	{
		MsgIRSetParma *pMsg = (MsgIRSetParma *)malloc(sizeof(MsgIRSetParma));
		memcpy(pMsg,pCMDData,nCMDLen);
		write(UART3_fd, pMsg->cmd, pMsg->nlen);
		//printf("SET PARMA**************\n");
		free((MsgIRSetParma *)pMsg);
	}
	else
	{
		MsgShutterCorrection *pMsg = (MsgShutterCorrection *)malloc(sizeof(MsgShutterCorrection));
		memcpy(pMsg,pCMDData,nCMDLen);
		if(pMsg->MsgCode == eMsgShutterCorrection)
		{
			unsigned char ShutterCorrection[9] = {0xAA,0x05,0x01,0x02,0x02,0x44,0xF8,0xEB,0xAA};
			write(UART3_fd, ShutterCorrection, sizeof(ShutterCorrection));
			//printf("ShutterCorrection******\n");
		}
	}
	
	return 0;
}




unsigned int fmt_to_bpp(unsigned int pixelformat)
{
	unsigned int bpp;
	switch (pixelformat)
	{
		case IPU_PIX_FMT_RGB565:
	   /*interleaved 422*/
		case IPU_PIX_FMT_YUYV:
		case IPU_PIX_FMT_UYVY:
		/*non-interleaved 422*/
		case IPU_PIX_FMT_YUV422P:
		case IPU_PIX_FMT_YVU422P:
				bpp = 16;
				break;
		case IPU_PIX_FMT_BGR24:
		case IPU_PIX_FMT_RGB24:
		case IPU_PIX_FMT_YUV444:
		case IPU_PIX_FMT_YUV444P:
				bpp = 24;
				break;
		case IPU_PIX_FMT_BGR32:
		case IPU_PIX_FMT_BGRA32:
		case IPU_PIX_FMT_RGB32:
		case IPU_PIX_FMT_RGBA32:
		case IPU_PIX_FMT_ABGR32:
				bpp = 32;
				break;
		/*non-interleaved 420*/
		case IPU_PIX_FMT_YUV420P:
		case IPU_PIX_FMT_YVU420P:
		case IPU_PIX_FMT_YUV420P2:
		case IPU_PIX_FMT_NV12:
		case IPU_PIX_FMT_TILED_NV12:
				bpp = 12;
				break;
		default:
				bpp = 8;
				break;
	}
	return bpp;
}


//*********************IPU 参数*****************************
struct ipu_task task;
int fd_ipu = 0;   // IPU file descriptor
int isize = 0;	  // input size
int osize = 0;	  // output size

unsigned char *inbuf = NULL;
//void *outbuf = NULL;
unsigned char *outbuf = NULL;

#define IPU_WIDTH 720
#define IPU_HEIGHT 576

//*********************************************************

int IPU_Init(void)
{
	int ret;
	task.input.width    = IPU_WIDTH;
	task.input.height   = IPU_HEIGHT;
	task.input.format   = v4l2_fourcc('N', 'V', '1', '2');
 
	// Output image size and format
	task.output.width   = IPU_WIDTH;
	task.output.height  = IPU_HEIGHT;
	task.output.format  = v4l2_fourcc('R', 'G', 'B', '3');
  
	// Open IPU device
	fd_ipu = open("/dev/mxc_ipu", O_RDWR, 0);
	if (fd_ipu < 0) 
	{
		printf("open ipu dev fail\n");
		return  -1;
	}
	
	isize = task.input.paddr =
	task.input.width * task.input.height
	* fmt_to_bpp(task.input.format)/8;

	ret = ioctl(fd_ipu, IPU_ALLOC, &task.input.paddr);
	if (ret < 0) {
		printf("ioctl IPU_ALLOC fail: (errno = %d)\n", errno);
		return  -1;
	}
	//inbuf = mmap(0, isize, PROT_READ | PROT_WRITE,MAP_SHARED, fd_ipu, task.input.paddr);
	inbuf = (unsigned char*)mmap(0, isize, PROT_READ | PROT_WRITE,MAP_SHARED, fd_ipu, task.input.paddr);//g++ :'void*' to 'unsigned char*'
	if (!inbuf) {
		printf("mmap fail\n");
		return  -1;
		}

	osize = task.output.paddr =
		task.output.width * task.output.height
		* fmt_to_bpp(task.output.format)/8;

	ret = ioctl(fd_ipu, IPU_ALLOC, &task.output.paddr);
	if (ret < 0) {
		printf("ioctl IPU_ALLOC fail\n");
		return	-1;
	}

	//outbuf = mmap(0, osize, PROT_READ | PROT_WRITE,MAP_SHARED, fd_ipu, task.output.paddr);
	outbuf = (unsigned char*)mmap(0, osize, PROT_READ | PROT_WRITE,MAP_SHARED, fd_ipu, task.output.paddr);//g++ :'void*' to 'unsigned char*'
	if (!outbuf) {
		printf("mmap fail\n");
		return	-1;
	}

	return 0;
}



// *************************保存图像或视频*************************
//注意是否存在路径./File_pictures                                    ！！！
//void *p：   图像指针
//size：      图像大小
//type:      保存 1：图像 或者 其它：视频
// ****************************************************************
void Process_Save_image(const void *p, int size , int type)
{
    FILE *fp_tmp;
	
	if(type == 1)  //保存图片
	{
		static int num = 0;
		num++;
		char number[100] = "";
		char DIR[100]="./File_picture/";      //需要已经存在File_picture路径 ，否则需要创建
		sprintf(number, "%d", num);
		strcat(number, ".yuv");
		strcat(DIR, number);
		fp_tmp = fopen(DIR, "w+");
        fwrite(p, size, 1, fp_tmp);
       
	}
	else
	{
		char number[100] = "YUV_Video";
		char DIR[100]="./File_picture/";      //需要已经存在File_picture路径 ，否则需要创建
		strcat(number, ".yuv");
		strcat(DIR, number);
		fp_tmp = fopen(DIR, "ab+");
        fwrite(p, size, 1, fp_tmp);
	}
	fclose(fp_tmp);
}


/*********************跟踪相关变量*NCC*********************/  
int dis_w, dis_h;
extern void Init_Addr(void);
extern int TmpH;
extern int TmpW;
extern int gSubImageWidth;
extern int gSubImageHeight;
extern unsigned short tar_x,tar_y;
extern unsigned char *gSubImageData;
//static float f32_NccRatio = 0.01f;
int Cnt_Track = 0;
extern unsigned char *Template;
extern unsigned char *Template2;
unsigned char mubiaodiushi = 1;

//**********************跟踪图像****************************
unsigned char QS_img_8u3_rgb888[960*544*3]={0};
//unsigned char gSubImageData_QS_544[960*544*3/2]={0};
//unsigned char* QS_img_8u3_rgb888;
unsigned char* gSubImageData_QS;
unsigned char gSubImageData_malloc[1920*1080*3/2] = {0};


unsigned short CCD_IR_Target_x = 0, CCD_IR_Target_y = 0; 
extern unsigned short Rec_Target_x,Rec_Target_y;

//QS跟踪器  _目标框*******************************************
QSRECT rcInit;

//**********************************************************

extern unsigned char Frame_NUM_FPGA;

int QS_IMAGE_TRACK(int Frame_NUM)
{
	int ret = 0;
	gSubImageData_QS = gSubImageData_malloc;
	//***************************拷贝图像到IPU进行转换*****************************
	memcpy(inbuf, gSubImageData_QS, isize);                 //将原始图像拷贝到inbuf

	//****IPU***转换实现************************
	ret = ioctl(fd_ipu, IPU_QUEUE_TASK, &task);
	if (ret < 0)
		printf("YUYV -> RGB  IPU_QUEUE_TASK Fail****\n");

	memcpy(QS_img_8u3_rgb888, outbuf, osize);                //将outbuf图像拷贝出去
	//*******************************保存RGB图像**********************************
	static int KKK = 0;
	if(KKK == 0)
	{
		//qsBmpRgbSave(QS_img_8u3_rgb888, 720, 576, "qs.bmp", 1);
		KKK++;
	}
	//*****************************初始化跟踪器************************************
	if(Frame_NUM == 0)						
		qsObjectTrackStart(QS_img_8u3_rgb888, IPU_WIDTH, IPU_HEIGHT, rcInit);					
	if(Frame_NUM >= 1)
		rcInit = qsObjectTrackExcute(QS_img_8u3_rgb888, IPU_WIDTH, IPU_HEIGHT);

	mubiaodiushi = 0;

	CCD_IR_Target_x = (rcInit.left + rcInit.right)/2;
	CCD_IR_Target_y = (rcInit.top + rcInit.bottom)/2;
	//printf("*************CCD_Target_x =%d CCD_Target_y =%d***************\n",CCD_Target_x,CCD_Target_y);
	//if(Frame_NUM >= 1)
		//Send_Target_to_SIFU(CCD_Target_x, CCD_Target_y, 1);   //0: IR ,1:CCD   //发送跟踪目标位置
}

//统计单帧图像获取与处理的耗时
struct	timeval tpstart,tpend;
float timeuse;


int NCC_IMAGE_TRACK(int Frame_NUM)
{
	int x,y;
	gSubImageData = gSubImageData_malloc;
	
	gettimeofday(&tpstart, NULL);
	
	//memcpy(gSubImageData, gSubImageData_malloc, 720*576);
	//Process_Save_image(gSubImageData, 720*576 , 1);
	#if 1
	if(Frame_NUM == 0)
	{
		tar_x = Rec_Target_x;
		tar_y = Rec_Target_y;
		for(x=0; x<TmpW; x++)
		{
			for(y=0; y<TmpH; y++)
			{
				Template[x*TmpH+y] = gSubImageData[(tar_y-TmpW/2+x)*720+(tar_x-TmpW/2+y)];
			}
		}	
		TrackLevel(0);//判断跟踪器强度: 0	丢失目标局部重检; 1	丢失目标全屏重检; 2	更难丢失目标条件
		TrackInit(&tar_x,&tar_y);
		mubiaodiushi = 0;              //初始跟踪 应为跟上目标
		printf("Begin NCC Tracking *****tar_x =%d, tar_y = %d********************\n",tar_x,tar_y);
	}

	if(Frame_NUM >= 1)
	{
		gSubImageWidth  = 720;
		gSubImageHeight = 576;
		mubiaodiushi = TrackTarget(gSubImageData,Template,Template2,gSubImageWidth,gSubImageHeight,TmpW,TmpH,&tar_x,&tar_y);	
		
	}
	#endif
	
	
	CCD_IR_Target_x = tar_x;
	CCD_IR_Target_y = tar_y;
	//printf("Begin NCC Tracking ***CCD_IR_Target_x = %d  CCD_IR_Target_y = %d ********************\n",CCD_IR_Target_x,CCD_IR_Target_y);

	gettimeofday(&tpend, NULL);
	timeuse = 1000*(tpend.tv_sec - tpstart.tv_sec) + (tpend.tv_usec - tpstart.tv_usec)/1000.0;
	//printf("*************************Frame_NUM = %d*******mubiaodiushi = %d*******Usetime = %.2f ms   *******\n",Frame_NUM,mubiaodiushi,timeuse);
}


#if 1

Mat last_gray;
Mat current_gray;
Rect box_b;
Mat frame;
Mat first;
FILE *bb_file;
bool tl1;


int TLD_IMAGE_TRACK(int Frame_NUM)
{
	// 跟踪开始	
	// 跟踪初始化	
	if(Frame_NUM == 0)
	{
		FileStorage fs;
		fs.open("parameters.yml", FileStorage::READ);
		read(fs.getFirstTopLevelNode());
		fs.release();		
		
		tl1 = true;
		last_gray    = Mat::zeros(576, 720, CV_8UC1);
		current_gray = Mat::zeros(576, 720, CV_8UC1);
		bb_file = fopen("bounding_boxes.txt", "w");

	
		memcpy(last_gray.data, gSubImageData_malloc, 720*576); 

        Mat last_gray_resize(576/MULTIPLE, 720/MULTIPLE, CV_8UC1);

//		resize(last_gray, last_gray_resize, Size(last_gray.cols/MULTIPLE, last_gray.rows/MULTIPLE));
        my_resize(last_gray.data, last_gray_resize.data, last_gray.cols, last_gray.rows, last_gray.cols/MULTIPLE, last_gray.rows/MULTIPLE);

		//box_b.x = box_b.x / 4;
		//box_b.y = box_b.y / 4;
		//box_b.width = box_b.width / 4;
		//box_b.height = box_b.height / 4;
		
		box_b.x = (Rec_Target_x-32) / MULTIPLE;
		box_b.y = (Rec_Target_y-32) / MULTIPLE;
		//printf("``````````````````````box_b.x:%d\n",box_b.x);
		//printf("``````````````````````box_b.y:%d\n",box_b.y);
		box_b.width  = 64 / MULTIPLE;
		box_b.height = 64 / MULTIPLE;

		
// printf("box:%d %d %d %d \n", box.x, box.y, box.width, box.height);
// imwrite("box.bmp",last_gray_resize(box));
		init(last_gray_resize, box_b, bb_file);


		ppp();
		detect_d2();
		detect_d3();
		detect_d4();


		// drawBox(last_gray, box_b);
		// init(last_gray, box_b, bb_file);
		// imwrite("last_gray.bmp", last_gray);
		////tld_init_flag = false;
		//tld_pro_flag = true;
	}
	// 跟踪 检测
	if(Frame_NUM >= 1)
	{
		memcpy(current_gray.data, gSubImageData_malloc, 720*576); 
		BoundingBox pbox;
		BoundingBox draw_pbox;
		std::vector<Point2f> pts1;
		std::vector<Point2f> pts2;
		bool status = true;

		processFrame(last_gray, current_gray, pts1, pts2, pbox, status, tl1, bb_file);
		mubiaodiushi = 1;
		if(status)
		{
			// drawPoints(current_gray, pts1);
			// drawPoints(current_gray, pts2, Scalar());
			draw_pbox.x = pbox.x*MULTIPLE;
			draw_pbox.y = pbox.y*MULTIPLE;
			draw_pbox.width = pbox.width*MULTIPLE;
			draw_pbox.height = pbox.height*MULTIPLE;
           // drawBox(current_gray, draw_pbox);
           mubiaodiushi = 0;
		   CCD_IR_Target_x = draw_pbox.x + draw_pbox.width/2;
		   CCD_IR_Target_y = draw_pbox.y + draw_pbox.height/2;
		   //printf(" *********************************************mubiaodiushi = %d  ,CCD_IR_Target_x = %d  ,CCD_IR_Target_y = %d***** \n",mubiaodiushi,CCD_IR_Target_x,CCD_IR_Target_y);
		   
		}
		//memcpy(capture_buffers[capture_buf.index].start, current_gray.data, WIDTH*HEIGHT); 
		swap(last_gray, current_gray);
		pts2.clear();
		pts1.clear();
	}
}

#endif
