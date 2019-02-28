#define __USE_GNU                 //启用CPU_ZERO等相关的宏
//#define _GNU_SOURCE

#include  <pthread.h>
#include  <sched.h>
#include  <signal.h>
#include  <getopt.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <math.h>

#include <malloc.h>
#include "mxcfb.h"
#include "mxc_v4l2.h"
//#include "ipu.h"

#include <linux/types.h>
#include <assert.h>
#include <dirent.h>

#include <sys/msg.h>  
#include <errno.h>  

#include "File_Queue.h"
#include "Init_dev.h"
#include "Communication.h"
#include "VPU_Camera.h"
#include "Variable_Function.h"
#include "qsImgLib.h"

extern "C" {

#include "NCC_Track.h"
}

#ifndef   UINT64_C
#define   UINT64_C(value)__CONCAT(value,ULL)
#endif

#ifdef __cplusplus
extern "C" {
#endif
//#define __STDC_CONSTANT_MACROS

#include "libavformat/avformat.h"
//#include "libavutil/mathematics.h"
#include "libavutil/time.h"

#ifdef __cplusplus
}
#endif

#include <opencv2/opencv.hpp>
#include <tld_utils.h>
#include <TLD.h>
#include <sstream>
#include <iostream>

using namespace cv;
using namespace std;



//****************************************************
//创建线程
//****************************************************
pthread_t camera_tid;
pthread_t UDP_tid;  
pthread_t Track_tid;                                 
pthread_t UART2_tid;
pthread_t UART1_tid;

pthread_t socket_tid;

//********************标志位&变量**************************
unsigned char Track_Begin_Flag = 0;

volatile unsigned char Frame_Process_Begin_Flag = 0;
volatile unsigned char Frame_Process_End_Flag = 1;

int Frame_num = 0;
int Frame_Num_Track = 0 ;

unsigned short Rec_Target_x = 0;
unsigned short Rec_Target_y = 0;

unsigned short tar_x1,tar_y1,tar_x2,tar_y2;
extern unsigned char mubiaodiushi;


unsigned short VPU_Capture_Resolution = 576;//VPU设置图像分辨率
unsigned char VPU_Compress_Format = 2;


extern unsigned char Frame_NUM_FPGA;
unsigned char Alarm_Begin_Flag = 0;
unsigned char Current_Cmd_State = 2;

extern int New_JPG_filename ;
extern int New_JPG ;

//管道******************************************************
int pipe_fdname;

//串口设备文件描述符****************************************
int UART1_fd,UART2_fd,UART3_fd;
extern int quitflag;

//*********************IPU 参数*****************************
extern struct ipu_task task;
extern int fd_ipu ;   // IPU file descriptor
extern int isize ;	  // input size
extern int osize ;	  // output size

extern unsigned char *inbuf ;
extern void *outbuf ;
//**********************************************************
//QS跟踪器  _目标框*******************************************
extern QSRECT rcInit;

//处理函数
//**********************************************************

//void tracking_process(void)
void* tracking_process(void *ptr)
{	
	int x, y;
	cpu_set_t cpuset;
	pthread_t thread = pthread_self();
	int s,core_no = 3;
	CPU_ZERO(&cpuset);
	CPU_SET(core_no,&cpuset);
	s = pthread_setaffinity_np(thread,sizeof(cpuset),&cpuset);

	if(s != 0)
		printf("Set pthread_setaffinity_np failed\n"); 


	//***************************NCC_Init初始化************************************
	Init_Addr();

	//*****************************IPU NV12->RGB**初始化***************************
	IPU_Init();
	

	//统计单帧图像获取与处理的耗时
	struct	timeval tpstart,tpend;
	struct	timeval tpend1,tpend2,tpend3,tpend4;
	float timeuse;
	float timeuse1,timeuse2,timeuse3,timeuse4;


	while(1)
	{
		usleep(1000);//1ms
		if(Frame_Process_Begin_Flag == 1)
		{
			Frame_Process_Begin_Flag = 0;
			switch(Current_Cmd_State)
			{
				case NCC_Algorithm_Track://0
					if(Track_Begin_Flag == 1)
					{
						NCC_IMAGE_TRACK(Frame_Num_Track);
						Frame_Num_Track++;
						if(Frame_Num_Track >= 100)
							Frame_Num_Track = 10;
					}
					else
						Frame_Num_Track = 0;
	
					break;

				case QSImage_Algorithm_Track://1
					if(Track_Begin_Flag == 1)
					{
						QS_IMAGE_TRACK(Frame_Num_Track);
						Frame_Num_Track++;
						if(Frame_Num_Track >= 100)
							Frame_Num_Track = 10;
					}
					else
						Frame_Num_Track = 0;

					break;	
					
				case TLD_Algorithm_Track: //2
					if(Track_Begin_Flag == 1)
					{
						TLD_IMAGE_TRACK(Frame_Num_Track);
						Frame_Num_Track++;
						if(Frame_Num_Track >= 100)
							Frame_Num_Track = 10;
					}
					else
						Frame_Num_Track = 0;
					
					break;
					
				default:
					break;
			}

			Frame_Process_End_Flag = 1;
		}
	}
	UnInit_Addr();
	return NULL;
}

void* camera_process(void *ptr)
{
	cpu_set_t cpuset;
	pthread_t thread = pthread_self();
	int s,core_no = 0;
	CPU_ZERO(&cpuset);
	CPU_SET(core_no,&cpuset);
	s = pthread_setaffinity_np(thread,sizeof(cpuset),&cpuset);
	if(s != 0)
		printf("Set pthread_setaffinity_npfailed\n");

	pipe_fdname = open("pipe_filename", O_WRONLY);
//**************************************************************************
	while(1)
	{ 
		usleep(1000);
		quitflag = 0;
		printf("Begin  VPU**************************\n");
		VPU_camera(VPU_Capture_Resolution, VPU_Compress_Format);
	}
	return NULL;
}

//void UART2_process(void)
void* UART2_process(void *ptr)
{
	cpu_set_t cpuset;
	pthread_t thread = pthread_self();
	int s,core_no = 0;
	CPU_ZERO(&cpuset);
	CPU_SET(core_no,&cpuset);

	s = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
   	if(s != 0)
    	printf("Set pthread_setaffinity_np failed\n");

	PayLoad_Init2();
	for(;;)
	{
		usleep(15000);
		PayLoad_TaskRx2(UART3_fd);
	}
	return NULL;
}

//void UART1_process(void)
void* UART1_process(void *ptr)
{
	cpu_set_t cpuset;
	pthread_t thread = pthread_self();
	int s,core_no = 0;	
	CPU_ZERO(&cpuset);
	CPU_SET(core_no,&cpuset);

	s = pthread_setaffinity_np(thread,sizeof(cpu_set_t),&cpuset);
   	if(s != 0)
    	printf("Set pthread_setaffinity_np failed\n");

	while(1)
	{
		sleep(1);
	}
	return NULL;
}

#if 1
/*****遇到error C3861: 'UINT64_C': identifier not found在common.h里加入定义如下： 
/home/xixian/my-imx6/ffmpeg/ffmpeglib/include/libavutil/common.h
#ifndef INT64_C 
#define INT64_C(c) (c ## LL) 
#define UINT64_C(c) (c ## ULL) 
#endif
*******************************************************************************/
int rtmpStart()
{
    printf("rtmp process Start......\n");
    AVOutputFormat *ofmt = NULL;

    AVFormatContext *ifmt_ctx = NULL, *ofmt_ctx = NULL;

    //
    AVPacket pkt;
    //Êä³öÂ·Ÿ¶
    const char *out_filename;
    const char *in_filename = NULL;

    uint8_t *buffer = NULL;
    size_t buffer_size;

    int ret, i;
    int videoindex = -1;
    int frame_index = 0;
    int64_t start_time = 0;

    //×¢²á
    av_register_all();
    //Network
    avformat_network_init();

    out_filename = "udp://192.168.1.1:6666";
    printf("addr:%s\n", out_filename);

    //ÊäÈë£šInput£©
    char buf[] = "";
    if ((ret = avformat_open_input(&ifmt_ctx, "pipe_filename"/*in_filename*/, NULL, NULL)) < 0)
    {
        printf( "Could not open input file.\n");
        av_strerror(ret, buf, 1024);
        printf("Couldn't open file: %d(%s)\n", ret, buf);
        goto end;
    }

    //Á÷ÐÅÏ¢
    if ((ret = avformat_find_stream_info(ifmt_ctx, 0)) < 0)
    {
        printf( "Failed to retrieve input stream information\n");
        goto end;
    }

    for(i=0; i<ifmt_ctx->nb_streams; i++)
    {
        if(ifmt_ctx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO)
        {
            videoindex=i;
            break;
        }
    }
    av_dump_format(ifmt_ctx, 0, in_filename, 0);
    avformat_alloc_output_context2(&ofmt_ctx, NULL, "mpegts", out_filename);//UDP

    if (!ofmt_ctx)
    {
        printf( "Could not create output context\n");
        ret = AVERROR_UNKNOWN;
        goto end;
    }

    ofmt = ofmt_ctx->oformat;
    for (i = 0; i < ifmt_ctx->nb_streams; i++)
    {
        //žùŸÝÊäÈëÁ÷ŽŽœšÊä³öÁ÷£šCreate output AVStream according to input AVStream£©
        AVStream *in_stream = ifmt_ctx->streams[i];
        AVStream *out_stream = avformat_new_stream(ofmt_ctx, in_stream->codec->codec);
        if (!out_stream)
        {
            printf( "Failed allocating output stream\n");
            ret = AVERROR_UNKNOWN;
                goto end;
        }
        //žŽÖÆAVCodecContextµÄÉèÖÃ£šCopy the settings of AVCodecContext£©
        ret = avcodec_copy_context(out_stream->codec, in_stream->codec);
        if (ret < 0)
        {
            printf( "Failed to copy context from input to output stream codec context\n");
                goto end;
        }
        out_stream->codec->codec_tag = 0;
        if (ofmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
            out_stream->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }
    //Dump Format------------------
    av_dump_format(ofmt_ctx, 0, out_filename, 1);
    //Žò¿ªÊä³öURL£šOpen output URL£©
    if (!(ofmt->flags & AVFMT_NOFILE))
    {
        ret = avio_open(&ofmt_ctx->pb, out_filename, AVIO_FLAG_WRITE);
        if (ret < 0)
        {
            printf( "Could not open output URL '%s'\n", out_filename);
                goto end;
        }
    }
    ofmt_ctx->streams[0]->codec->width = 1;
    ofmt_ctx->streams[0]->codec->height = 1;

    //ÐŽÎÄŒþÍ·£šWrite file header£©
    ret = avformat_write_header(ofmt_ctx, NULL);
    if (ret < 0)
    {
        printf( "Error occurred when opening output URL\n");
            goto end;
    }

    start_time=av_gettime();

    while (1)
    {
        AVStream *in_stream, *out_stream;
        //»ñÈ¡Ò»žöAVPacket£šGet an AVPacket£©
        ret = av_read_frame(ifmt_ctx, &pkt);
        if (ret < 0)
            break;
        //FIX£ºNo PTS (Example: Raw H.264)
        //Simple Write PTS
        if(pkt.pts==AV_NOPTS_VALUE)
        {
            //Write PTS
            AVRational time_base1=ifmt_ctx->streams[videoindex]->time_base;
            //Duration between 2 frames (us)
            int64_t calc_duration=(double)AV_TIME_BASE/av_q2d(ifmt_ctx->streams[videoindex]->r_frame_rate);
            //Parameters
            pkt.pts=(double)(frame_index*calc_duration)/(double)(av_q2d(time_base1)*AV_TIME_BASE);
            pkt.dts=pkt.pts;
            pkt.duration=(double)calc_duration/(double)(av_q2d(time_base1)*AV_TIME_BASE);
        }
        //Important:Delay
        if(pkt.stream_index == videoindex)
        {
            AVRational time_base=ifmt_ctx->streams[videoindex]->time_base;
            AVRational time_base_q={1,AV_TIME_BASE};
            int64_t pts_time = av_rescale_q(pkt.dts, time_base, time_base_q);
            int64_t now_time = av_gettime() - start_time;
            if (pts_time > now_time)
            {
               // printf("delay:%d\n", pts_time - now_time);
               // av_usleep(pts_time - now_time);           //关键！！！注意
            } 
        }

        in_stream  = ifmt_ctx->streams[pkt.stream_index];
        out_stream = ofmt_ctx->streams[pkt.stream_index];
		
        /* copy packet */
        //×ª»»PTS/DTS£šConvert PTS/DTS£©
        pkt.pts = av_rescale_q_rnd(pkt.pts, in_stream->time_base, out_stream->time_base, (AVRounding)(AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX));
        pkt.dts = av_rescale_q_rnd(pkt.dts, in_stream->time_base, out_stream->time_base, (AVRounding)(AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX));
        pkt.duration = av_rescale_q(pkt.duration, in_stream->time_base, out_stream->time_base);
        pkt.pos = -1;
        //Print to Screen
        if(pkt.stream_index == videoindex)
        {
          //  printf("Send %8d video frames to output URL\n",frame_index);
            frame_index++;
        }

		//printf("%d kbs\n", pkt.size / 1024);//发送压缩图像大小
        //ret = av_write_frame(ofmt_ctx, &pkt);
        ret = av_interleaved_write_frame(ofmt_ctx, &pkt);

        if (ret < 0)
        {
            printf( "Error muxing packet\n");
            break;
        }

        av_free_packet(&pkt);

    }
    //ÐŽÎÄŒþÎ²£šWrite file trailer£©
    av_write_trailer(ofmt_ctx);

end:
    avformat_close_input(&ifmt_ctx);
    /* close output */
    if (ofmt_ctx && !(ofmt->flags & AVFMT_NOFILE))
    {
        avio_close(ofmt_ctx->pb);
    }
    avformat_free_context(ofmt_ctx);
    if (ret < 0 && ret != AVERROR_EOF)
    {
        printf( "Error occurred.\n");
        return -1;
    }
}

#endif

/*
///客户端套接字
	struct sockaddr_in TCP_Client_addr;
	socklen_t TCP_Client_addr_length = sizeof(TCP_Client_addr);

	printf("TCP waiting client connnecting!!!\n");
	///成功返回非负描述字，出错返回-1
	*TCP_Accept_fd = accept(TCP_Server_sockfd, (struct sockaddr*)&TCP_Client_addr, &TCP_Client_addr_length);
	if(TCP_Accept_fd < 0)
	{
		perror("TCP Accept error !!! \n");
		return -1;
	}
	//printf("TCP Client connected !!!\n");

void* TCP_CMD_Process(void *ptr)
{
	int sock_fd = -1;
	int ret = -1;
	
	int client_fd[MAX_CLIENT_NUM];

	struct sockaddr_in serv_addr;
	struct sockaddr_in cli_addr;

	socklen_t serv_addr_len = 0;
	socklen_t cli_addr_len = 0; 

	
	char recv_buf[MAX_RECV_LEN]; 
	int new_conn_fd = -1; 
	int i = 0; 
	int max_fd = -1; 
	int num = -1; 

	static int running = 1; 

	while(1)
	{
		int count = 0;
		struct timeval timeout;
		fd_set read_set;
		fd_set write_set; 
		fd_set select_read_set;
		FD_ZERO(&read_set);
		FD_ZERO(&write_set); 
		FD_ZERO(&select_read_set);

		for (i = 0; i < MAX_CLIENT_NUM; i++) 
		{ 
			client_fd[i] = -1; 
		}
		
		memset(&serv_addr, 0, sizeof(serv_addr));
		memset(&cli_addr, 0, sizeof(cli_addr));
		sock_fd = socket(AF_INET, SOCK_STREAM, 0);
		if (sock_fd < 0)
		{ 
			perror("Fail to socket"); 
			return -1; 
		} 

		serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(TCP_CMD_SERVER_PORT);
		serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

		unsigned int value = 1; 

		if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, (void *)&value, sizeof(value)) < 0) 
		{ 
			perror("Fail to setsockopt"); 
			return -1; 
		}
		serv_addr_len = sizeof(serv_addr); 
		if (bind(sock_fd, (struct sockaddr*)&serv_addr, serv_addr_len) < 0) 
		{ 
			perror("Fail to bind"); 
			return -1;  
		} 
		if (listen(sock_fd, BACK_LOG) < 0) 
		{ 
			perror("Fail to listen"); 
			return -1; 
		} 
		char buf[1024]; 
		max_fd = sock_fd; 
		int len; 
		FD_SET(sock_fd, &read_set); 
	
		printf("start server...\n");

		while (running) 
		{ 

			//if(count >= 3)
				//break;

			timeout.tv_sec = 5; 
			timeout.tv_usec = 0; 
			max_fd = sock_fd; 

			for (i = 0; i < MAX_CLIENT_NUM; i++) 
			{ 
				if (max_fd < client_fd[i]) 
				{ 
					max_fd = client_fd[i]; 
				}
			}

			select_read_set = read_set; 
			ret = select(max_fd + 1, &select_read_set, NULL, NULL, &timeout); 


			if (ret == 0)
			{ 
				//printf("timeout\n");
				count++;
				//printf("count:%d\n", count);
			}
			else if (ret < 0) 
			{ 
				printf("error occur\n"); 
				count = 0;
			} 
			else 
			{ 
				count = 0;
				if (FD_ISSET(sock_fd, &select_read_set)) 
				{ 
					printf("new client comes\n");
					len = sizeof(cli_addr); 
					new_conn_fd = accept(sock_fd, (struct sockaddr*)&cli_addr, &len);
					if (new_conn_fd < 0) 
					{ 
						perror("Fail to accept"); 
						exit(1);
					} 
					else 
					{ 
						for (i = 0; i < MAX_CLIENT_NUM; i++)
						{
							if (client_fd[i] == -1)
							{ 
								client_fd[i] = new_conn_fd; 
								TCP_Accept_CMD_fd = new_conn_fd;//***************
								FD_SET(new_conn_fd, &read_set);
								break; 
							} 
							if (max_fd < new_conn_fd)
							{
								max_fd = new_conn_fd; 
							} 
						}
		 			} 
				} 
				else 
				{ 
					for (i = 0; i < MAX_CLIENT_NUM; i++) 
					{ 
						if (-1 == client_fd[i]) 
						{
		 					continue; 
						} 
						memset(recv_buf, 0, MAX_RECV_LEN); 
						if (FD_ISSET(client_fd[i], &select_read_set)) 
						{ 
							unsigned int Sizeof_MSG = 0;
							num = read(client_fd[i], recv_buf, 4);
							if(num == 4)
							{
								Sizeof_MSG = recv_buf[0] + recv_buf[1]*256 + recv_buf[2]*256*256 + recv_buf[3]*256*256*256;
								//printf("recv_buf[0] = %d recv_buf[1]=%d recv_buf[2]=%d  recv_buf[3]=%d \n",recv_buf[0],recv_buf[1],recv_buf[2],recv_buf[3]);
								//printf("Sizeof_MSG = %d \n",Sizeof_MSG);
								if((Sizeof_MSG <= 1020) && (Sizeof_MSG > 0))
								{
									num = read(client_fd[i], recv_buf+4, Sizeof_MSG-4);
									//printf("num = %d *******\n",num);
									if(num == (Sizeof_MSG-4))
									{
										Receive_CMD(recv_buf,num+4);//********************
										Analysis_Msg();
									}
								}
							}
							if (num < 0)
							{
								printf("Client(%d) left\n", client_fd[i]); 
								FD_CLR(client_fd[i], &read_set);
								close(client_fd[i]);
								client_fd[i] = -1; 
								TCP_Accept_CMD_fd = -1;
							} 
							else if (num > 0) 
							{ 
								//printf("Recieve client(%d) data\n", client_fd[i]); 
								#if 0
								int i;				
								printf("numLen : %d \n date:",num);
								for(i=0; i<num+4; i++)
									printf("%02x", recv_buf[i]);
								printf("\n");
								//Receive_CMD(recv_buf,num);//********************
								//Analysis_Msg();
								#endif
							} 
							if (num == 0)
							{ 
								printf("Client(%d) exit\n", client_fd[i]);
								FD_CLR(client_fd[i], &read_set);
								close(client_fd[i]);
								client_fd[i] = -1; 
								TCP_Accept_CMD_fd = -1;
							} 
						} 
					} 
				} 
			} 
		} 
		close(sock_fd);
	}
	return NULL; 
}
*/
int socketCon;

#define MAX_TEXT 512  
struct msg_st  
{  
    long int msg_type;  
    char text[MAX_TEXT];  
};

struct msg_st data;

int msgid = -1;
long int msgtype = 0; //注意1

void* socket_process(void *ptr)
{  
	cpu_set_t cpuset;
	pthread_t thread = pthread_self();
	int s,core_no = 0;
	CPU_ZERO(&cpuset);
	CPU_SET(core_no,&cpuset);
	s = pthread_setaffinity_np(thread,sizeof(cpuset),&cpuset);
	if(s != 0)
		printf("Set pthread_setaffinity_npfailed\n");

    int isconnect;

    fd_set readfds,writefds;
	char buffer[MAX_TEXT];
	time_t timet;
	struct timeval tv;

	int msgid = -1;  
	int msgid2 = -1;//*((int *)msg_fd);

	while(1)
	{		
		printf("开始socket\n");  
		/* 创建TCP连接的Socket套接字 */  
		socketCon = socket(AF_INET, SOCK_STREAM, 0);  
		if(socketCon < 0)
		{  
			printf("创建TCP连接套接字失败\n");  
			exit(-1);  
		}  
		/* 填充客户端端口地址信息，以便下面使用此地址和端口监听 */  
		struct sockaddr_in server_addr;  
		bzero(&server_addr,sizeof(struct sockaddr_in));  
		server_addr.sin_family = AF_INET;  
		server_addr.sin_addr.s_addr = inet_addr("192.168.1.1");  
		server_addr.sin_port = htons(6767);  
		printf("连接之前的socketCon:%d",socketCon);  
	
		/* 连接服务器 */  
		int res_con = connect(socketCon,(struct sockaddr *)(&server_addr),sizeof(struct sockaddr));  
		if(res_con != 0)
		{  
			printf("连接失败,等待3秒后继续连接......\n");
			// 关闭套接字	
			close(socketCon); 
			sleep(3);			
			continue;  
		}  
		printf("连接成功\n");  
		//开启新的实时接受数据线程			
		isconnect = 1;
		
		/* 实时发送数据 */  
		while(1)
		{  
			if(isconnect == 0)
				break;
	
			FD_ZERO(&readfds);
			FD_ZERO(&writefds);
			FD_SET(socketCon, &readfds);
			FD_SET(socketCon, &writefds);
			tv.tv_usec = 0;
			tv.tv_sec = 60;
			int ret;
			ret = select(socketCon+1, &readfds, NULL, NULL, &tv);
			switch(ret)
			{
				case 0:
					printf("select timeout!\n");
					break;
				case -1:
					perror("select return failed!\n");
					isconnect = 0;
				default:
					if(FD_ISSET(socketCon, &readfds) > 0)
					{
						memset(buffer, 0, MAX_TEXT);
						time(&timet);
						int readlen;
						readlen = read(socketCon, buffer, MAX_TEXT);
						printf("readlen:%d\n", readlen);
						if(readlen <= 0)
						{
							perror("read data failed!\n");
							isconnect = 0;
							break;
						}
						else
						{	
							int ret;
							//ret = Command_Analysis(socketCon, buffer, readlen);
						
						
							printf("%d-->:\n", readlen);
							if(buffer[0] == 0x68)
							{
								printf("buffer[0]:%x\n", buffer[0]);
							}
							unsigned char i;
							for(i=0; i<25; i++)
								printf("0x=%x ", buffer[i]);
							printf("\n");
							if((buffer[0] == 0x68) && (buffer[2] == 0x68) && (buffer[13] == 0x16))
							{
								int x ;

								tar_x1  = (buffer[5]<<8) + buffer[4];
								tar_y1  = (buffer[7]<<8) + buffer[6];
							    tar_x2	= (buffer[9]<<8) + buffer[8];
							    tar_y2	= (buffer[11]<<8) + buffer[10];
								printf("tar_x1 = %d ,tar_y1 = %d, tar_x2 = %d, tar_y2 = %d \n",tar_x1,tar_y1,tar_x2,tar_y2);
								if(buffer[3] == 1)//跟踪框
								{
									if((tar_x1!=rcInit.left)||(tar_y1!=rcInit.top)||(tar_x2!=rcInit.right)||(tar_y2!=rcInit.bottom))
									{
										if((tar_x1+5<tar_x2)&&(tar_y1+5<tar_y2))
										{
											Track_Begin_Flag = 0;         //切换成手动模式
											switch(Current_Cmd_State)
											{
												//NCC***********************************************
												case NCC_Algorithm_Track:        
													    usleep(40000);          //等待跟踪算法结束，保持在手动状态
														Rec_Target_x = (tar_x1+tar_x2)/2;
														Rec_Target_y = (tar_y1+tar_y2)/2;
														printf("Rec_Target_x = %d ,Rec_Target_y = %d ******\n",Rec_Target_x,Rec_Target_y);
													break;
													
													//QS********************************************
												case QSImage_Algorithm_Track:
														usleep(40000);          //等待跟踪算法结束，保持在手动状态
														qsObjectTrackStop();	 //停止跟踪器
														rcInit.left = tar_x1;	 //选中区域
														rcInit.right = tar_x2;
														rcInit.top = tar_y1;
														rcInit.bottom = tar_y2;
													break;
													
													//TLD********************************************
												case TLD_Algorithm_Track:
														if(Frame_Num_Track != 0)
														{
															TLD_Pthread_destory();
														}
														
														usleep(400000);          //等待跟踪算法结束
													    Rec_Target_x = (tar_x1+tar_x2)/2;
														Rec_Target_y = (tar_y1+tar_y2)/2;
														TLD_Pthread_create();
													break;
													
												default:
													break;
													
											}
											Frame_Num_Track = 0;                  //
											printf("*************接收到新的跟踪区域*************\n");
											mubiaodiushi = 1;
											Track_Begin_Flag = 1;         //切换成跟踪模式
										}
									}
								}
								else
								{
									Track_Begin_Flag = 0;         //切出跟踪模式
									if(buffer[3] == 2)
										Current_Cmd_State = NCC_Algorithm_Track;
									else if(buffer[3] == 3)
										Current_Cmd_State = QSImage_Algorithm_Track;
									else if(buffer[3] == 4)
										Current_Cmd_State = TLD_Algorithm_Track;
									else if(buffer[3] == 5)//可见光
									{
									
									}
									else if(buffer[3] == 6)//红外
									{
										
									}
									Frame_Num_Track = 0; 
								}
							
							}
							
							data.msg_type = 1;
							memcpy(data.text, buffer, readlen); 
							msgsnd(msgid, (void*)&data, MAX_TEXT, 0);
						}
					}	
			}
		}
		close(socketCon);  
	}
	//return 0;  
}


int main(int argc, char **argv)
{
	int ret;
	unlink("pipe_filename");
	mkfifo("pipe_filename", 0777);
	
		
//初始化各个模块设备************************************************			
	if(Uart_Init(&UART1_fd, "/dev/ttymxc1", 115200) < 0) //串口1初始化	
	{
		exit(1);
	}

	if(Uart_Init(&UART2_fd, "/dev/ttymxc2", 115200) < 0) //串口2初始化	
	{
		exit(1);
	}
	
	if(Uart_Init(&UART3_fd, "/dev/ttymxc3", 115200) < 0) //串口3初始化	
	{
		exit(1);
	}

//*************************************************切换视频********
#if 1
	int i;
	unsigned char ACK[11] = {0,1,2,3,4,5,6,7,8,9,10};
	for(i=0; i<11; i++)
	{
		//ACK[i] = 0;
	}
	write(UART3_fd, ACK, 11);
#endif
//******************************************************************	

//创建线程**********************************************************
	ret = pthread_create(&UART2_tid, NULL, UART2_process, NULL); 
	if(ret != 0)
		printf ("Create UART2_tid pthread error!\n");

	ret = pthread_create(&UART1_tid, NULL, UART1_process, NULL); 
	if(ret != 0)
		printf ("Create UART1_tid pthread error!\n");
	
	ret = pthread_create(&camera_tid, NULL, camera_process, NULL);
	if(ret!=0)
		printf ("Create camera pthread error!\n");
	
	ret = pthread_create(&Track_tid, NULL, tracking_process, NULL);
	if(ret!=0)
		printf ("Create tracking_process pthread error!\n");

	ret = pthread_create(&socket_tid, NULL, socket_process, NULL);
	if(ret!=0)
		printf ("Create socket_process pthread error!\n");

	for(;;)
	{
		rtmpStart();
		sleep(2);
	}

	pthread_join(socket_tid,NULL);
	pthread_join(camera_tid,NULL);
	pthread_join(Track_tid,NULL);
	pthread_join(UART2_tid,NULL);
	pthread_join(UART1_tid,NULL);

	
	/*关闭文件描述符*/ 
	close(UART3_fd);
	close(UART2_fd);
	close(UART1_fd);
			
	return 0;
}




