/*
 * Copyright 2004-2013 Freescale Semiconductor, Inc.
 *
 * Copyright (c) 2006, Chips & Media.  All rights reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <getopt.h>
#include "vpu_test.h"

struct input_argument {
	int mode;
	pthread_t tid;
	char line[256];
	struct cmd_line cmd;
};

struct input_argument VPU_Camera_Enc;

int quitflag;

int vpu_test_dbg_level;

//extern int encode_test(void *arg);
extern void* encode_test(void *arg); //g++ :由于创建线程时'void*' to 'void* (*)(void*)' 不成功

int VPU_Init(void)
{
	int err = 0;
	vpu_versioninfo ver;

	info_msg("VPU test program built on %s %s\n", __DATE__, __TIME__);
	framebuf_init();

	err = vpu_Init(NULL);
	if (err) {
		err_msg("VPU Init Failure.\n");
		return -1;
	}

	err = vpu_GetVersionInfo(&ver);
	if (err) {
		err_msg("Cannot get version info, err:%d\n", err);
		vpu_UnInit();
		return -1;
	}

	info_msg("VPU firmware version: %d.%d.%d_r%d\n", ver.fw_major, ver.fw_minor,
						ver.fw_release, ver.fw_code);
	info_msg("VPU library version: %d.%d.%d\n", ver.lib_major, ver.lib_minor,
						ver.lib_release);
	return 0;

}

//**********VPU Capture图像分辨率****************
unsigned short VPU_Camera_WIDTH = 720;
unsigned short VPU_Camera_HEIGHT = 576;
//************VPU 压缩图像格式*******************
// -f <format> 0 - MPEG4, 1 - H.263, 2 - H.264, 7 - MJPG 
unsigned char VPU_Camera_Format = 2;
//****************帧率***1080P  **15FPS************
unsigned char VPU_Camera_FPS = 30;
unsigned short VPU_Camera_Resolution = 512;

char Output_FileDir[256] = "./Video_File/";

void VPU_Input_Init(unsigned short resolution, unsigned char VPU_Format)
{
	VPU_Camera_Enc.mode = ENCODE;
	VPU_Camera_Enc.cmd.chromaInterleave = 1;
	if (cpu_is_mx6x())
		VPU_Camera_Enc.cmd.bs_mode = 1;
	VPU_Camera_Enc.cmd.src_scheme = PATH_V4L2;
	VPU_Camera_Enc.cmd.dst_scheme = PATH_FILE;
	VPU_Camera_Enc.cmd.video_node_capture = 0;  //代表/dev/video0中的0
	//******************************************************
	//*********************输出文件*************************
	struct tm *local;
	time_t tt;
	tzset();//void tzset(void);设置时间环境变量-时区
	tt=time(NULL);//等价于time(&tt);
	local=localtime(&tt);
	char current[256];
	//printf("%4d年%02d月%02d日 %2d:%2d:%2d\n",local->tm_year+1900,local->tm_mon+1,local->tm_mday,local->tm_hour,local->tm_min,local->tm_sec);
    //sprintf(current, "%04d年%02d月%02d日%02d-%02d-%02d",local->tm_year+1900,local->tm_mon+1,local->tm_mday,local->tm_hour,local->tm_min,local->tm_sec);
	sprintf(current, "%04d%02d%02d_%02d-%02d-%02d",local->tm_year+1900,local->tm_mon+1,local->tm_mday,local->tm_hour,local->tm_min,local->tm_sec);

	char file_format[15];
	VPU_Camera_Format = VPU_Format;
	if(VPU_Camera_Format == 0)
		strcpy(file_format,".mpeg4");
	else if(VPU_Camera_Format == 7)
		strcpy(file_format,".mjpg");
	else if(VPU_Camera_Format == 2)
		strcpy(file_format,".h264");
	else{
			printf("没有设置压缩格式，默认H.264\n");
			strcpy(file_format,".h264");
		}

	strcat(current,file_format);
	strcat(Output_FileDir,current);

	//用char nnnn[20]定义的，用strcpy 
	//用char *ppp定义的，用=来赋值
	strcpy(VPU_Camera_Enc.cmd.output,Output_FileDir);
	//******************************************************
	VPU_Camera_Enc.cmd.format = VPU_Camera_Format; //-f <format> 0 - MPEG4, 1 - H.263, 2 - H.264, 7 - MJPG 
	VPU_Camera_Enc.cmd.bitrate = 8000;// 0 : Default is auto 自适应
	VPU_Camera_Enc.cmd.gop = 5;
	//图像旋转180度，但表示压缩以后的图像，捕获的图像还是原始图
	#if 0
		VPU_Camera_Enc.cmd.rot_en = 1;    //VPU旋转使能
		VPU_Camera_Enc.cmd.rot_angle = 180;
	#endif
	//图像镜像，但表示压缩以后的图像，捕获的图像还是原始图
	//1表示上下镜像，2表示左右镜像，3表示上下左右镜像
	#if 0
		if(resolution == 512)
			VPU_Camera_Enc.cmd.rot_en = 1;    //VPU镜像使能
		else
			VPU_Camera_Enc.cmd.rot_en = 0;    //VPU镜像使能
		VPU_Camera_Enc.cmd.mirror = 1;
	#endif
	
	VPU_Camera_Resolution = resolution;
	if(VPU_Camera_Resolution == 1080)
	{
		VPU_Camera_FPS = 15;             //****帧率***1080P  **15FPS
		VPU_Camera_WIDTH = 1920;
		VPU_Camera_HEIGHT = 1080;
	}
	else if(VPU_Camera_Resolution == 512)
	{
		VPU_Camera_FPS = 30;
		VPU_Camera_WIDTH = 640;
		VPU_Camera_HEIGHT = 512;
	}
	else if(VPU_Camera_Resolution == 576)
	{
		VPU_Camera_FPS = 30;
		VPU_Camera_WIDTH = 720;
		VPU_Camera_HEIGHT = 576;
	}
	else if(VPU_Camera_Resolution == 544)
	{
		VPU_Camera_FPS = 30;
		VPU_Camera_WIDTH = 960;
		VPU_Camera_HEIGHT = 544;
	}
	else if(VPU_Camera_Resolution == 480)
	{
		VPU_Camera_FPS = 30;
		VPU_Camera_WIDTH = 640;
		VPU_Camera_HEIGHT = 480;
	}
	else if(VPU_Camera_Resolution == 720)
	{
		VPU_Camera_FPS = 30;
		VPU_Camera_WIDTH = 1280;
		VPU_Camera_HEIGHT = 720;
	}
	else// VPU_Camera_Resolution默认720*576
	{
		printf("没有设置图像分辨率，默认720*576!\n");
		VPU_Camera_FPS = 30;
		VPU_Camera_WIDTH = 720;
		VPU_Camera_HEIGHT = 576;
	}	
	VPU_Camera_Enc.cmd.fps = VPU_Camera_FPS;
	VPU_Camera_Enc.cmd.width = VPU_Camera_WIDTH;
	VPU_Camera_Enc.cmd.height = VPU_Camera_HEIGHT;
	
	//create_file(output_filedir);
	//打开需要存入视频的文件
	//open_files(&VPU_Camera_Enc.cmd);
}

int VPU_camera(unsigned short resolution, unsigned char VPU_Format)
{
	VPU_Init();
	VPU_Input_Init(resolution,VPU_Format);
	
	//pthread_create(&VPU_Camera_Enc.tid,NULL,(void *)&encode_test,(void *)&VPU_Camera_Enc.cmd);
	pthread_create(&VPU_Camera_Enc.tid,NULL, encode_test, (void *)&VPU_Camera_Enc.cmd);//g++ : 'void*' to 'void* (*)(void*)'
		
	if (VPU_Camera_Enc.tid != 0)
		pthread_join(VPU_Camera_Enc.tid, NULL);
	
	return 1;         
}



#ifdef _FSL_VTS_
#include "dut_api_vts.h"


#define MAX_CMD_LINE_LEN    1024

typedef struct _tagVideoDecoder
{
    unsigned char * strStream;          /* input video stream file */
    int             iPicWidth;          /* frame width */
    int             iPicHeight;         /* frame height */
    DUT_TYPE        eDutType;           /* DUT type */
    FuncProbeDut    pfnProbe;           /* VTS probe for DUT */
} VDECODER;


FuncProbeDut g_pfnVTSProbe = NULL;
unsigned char *g_strInStream = NULL;


DEC_RETURN_DUT VideoDecInit( void ** _ppDecObj, void *  _psInitContxt )
{
    DEC_RETURN_DUT eRetVal = E_DEC_INIT_OK_DUT;
    DUT_INIT_CONTXT_2_1 * psInitContxt = _psInitContxt;
    VDECODER * psDecObj = NULL;

    do
    {
        psDecObj = malloc( sizeof(VDECODER) );
        if ( NULL == psDecObj )
        {
            eRetVal = E_DEC_INIT_ERROR_DUT;
            break;
        }
        *_ppDecObj = psDecObj;
        psDecObj->pfnProbe   = psInitContxt->pfProbe;
        psDecObj->strStream  = psInitContxt->strInFile;
        psDecObj->iPicWidth  = (int)psInitContxt->uiWidth;
        psDecObj->iPicHeight = (int)psInitContxt->uiHeight;
        psDecObj->eDutType   = psInitContxt->eDutType;

        g_pfnVTSProbe = psInitContxt->pfProbe;
        g_strInStream = psInitContxt->strInFile;
    } while (0);

    return eRetVal;
}

DEC_RETURN_DUT VideoDecRun( void * _pDecObj, void * _pParam )
{
    VDECODER * psDecObj = (VDECODER *)_pDecObj;
    char * strCmdLine = NULL;
    char * argv[3];
    int argc = 3;
    int iDecID = 0;
    int iMPEG4Class = 0;
    DEC_RETURN_DUT eRetVal = E_DEC_ALLOUT_DUT;

    do
    {
        strCmdLine = malloc( MAX_CMD_LINE_LEN );
        if ( NULL == strCmdLine )
        {
            fprintf( stderr, "Failed to allocate memory for command line." );
            eRetVal = E_DEC_ERROR_DUT;
            break;
        }

        argv[0] = "./mxc_vpu_test.out";
        argv[1] = "-D";
        argv[2] = strCmdLine;

        /* get decoder file name */
        switch (psDecObj->eDutType)
        {
        case E_DUT_TYPE_H264:
            iDecID = 2;
            break;
        case E_DUT_TYPE_DIV3:
            iDecID = 5;
            break;
        case E_DUT_TYPE_MPG4:
            iMPEG4Class = 0;
            break;
        case E_DUT_TYPE_DIVX:
        case E_DUT_TYPE_DX50:
            iMPEG4Class = 1;
            break;
        case E_DUT_TYPE_XVID:
            iMPEG4Class = 2;
            break;
        case E_DUT_TYPE_DIV4:
            iMPEG4Class = 5;
            break;
        case E_DUT_TYPE_MPG2:
            iDecID = 4;
            break;
        case E_DUT_TYPE_RV20:
        case E_DUT_TYPE_RV30:
        case E_DUT_TYPE_RV40:
        case E_DUT_TYPE_RV89:
            iDecID = 6;
            break;
        case E_DUT_TYPE_WMV9:
        case E_DUT_TYPE_WMV3:
        case E_DUT_TYPE_WVC1:
            iDecID = 3;
            break;
        default:
            perror( "DUT type %d is not supported.\n" );
            iDecID = -1;
        }

        if ( -1 == iDecID )
        {
            eRetVal = E_DEC_ERROR_DUT;
            break;
        }

        /* run decoder */
        sprintf( strCmdLine, "-i %s -f %d -l %d -o vts",
                 psDecObj->strStream, iDecID, iMPEG4Class );

        vputest_main(argc, argv);
    } while (0);

    if ( strCmdLine )
    {
        free( strCmdLine );
    }

    return eRetVal;
}

DEC_RETURN_DUT VideoDecRelease( void * _pDecObj )
{
    DEC_RETURN_DUT eRetVal = E_DEC_REL_OK_DUT;

    if ( _pDecObj )
    {
        free( _pDecObj );
    }

    return eRetVal;
}

void QueryAPIVersion( long * _piAPIVersion )
{
    *_piAPIVersion = WRAPPER_API_VERSION;
}
#endif
