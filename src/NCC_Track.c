#include "NCC_Track.h"
#include <math.h>
#include "string.h"
#include "stdlib.h"
//#include "camera.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
//#include "struct_defined.h"
int track_mubiaodiushi=0;

/**************************预处理命令********************/
#define  SDI                     0x01
#define  PAL                     0x02
#define  TM_EPSILON   0.000001
#define  TM_MAX(a,b)  ((a) < (b) ? (b) : (a))
#define  TM_MIN(a,b)  ((a) > (b) ? (b) : (a))

#define NUM_SAMPLES 20		//每个像素点的样本个数
#define MIN_MATCHES 2		//#min指数
#define RADIUS 80			//Sqthere半径
#define SUBSAMPLE_FACTOR 16	//子采样概率
#define LIFELENGHT 20		// 清除场景中存在时间较长的像素，用于控制允许存在的最长时间20

///***************************内存地址*********************/
//unsigned char *pSrc1        ;
unsigned char *Template     ;//指定模板存储位置,模板大小为64*64:0x1000，预留0x3000
unsigned char *Template2    ;//指定备份模板存储位置,备份模板为64*64:0x1000，预留0x3000
unsigned char *gSubImageData;//读子图像,大小为256*256:0x10000
int           *gSCorrData   ;//最终的相关系数值，大小为128*128*4:0x10000
int           *gSum         ;//图像积分图，大小为129*129*4：0x10404
long long     *gSqsum       ;//图像平方积分图,大小为129*129*8：0x20808
unsigned char *TmpCache     ;//缓存模板，用于全屏匹配，大小32*32:0x0400，预留0x0B00
unsigned char *eImage       ;//缓存图像，用于全屏匹配，大小128*128:0x4000
unsigned char *gSImageData  ;//搜索区域图像，大小为128*128:0x4000

/*******************************************************************/

/***************************全局变量********************************/

unsigned short tar_x=0,tar_y=0;//目标在搜索子图像中的坐标
int TmpW=64,TmpH=64;     //模板宽度、模板高度
int schWidth=128;//搜索区域宽度
int schHeight=128;//搜索区域高度
//int TmpW=32,TmpH=32;
//int schWidth=64;
//int schHeight=64;
int gSubImageWidth=0;//搜索子图像宽度
int gSubImageHeight=0;//搜索子图像高度

//int gSubImageWidth=1920;//������ͼ����
//int gSubImageHeight=1080;//������ͼ��߶�
int subimgleft = 0,subimgright = 0,subimgtop = 0,subimgbottom = 0;
int cnt_track = 0;

// 模板宽度、高度，该数值由外部确定
static int gTemplWidth  = 0;
static int gTemplHeight = 0 ;
static int gTemplPreRow = 0 ; // 模板在前一帧中起点的行坐标
static int gTemplPreCol = 0 ; // 模板在前一帧中起点的列坐标
static int gTemplCurRow = 0 ; // 模板在当前帧中起点的行坐标
static int gTemplCurCol = 0 ; // 模板在当前帧中起点的列坐标
static float gConfiLevel  = 0 ; //当前匹配的可信度


static unsigned char *gTemplData= 0;// 执行模板的数据，其值由外部确定
static unsigned char *gTemplBuf	= 0;
static unsigned char *gImageData= 0;// 指向图像数据的指针，其初始值由外部确定
static unsigned char *gImageBuf	= 0;
static unsigned int * iResult= 0;

unsigned int trackdata[200];
static int trackstate;
static int tracklevel;

int cnt_trackon=0;
int cnt_trackoff=0;
static int track_x=0;//模板中心列坐标
static int track_y=0;//模板中心行坐标

static float f32_nccRatio=0.01f;
static float f32_avrRatio=0.01f;
static float f32_trackRatio=0.01f;
static float f32_allRatio[20]={0};
static float f32_avrgray=0.01f;
static float f32_avrgray2=0.01f;
static float f32_sigma=0.01f;
static float f32_sigma2=0.01f;




void Init_Addr(void)
{
	/***************************内存地址*********************/
	//pSrc1 = (unsigned char*)malloc(sizeof(char)* 256 * 256);
	Template = (unsigned char*)malloc(sizeof(char)* 64 * 64);//指定模板存储位置,模板大小为64*64:0x1000，预留0x3000
	Template2 = (unsigned char*)malloc(sizeof(char)* 64 * 64);//指定备份模板存储位置,备份模板为64*64:0x1000，预留0x3000
	//gSubImageData = (unsigned char*)malloc(sizeof(char)* 1920 * 1080);//读子图像,大小为256*256:0x10000
	gSCorrData = (int*)malloc(sizeof(int)* 128 * 128);// 最终的相关系数值，大小为128*128*4:0x10000
	gSum = (int*)malloc(sizeof(int)* 129 * 129); // 图像积分图，大小为129*129*4：0x10404
	gSqsum = (long long*)malloc(sizeof(long long)* 129 * 129); //图像平方积分图,大小为129*129*8：0x20808
	TmpCache = (unsigned char*)malloc(sizeof(char)* 32 * 32);//缓存模板，用于全屏匹配，大小32*32:0x0400，预留0x0B00
	eImage = (unsigned char*)malloc(sizeof(char)* 128 * 128);//缓存图像，用于全屏匹配，大小128*128:0x4000
	gSImageData = (unsigned char*)malloc(sizeof(char)* 128 * 128);//搜索区域图像，大小为128*128:0x4000

	
	//根据图像情况选择合适的尺寸
	//gSubImageWidth   = 640;//搜索子图像宽度
	//gSubImageHeight  = 480;//搜索子图像高度
}

void UnInit_Addr(void)
{
	/***************************内存地址*********************/
	free(Template);
	free(Template2);
	//free(gSubImageData);
	free(gSCorrData);
	free(gSum);
	free(gSqsum);
	free(TmpCache);
	free(eImage);
	free(gSImageData);
}

/////////////////////////////////////////////////////////////////////

void achieveTemplate(
	unsigned char *pSrc,	//输入图像数据
	unsigned char *pDst,	//输出模板数据
	int img_width,			//图像宽度
	int img_height,			//图像高度
	int tmp_width,			//模板宽度
	int tmp_height,			//模板高度
	unsigned short col,		//取模板中心位置的列坐标
	unsigned short row)		//取模板中心位置的行信息
{
	int i,j,k;
	unsigned char *pTmpSrc;
	//*************************************************
	if(col < tmp_width/2)
	{
		col = tmp_width/2;
	}
	else if(col > img_width-tmp_width/2)
	{
		col = img_width-tmp_width/2;
	}
	//**************************************************
	if(row < tmp_height/2)
	{
		row = tmp_height/2;
	}
	else if(row > img_height-tmp_height/2)
	{
		row = img_height-tmp_height/2;
	}
	//**************************************************
	for(i=row-tmp_height/2,k=0; i<row+tmp_height/2; i++)
	{
		pTmpSrc	= pSrc + i*img_width;
		for(j=col-tmp_width/2; j<col+tmp_width/2; j++)
		{
			pDst[k] = pTmpSrc[j];
			k++;
		}
	}
}

// 0xE0023608
// 直接计算相关系数
// 输入参数： img   - SEARCHHEIGHT*SEARCHWIDTH = 64*128
//            templ - gTemplHeight*gTemplWidth
// 输出参数： corr  - SEARCHHEIGHT*SEARCHWIDTH = 64*128
int crossCorr(unsigned char* img,unsigned char *templ,int *corr,int searchWidth,int searchHeight)
{
	int i,j,p,q ;
	int sum,*pTmpCorr ;
	unsigned char *pTmpTempl,*pTmpImg ;
	int hLimit,wLimit,hhLimit,wwLimit ;
	hLimit = searchHeight-gTemplHeight+1 ;
	wLimit = searchWidth-gTemplWidth+1 ;
	//之前已经清空过********************************************
	memset(corr,0,searchHeight*searchWidth*sizeof(int) ) ;
	for( i = 0 ; i < hLimit ; i++ )
	{
		pTmpCorr = corr + i*searchWidth ;
		for( j = 0 ; j < wLimit ; j++ )
		{
			sum = 0 ;
			hhLimit = i+gTemplHeight ;
			wwLimit = j+gTemplWidth ;
			for( p = i ; p <  hhLimit ; p++ )
			{
				pTmpImg   = img + p*searchWidth ;
				pTmpTempl = templ + (p-i)*gTemplWidth ;
				for( q = j ; q < wwLimit ; q++ )
				{
					sum += ((int)pTmpImg[q])*(int)pTmpTempl[(q-j)] ;
				}
			}
			pTmpCorr[j] = sum ;
		}
	}
	return 0 ;
}

// 计算图像的积分图和平方积分图，假设图像数据的存储是连续的。
// 输入参数 ： img - SEARCHWIDTH*SEARCHHEIGHT的图像
// 输出参数 ： sum - (SEARCHWIDTH+1)*(SEARCHHEIGHT+1)的图像
//		  sqsum - (SEARCHWIDTH+1)*(SEARCHHEIGHT+1)的图像

void integral(int searchWidth,int searchHeight)
{
	int    i,j,sumstep,sqstep,imgstep ;
	unsigned char *pImgTmp ;
	int    *pSumTmp ;
	long long *pSqsumTmp ;
	imgstep = searchWidth ;
	sumstep = searchWidth + 1 ;
	sqstep  = searchWidth + 1 ;
	// 第0行的数据置为0
	memset(gSum,0,(searchWidth+1)*(searchHeight+1)*sizeof(int)) ;
	memset(gSqsum,0,(searchWidth+1)*(searchHeight+1)*sizeof(long long)) ;
	// 积分图、平方积分图的计算
	pImgTmp   = gSImageData ;
	pSumTmp   = gSum ;
	pSqsumTmp = gSqsum ;
	for( i = 1 ; i < searchHeight+1 ; i++ )
	{
		pImgTmp  += imgstep ;
		pSumTmp  += sumstep ;
		pSqsumTmp+= sqstep  ;
		for( j = 1 ; j < searchWidth+1 ; j++ )
		{
			pSumTmp[j]   = pSumTmp[j-1]   + pSumTmp[j-sumstep]  - pSumTmp[j-sumstep-1]  + pImgTmp[j-1-imgstep];
			pSqsumTmp[j] = pSqsumTmp[j-1] + pSqsumTmp[j-sqstep] - pSqsumTmp[j-sqstep-1] +
						   (long long)pImgTmp[j-1-imgstep]*pImgTmp[j-1-imgstep] ;
		}
	}
}

// 计算sqrt(SIGMA((img(i,j)-mean(img)).^2))，即方差*像素个数,再开根号。
// 假设图像数据的存储是连续的。
// mean为图像的均值
void variance(float *mean,float *var)
{
	int i, length, sum ;
	length = gTemplWidth*gTemplHeight ;
	sum   = 0 ;
	// 计算图像的均值
	for( i = 0 ; i < length ; i++ )
	{
		sum += gTemplData[i] ;
	}
	*mean = (float)sum/length ;
	// 计算图像的方差
	*var = 0 ;
	for( i = 0 ; i < length ; i++ )
	{
		*var += (*mean-gTemplData[i])*(*mean-gTemplData[i]) ;
	}
	*var = sqrt(*var) ;
	return ;
}

// NCC匹配算法
// 备注 - 行索引与列索引为目标的中心坐标
float matchTemplate(int *newRow,int *newCol,int imgWidth,int imgHeight,int searchWidth,int searchHeight)
{
     static int left=0,right=0,top=0,bottom=0;// 定位搜索区域，[top,left]将作为偏置
	static int i,j ; // 循环变量
	static unsigned char *pSImage ; // 临时辅助变量
	static unsigned char *pImage   ; // 临时辅助变量，图像数据指针、模板数据指针
	static int templArea ; // 模板面积
	static int       *p0, *p1, *p2, *p3 ;// 图像积分图中，第一个滑动窗口的左上、右上、左下、右下点的位置
	static long long *q0, *q1, *q2, *q3 ;// 图像平方积分图中，第一个滑动窗口的左上、右上、左下、右下点的位置
	static int   sumstep, sqstep ;// 图像行长、图像积分图行长、图像平方积分图行长、结果行长
	static float templMean, templNorm ;  // 模板均值、NCC中分母的第二项
	static int   *rrow ;    // 指向结果某行的指针
	static int   idx,idx2 ; // 积分图、平方积分图的索引
	static float  num, t;    // 交叉系数或匹配可信度、式（4）的分母
	static double t_sum ;    // 子图像灰度和
	static double wndSum2 ; // 式（8）中的第1项
	static double wndMean2 ;// 式（8）中的第2项
	static int    hLimit,wLimit ;
	static float  confi ;  // 计算的NCC值
	
		// 1，确定搜索位置并提取搜索区域
	if( (gTemplPreCol-((searchWidth-gTemplWidth)>>1)) <= 0 )
	{ // 假如默认搜索窗超出了图像左边界
		left  = 0 ;
		right = left + searchWidth ;
	}
	else if( (gTemplPreCol+((searchWidth-gTemplWidth)>>1)+gTemplWidth) >= imgWidth )
	{  // 假如默认搜索窗超出了图像右边界
		right = imgWidth ;
		left  = imgWidth - searchWidth ;
	}
	else
	{// 假如默认搜索窗没有超出边界
		left  = gTemplPreCol - ((searchWidth-gTemplWidth)>>1);
		right = left + searchWidth ;
	}

	if( (gTemplPreRow-((searchHeight-gTemplHeight)>>1)) <= 0 )
	{
		top   = 0 ;
		bottom= top + searchHeight ;
	}
	else if( (gTemplPreRow+((searchHeight-gTemplHeight)>>1) + gTemplHeight) >= imgHeight )
	{
		bottom= imgHeight ;
		top   = bottom - searchHeight ;
	}
	else
	{
		top   = gTemplPreRow - ((searchHeight-gTemplHeight)>>1) ;
		bottom= top + searchHeight ;
	}
	// 提取图像的子图像
 	for(i = top ; i < bottom ; i++ )
 	{
		pSImage = gSImageData + (i-top)*searchWidth ;
		pImage  = (unsigned char *)gImageData  + i*imgWidth ;
		for( j = left ; j < right ; j++ )
		{
			pSImage[j-left] = pImage[j] ;
		}
	}

	memset( gSCorrData,0,searchWidth*searchHeight*sizeof(int) ) ;

	integral(searchWidth,searchHeight) ;
	variance(&templMean,&templNorm) ;
	// 计算相关系数
	crossCorr(gSImageData,gTemplData,gSCorrData,searchWidth,searchHeight);

	templArea  = gTemplWidth*gTemplHeight ; // 模板面积
	sumstep = searchWidth + 1 ; // 图像积分图谐ぁ?
	sqstep  = searchWidth + 1 ; // 图像平方积分图行长
	// 假如模板的2范数小于最小浮点数，说明模板图像中的灰度值全都一样，不能执行NCC计算,函数直接返回。
	if(templNorm < TM_EPSILON)
		return -1;

	q0 = gSqsum;           // 平方积分图的实际数据的首地址
	q1 = q0 + gTemplWidth; // 平方积分图的第一行与模板偏置的地址
	q2 = gSqsum + gTemplHeight*sqstep;
	q3 = q2 + gTemplWidth ;

	p0 = gSum ;
	p1 = p0  +  gTemplWidth ;
	p2 = gSum + gTemplHeight*sumstep ;
	p3 = p2  +  gTemplWidth ;

	hLimit = searchHeight-gTemplHeight+1 ;
	wLimit = searchWidth-gTemplWidth+1 ;
	gConfiLevel = -1 ;

	for(i=0; i<hLimit; i++)
	{
		// 指向结果图像的第i行的指针
		rrow = gSCorrData + i*searchWidth ;
		idx  = i * sumstep; // 积分图的索引位置
		idx2 = i * sqstep;  // 平方积分图的索引位置
		for( j = 0; j < wLimit; j++, idx++, idx2++ )
		{
			num   = rrow[j]; // (i,j)位置，图像与模板的交叉相关系数，参见式(7)中的第1项。

			// 计算式（8）中的第2项
			t_sum    = p0[idx] - p1[idx] - p2[idx] + p3[idx] ; // (i,j)位置，模板区域的灰度和

			wndMean2 =(double)t_sum*t_sum ; // 图像上，模板位置的灰度和的平方,wndMean2必须是long long型的或double型的，否则会溢出。
            wndMean2 = wndMean2/templArea;

			num -= t_sum*templMean ;// 参考式（7）中的第2项，此时的num为NCC的分子，即式（4）中的分子

			// 计算式（8）中的第1项
			wndSum2 = q0[idx2] - q1[idx2] - q2[idx2] + q3[idx2] ; // 图像上，模板位置的灰度平方和

			// 计算式（4）中的分母
			t = sqrt((double)(TM_MAX(wndSum2-wndMean2,0)))*templNorm; //templNorm是帜的刀?
			// 计算并保存NCC值

			if( abs(num) < t )
				confi = (double)num/t ;
			else if( abs(num) < t*1.125 )
				confi = num > 0 ? 1.0 : -1.0 ;
			else
				confi = 0.0 ;

			if( confi>gConfiLevel )
			{
				gConfiLevel   = confi ;
				gTemplCurRow  = i ;
				gTemplCurCol  = j ;
			}
		}
	}

	gTemplCurRow += top  ;//top表示匹配位置的上方位置
	gTemplCurCol += left ;//left表示匹配位置的左方位置
	*newRow = gTemplCurRow + gTemplHeight/2;//top表示匹配位置的上方位置
	*newCol = gTemplCurCol + gTemplWidth/2;
	return  gConfiLevel;
}

////////////////////////////////////////////////////////////////////////////////////////////
//求图像中指定位置的自适应阈值,平均灰度及均方差,输入参数错误返回-1.
//Author:LongTao
////////////////////////////////////////////////////////////////////////////////////////////
int TempThreshold(
	unsigned char *pSrc,	//输入图像数据
	int	srcWidth,			//图像宽度
	int srcHeight,			//图像高度
	float *srcAvr,			//返回平均灰度
	float *srcSigma)		//返回均方差
{
	int i,j;
	volatile float sigma,sumavr;//,sumavr2;
	volatile float thresh;
	int sum,area;
	unsigned char *pTmpGray;

	if((srcWidth==0) ||(srcHeight==0))
	{
		return -1;
	}

	for(i=0,sum=0; i<srcHeight; i++)
	{
		pTmpGray = pSrc + i*srcWidth;
		for(j=0; j<srcWidth; j++)
		{
			sum	+= pTmpGray[j];
		}
	}
	area	= srcWidth * srcHeight;
	sumavr	= sum/area;
	//sumavr2	= sumavr*sumavr;
	for(i=0,sum=0; i<srcHeight; i++)
	{
		pTmpGray = pSrc + i*srcWidth;
		for(j=0; j<srcWidth; j++)
		{
			sum	+= (pTmpGray[j]-sumavr)*(pTmpGray[j]-sumavr);
		}
	}
	sigma	= sqrt(sum/area);
	thresh	= sumavr+1.5*sigma;

	*srcAvr	= sumavr;
	*srcSigma=sigma;

	return thresh;
}

void init_NCC()
{
	gTemplHeight= TmpH;
	gTemplWidth = TmpW;
	gTemplPreRow= tar_y-TmpH/2;
	gTemplPreCol= tar_x-TmpW/2;
	gImageData	= (unsigned char *)gSubImageData;
	gImageBuf	= (unsigned char *)eImage;
	gTemplData	= (unsigned char *)Template;
	gTemplBuf	= (unsigned char *)TmpCache;
    iResult	    = (unsigned int *)trackdata;
}

////////////////////////////////////////////////////////////////////////////////////////////
//跟踪器初始化
//Author:LongTao
////////////////////////////////////////////////////////////////////////////////////////////
void TrackInit(unsigned short *x,unsigned short *y)
{
	track_x = *x;
	track_y = *y;
	init_NCC();
	cnt_track 	= cnt_trackon = cnt_trackoff = trackstate = 0;
	if(tracklevel == 2)
	{
		f32_trackRatio = 0.35;
	}
	else
	{
		f32_trackRatio = 0.55;
	}
}

void setNCCImagePtr(unsigned char *img)
{
	gImageData	= img;
}

void setNCCTemplPtr(unsigned char *img,int width,int height)
{
	gTemplData	= img;
	gTemplWidth = width;
	gTemplHeight= height;
}

////////////////////////////////////////////////////////////////////////////////////////////
//求图像中指定区块自适应阈值,平均灰度及均方差,输入边界有误时返回-1.
//Author:LongTao
///////////////////////////////////////////////////////////////////////////////////////
int RectThreshold(
	unsigned char *pSrc,	//输入图像数据
	int srcWidth,			//图像宽度
	int srcHeight,			//图像高度
	int top,				//计算区块上边界
	int bottom,				//计算区块下边界
	int left,				//计算区块左边界
	int right,				//计算区块右边界
	float *srcAvr,			//返回图像区块平均灰度
	float *srcSigma)		//返回图像区块均方差
{
	int i,j;
	volatile float sigma,sumavr;//,sumavr2;
	volatile float thresh;
	int sum,area;
	unsigned char *pTmpGray;

	if((srcWidth==0)||(srcHeight==0)||(top>bottom)||(left>right))
	{
		return -1;
	}

	if(top < 0)
		top = 0;
	if(left <0)
		left = 0;
	if(bottom > srcHeight)
		bottom = srcHeight;
	if(right  > srcWidth)
		right  = srcWidth;

	for(i=top,sum=0; i<bottom; i++)
	{
		pTmpGray = pSrc + i*srcWidth;
		for(j=left; j<right; j++)
		{
			sum	+= pTmpGray[j];
		}
	}
	area	= (right-left)*(bottom-top);
	sumavr	= sum/area;
	//sumavr2	= sumavr*sumavr;
	for(i=top,sum=0; i<bottom; i++)
	{
		pTmpGray = pSrc + i*srcWidth;
		for(j=left; j<right; j++)
		{
			sum	+= (pTmpGray[j]-sumavr)*(pTmpGray[j]-sumavr);
		}
	}
	sigma	= sqrt(sum/area);
	*srcAvr	= sumavr;
	*srcSigma=sigma;
	thresh	= sumavr+1.5*sigma;

	return thresh;
}

// NCC匹配算法
// 备注 - 行索引与列索引为目标的中心坐标
float MatchUsingNCC(int *newRow,int *newCol,int imgWidth,int imgHeight,int searchWidth,int searchHeight)
{
    static int left=0,right=0,top=0,bottom=0;// 定位搜索区域，[top,left]将作为偏置
	static int i,j,iNum,iTotal; // 循环变量
	static unsigned char *pSImage ; // 临时辅助变量
	static unsigned char *pImage   ; // 临时辅助变量，图像数据指针、模板数据指针
	static int templArea ; // 模板面积
	static int       *p0, *p1, *p2, *p3 ;// 图像积分图中，第一个滑动窗口的左上、右上、左下、右下点的位置
	static long long *q0, *q1, *q2, *q3 ;// 图像平方积分图中，第一个滑动窗口的左上、右上、左下、右下点的位置
	static int sumstep, sqstep ;// 图像行长、图像积分图行长、图像平方积分图行长、结果行长
	static float templMean, templNorm ;  // 模板均值、NCC中分母的第二项
	static int *rrow ;    // 指向结果某行的指针
	static int   idx,idx2 ; // 积分图、平方积分图的索引
	static float num, t;    // 交叉系数或匹配可信度、式（4）的分母
	static double   t_sum ;    // 子图像灰度和
	static double wndSum2 ; // 式（8）中的第1项
	static double wndMean2 ;// 式（8）中的第2项
	static int hLimit,wLimit ;
	static float confi ;  // 计算的NCC值


	//gTemplPreCol、gTemplPreRow、gTemplWidth、gTemplHeight等参数在 init_NCC()中赋值
	// 1，确定搜索位置并提取搜索
	if( (gTemplPreCol-((searchWidth-gTemplWidth)>>1)) <= 0 )
	{ // 假如默认搜索窗超出了图像左边界
		left  = 0 ;
		right = left + searchWidth ;
	}
	else if( (gTemplPreCol+((searchWidth-gTemplWidth)>>1)+gTemplWidth) >= imgWidth )
	{ // 假如默认搜索窗超出了图像右边界
		right = imgWidth ;
		left  = imgWidth - searchWidth ;
	}
	else
	{ // 假如默认搜索窗没有超出边界
		left  = gTemplPreCol - ((searchWidth-gTemplWidth)>>1);
		right = left + searchWidth ;
	}

	if( (gTemplPreRow-((searchHeight-gTemplHeight)>>1)) <= 0 )
	{
		top   = 0 ;
		bottom= top + searchHeight ;
	}
	else if( (gTemplPreRow+((searchHeight-gTemplHeight)>>1) + gTemplHeight) >= imgHeight )
	{
		bottom= imgHeight ;
		top   = bottom - searchHeight ;
	}
	else
	{
		top   = gTemplPreRow - ((searchHeight-gTemplHeight)>>1) ;
		bottom= top + searchHeight ;
	}
	for(i=0; i<100; i++)
	{
		iResult[i]	= 0;
	}
	// 提取图像的子图像
 	for( i = top ; i < bottom ; i++ )
 	{
		pSImage = gSImageData + (i-top)*searchWidth ;
		pImage  = (unsigned char *)gImageData  + i*imgWidth ;
		for( j = left ; j < right ; j++ )
		{
			pSImage[j-left] = pImage[j] ;
		}
	}
	memset( gSCorrData,0,searchWidth*searchHeight*sizeof(int)) ;
 	integral(searchWidth,searchHeight) ;
	variance(&templMean,&templNorm) ;

	// 计算相关系数
	crossCorr(gSImageData,gTemplData,gSCorrData,searchWidth,searchHeight);

	templArea  = gTemplWidth*gTemplHeight ; // 模板面积
	sumstep = searchWidth + 1 ; // 图像积分图谐ぁ?
	sqstep  = searchWidth + 1 ; // 图像平方积分图行长
	// 假如模板的2范数小于最小浮点数，说明模板图像中的灰度值全都一样，不能执行NCC计算,函数直接返回。
	if( templNorm < TM_EPSILON )
		return -1;

	q0 = gSqsum;           // 平方积分图的实际数据的首地址
	q1 = q0 + gTemplWidth; // 平方积分图的第一行与模板偏置的地址
	q2 = gSqsum + gTemplHeight*sqstep;
	q3 = q2 + gTemplWidth ;

	p0 = gSum;
	p1 = p0  +  gTemplWidth ;
	p2 = gSum + gTemplHeight*sumstep;
	p3 = p2  +  gTemplWidth ;

	hLimit	= searchHeight-gTemplHeight+1 ;
	wLimit	= searchWidth-gTemplWidth+1 ;
	gConfiLevel = -1 ;
	iNum = iTotal = 0;
	for(i=0; i<hLimit; i++)
	{
		// 指向结果图像的第i行的指针
		rrow = gSCorrData + i*searchWidth ;
		idx  = i * sumstep; // 积分图的索引位置
		idx2 = i * sqstep;  // 平方积分图的索引位置
		for( j = 0; j <  wLimit; j++, idx++, idx2++ )
		{
			num   = rrow[j]; // (i,j)位置，图像与模板的交叉相关系数，参见式(7)中的第1项。

			// 计算式（8）中的第2项
			t_sum    = p0[idx] - p1[idx] - p2[idx] + p3[idx] ; // (i,j)位置，模板区域的灰度和

			wndMean2 =(double)t_sum*t_sum ; // 图像上，模板位置的灰度和的平方,wndMean2必须是long long型的或double型的，否则会溢出。
			wndMean2 /= templArea ;

			num -= t_sum*templMean ;// 参考式（7）中的第2项，此时的num为NCC的分子，即式（4）中的分子

			// 计算式（8）中的第1项
			wndSum2 = q0[idx2] - q1[idx2] - q2[idx2] + q3[idx2] ; // 图像上，模板位置的灰度平方和

			// 计算式（4）中的分母
			t = sqrt((double)(TM_MAX(wndSum2-wndMean2,0)))*templNorm; //templNorm是帜的刀?
			// 计算并保存NCC值
			if( abs(num) < t )
				confi = (double)num/t ;
			else if( abs(num) < t*1.125 )
				confi = num > 0 ? 1.0 : -1.0 ;
			else
				confi = 0.0 ;
			if( confi>gConfiLevel )
			{
				gConfiLevel   = confi ;
				gTemplCurRow  = i ;
				gTemplCurCol  = j ;
				if(iNum<30)
				{
					iResult[2*iNum]		= gTemplCurCol + gTemplWidth/2;
					iResult[2*iNum+1]	= gTemplCurRow + gTemplHeight/2;
					iNum++;
					iTotal++;
				}
				else
				{
					for(iNum=1; iNum<30; iNum++)
					{
						iResult[2*iNum-1]	= iResult[2*iNum+1];
						iResult[2*iNum-2]	= iResult[2*iNum];
					}
					iResult[58]	= gTemplCurCol + gTemplWidth/2;
					iResult[59] = gTemplCurRow + gTemplHeight/2;
					iTotal++;
				}
			}
		}
	}
	for(i=0; i<iNum-1; i++)
	{
		for(j=i+1; j<iNum; j++)
		{
			if((iResult[2*i]!=0) && (iResult[2*i+1]!=0) && (iResult[2*j]!=0) && (iResult[2*j+1]!=0))
			{
				if((abs(iResult[2*i] - iResult[2*j]) < 15) && (abs(iResult[2*i+1] - iResult[2*j+1]) < 15))
				{
					iResult[2*j]	= 0;
					iResult[2*j+1]	= 0;
				}
			}
		}
	}
	for(i=0,j=0; i<iNum; i++)
	{
		if(iResult[2*i]!=0)
		{
			iResult[2*j]	= iResult[2*i];
			iResult[2*j+1]	= iResult[2*i+1];
			j++ ;
		}
	}
	for(i=j; i<iNum; i++)
	{
		iResult[2*i]	= 0;
		iResult[2*i+1]	= 0;
	}
	iNum	= j;
	for(i=0; i<iNum/2; i++)
	{
		iResult[2*iNum]		= iResult[2*i];
		iResult[2*iNum+1]	= iResult[2*i+1];
		iResult[2*i] 		= iResult[2*(iNum-i)-2];
		iResult[2*i+1]		= iResult[2*(iNum-i)-1];
		iResult[2*(iNum-i)-2]	= iResult[2*iNum];
		iResult[2*(iNum-i)-1]	= iResult[2*iNum+1];
	}

	gTemplCurRow += top  ;//top表示匹配位置的上方位置
	gTemplCurCol += left ;//left表示匹配位置的左方位置
	*newRow = gTemplCurRow + gTemplHeight/2;//top表示匹配位置的上方位置
	*newCol = gTemplCurCol + gTemplWidth/2;
	return  gConfiLevel;
}

////////////////////////////////////////////////////////////////////////////////////////////
//图像和模板同时缩放【y抽样一半,x取平均值】,高度和宽度均缩小1/2
//Author:LongTao
////////////////////////////////////////////////////////////////////////////////////////////
void NCCScale(
	const unsigned char*  pSrc1,	//输入图像数据
	const unsigned char*  pSrc2,	//输入模板数据
	unsigned char *  pDst1,			//输出图像数据
	unsigned char *  pDst2,			//输出模板数据
	int srcWidth,							//图像宽度
	int srcHeight,							//图像高度
	int tmpWidth,							//模板宽度
	int tmpHeight)							//模板高度
{
	int i,j ;
	const unsigned char *pTmpSrc ;
	unsigned char *pTmpDst ;
	for(i=0; i<srcHeight/2; i++)
	{
		pTmpSrc = pSrc1 + i*srcWidth*2;
		pTmpDst	= pDst1 + i*srcWidth/2;
		for(j=0; j<srcWidth/2; j++)
		{
			pTmpDst[j] = (pTmpSrc[j<<1]+pTmpSrc[(j<<1)+1])>>1 ;
		}
	}
	for(i=0; i<tmpHeight/2; i++)
	{
		pTmpSrc = pSrc2 + i*tmpWidth*2;
		pTmpDst	= pDst2 + i*tmpWidth/2;
		for(j=0; j<tmpWidth/2; j++)
		{
			pTmpDst[j] = (pTmpSrc[j<<1]+pTmpSrc[(j<<1)+1])>>1 ;
		}
	}
}

//图像缩小，宽度缩小4倍，高度缩小2倍，y值抽样1/2，x值抽样1/4
void ImageScale(
	unsigned char  *pSrc,	//输入图像数据
	unsigned char  *pDst,	//输出图像数据
	int srcWidth,			//图像宽度
	int srcHeight)			//图像高度
{
	int i,j ;
	const unsigned char *pTempSrc=0 ;
	unsigned char *pTempDst=0 ;
	for(i=0; i<srcHeight/2; i++)
	{
		pTempSrc = pSrc + i*srcWidth*2;
		pTempDst = pDst + i*srcWidth/4;
		for(j=0; j<srcWidth/4; j++)
		{
			pTempDst[j] = pTempSrc[j*4];
		}
	}
}

float FullScreenNCC(
	int *newRow,
	int *newCol,
	int imgWidth,
	int imgHeight)
{
	float ratio;
	int search_x,search_y;
	int templ_w1,templ_h1;
	int templ_w2,templ_h2;
	static int image_w2,image_h2;
	unsigned char *pSrc1;
	unsigned char *pSrc2;
	unsigned char *pTmp1;
	unsigned char *pTmp2;

	pSrc1	= gImageData;
	pSrc2	= gImageBuf;
	pTmp1	= gTemplData;
	pTmp2	= gTemplBuf;
	templ_w1= gTemplWidth;
	templ_h1= gTemplHeight;
	templ_w2= gTemplWidth>>1;
	templ_h2= gTemplHeight>>1;
	image_w2= imgWidth>>1;
	image_h2= imgHeight>>1;

	NCCScale(pSrc1,pTmp1,pSrc2,pTmp2,imgWidth,imgHeight,gTemplWidth,gTemplHeight);
	gImageData	= pSrc2;
	gTemplData	= pTmp2;
	gTemplWidth = templ_w2;
	gTemplHeight= templ_h2;
	//updateNCCPos(image_h2,image_w2);
	ratio = MatchUsingNCC(&search_y,&search_x,image_w2,image_h2,image_w2,image_h2);
	gImageData	= pSrc1;
	gTemplData	= pTmp1;
	gTemplWidth = templ_w1;
	gTemplHeight= templ_h1;
	if(ratio > 0.7)
	{
		*newRow	= search_y<<1;
		*newCol	= search_x<<1;
		return ratio;
	}
	else
	{
		*newRow	= image_h2;
		*newCol	= image_w2;
		return -1;
	}
}


void updateNCCPos(int row,int col)
{
	gTemplPreRow = row - gTemplHeight/2;
	gTemplPreCol = col - gTemplWidth/2;
}

float getNewPos(int *row,int *col)
{
	*row = gTemplPreRow + gTemplHeight/2 ;
	*col = gTemplPreCol + gTemplWidth/2 ;
	return gConfiLevel ;
}

////////////////////////////////////////////////////////////////////////////////////////////
//求浮点型数据平均值
//Author:LongTao
////////////////////////////////////////////////////////////////////////////////////////////
float avr_constantf(
	float *array,			//矩阵起始地址
	int length)				//矩阵长度
{
	float sum,avr;
	int i;
	sum=0.f;
	avr=0.f;
	for(i=0; i<length; i++)
	{
		sum += array[i];
	}
	avr = sum/length;

	return avr;
}


////////////////////////////////////////////////////////////////////////////////////////////
//跟踪器
//Author:LongTao
////////////////////////////////////////////////////////////////////////////////////////////
float TrackTarget(
	unsigned char *pSrc,	//输入图像
	unsigned char *pTmp,	//输入模板
	unsigned char *pBuf,	//缓存模板
	int img_width,			//图像宽度
	int img_height,			//图像高度
	int tmp_width,			//模板宽度
	int tmp_height,			//模板高度
	unsigned short *x,
	unsigned short *y)
{

	cnt_track++;
	if(trackstate == 0)
	{
		
		setNCCTemplPtr(pTmp,tmp_width,tmp_height);

		setNCCImagePtr(pSrc);
		
		/*FILE *fccc1;  
		fccc1 = fopen("yuv.yuv","w"); 
		fwrite(pSrc,640*480, 1, fccc1);
		fclose(fccc1);
		
        FILE *fccc3;  
		fccc3 = fopen("yuv3.yuv","w"); 
		fwrite(gImageData,640*480, 1, fccc3);
		fclose(fccc3);
		
		FILE *fccc2; 	              	
		fccc2 = fopen("yuv2.yuv","w"); 		
		fwrite(pTmp,64*64, 1, fccc2);	
		fclose(fccc2);*/
	
		
		//track_y、track_x的赋值在TrackInit()中，坐标为128*128子图像坐标
		f32_nccRatio = matchTemplate(&track_y,&track_x,img_width,img_height,schWidth,schHeight);
		//返回图像区块的均值和方差*************************
		RectThreshold(pSrc,img_width,img_height,track_y-tmp_height/2,track_y+tmp_height/2,track_x-tmp_width/2,track_x+tmp_width/2,&f32_avrgray2,&f32_sigma2);
		f32_allRatio[cnt_track%20]	= f32_nccRatio;
		f32_avrRatio = 0.0001f;
		if(cnt_track >= 20)
		{
			f32_avrRatio = avr_constantf(f32_allRatio,20);
		}
		//*************************************************
		if(f32_nccRatio > f32_trackRatio)
		{
			updateNCCPos(track_y,track_x);
			cnt_trackon++;
			cnt_trackoff=0;
		}
		else
		{
			cnt_trackon=0;
			cnt_trackoff++;
		}
		if((f32_nccRatio > 0.5) && (f32_nccRatio < 0.85) && (f32_nccRatio/f32_avrRatio > 0.85) && (f32_nccRatio/f32_avrRatio < 1.15)
	//	&& (track_x < img_width*0.8) && (track_x > img_width*0.2)&&(track_y < img_height*0.8)&&(track_y > img_height*0.2)){
		)
		{
			achieveTemplate(pSrc,pBuf,img_width,img_height,tmp_width,tmp_height,track_x,track_y);
			//memcpy(pTmp,pBuf,tmp_width*tmp_height*sizeof(char));
		}

		if((f32_nccRatio > 0.55) && (f32_nccRatio < 0.7) && (f32_nccRatio/f32_avrRatio > 0.85) && (f32_nccRatio/f32_avrRatio < 1.15)
	//	&& (track_x < img_width*0.8) && (track_x > img_width*0.2)&&(track_y < img_height*0.8)&&(track_y > img_height*0.2)){
		)
		{
			if((cnt_trackon%5 == 0) && (abs(f32_sigma2-f32_sigma)<(f32_sigma*0.3)))
			{
				memcpy(pTmp,pBuf,tmp_width*tmp_height*sizeof(char));
				TempThreshold(pTmp,tmp_width,tmp_height,&f32_avrgray,&f32_sigma);
			}
		}
	}
	else
	{
		f32_nccRatio = FullScreenNCC(&track_y,&track_x,img_width,img_height);
		if(f32_nccRatio > 0.7)
		{
			//achieveTemplate(pSrc,pTmp,img_width,img_height,tmp_width,tmp_height,track_x,track_y);
			//TempThreshold(pTmp,tmp_width,tmp_height,&f32_avrgray,&f32_sigma);
			updateNCCPos(track_y,track_x);
			cnt_trackoff=0;
		}
		f32_avrRatio = 0.0001f;
	}

	if(cnt_trackoff>10)
	{
		track_mubiaodiushi=1;
		//schWidth	= widthL;	//168
		//schHeight	= heightL;	//128

		//schWidth	= 168;
		//schHeight	= 128;
		getNewPos(&track_y,&track_x);
		if((cnt_trackoff > 25) && (tracklevel == 1))
		{
			trackstate = 1;
		}
		*x	= track_x;
		*y	= track_y;
	//	return -1;
	}
	else
	{
		track_mubiaodiushi=0;
		//schWidth	= widthS-8;		//112
		//schHeight	= heightS-8;	//88
		schWidth	= 112;		//112
		schHeight	= 88;	//88
		trackstate	= 0;
		*x	= track_x;
		*y	= track_y;

		//return track_mubiaodiushi;//f32_nccRatio;
	}
	//printf("f32_nccRatio=%f\n",f32_nccRatio);
	return track_mubiaodiushi;
}

////////////////////////////////////////////////////////////////////////////////////////////
//判断跟踪器强度
//level=0	丢失目标局部重检
//level=1	丢失目标全屏重检
//level=2	更难丢失目标条件
//Author:LongTao
////////////////////////////////////////////////////////////////////////////////////////////
void TrackLevel(int level)
{
	tracklevel = level;
}

void GetSubImage(unsigned char *pSrc,	//输入图像数据
		unsigned char *pDst,	//输出子图像数据
		int imgWidth,			//图像宽度
		int imgHeight,			//图像高度
		unsigned short subimgWidth,      //子图像宽度
		unsigned short subimgHeight,     //子图像高度
		unsigned short col,				//取起始位置的列坐标
		unsigned short row)				//取起始位置的行坐标
{
	// 1，确定搜索位置并提取搜索
	int i=0,j=0;
	unsigned char *pSImage=0;
	unsigned char *pImage=0;
    //******************************************************
	if( (col-(subimgWidth>>1) ) <= 0 )
	{ // 假如默认搜索窗超出了图像左边界
		subimgleft  = 0 ;
		subimgright = subimgleft + subimgWidth ;
	}
	else if( (col+(subimgWidth>>1) ) >= imgWidth )
	{ // 假如默认搜索窗超出了图像右边界
		subimgright = imgWidth ;
		subimgleft  = imgWidth - subimgWidth ;
	}
	else
	{ // 假如默认搜索窗没有超出边界
		subimgleft  = col-(subimgWidth>>1);
		subimgright = subimgleft + subimgWidth ;
	}
	//******************************************************
	if( (row-(subimgHeight>>1) ) <= 0 )
	{ // 假如默认搜索窗超出了图像上边界
		subimgtop= 0 ;
		subimgbottom = subimgtop + subimgHeight ;
	}
	else if( (row+(subimgHeight>>1) ) >= imgHeight )
	{ // 假如默认搜索窗超出了图像下边界
		subimgbottom = imgHeight ;
		subimgtop  = imgHeight - subimgHeight ;
	}
	else
	{ // 假如默认搜索窗没有超出边界
		subimgtop  = row-(subimgHeight>>1);
		subimgbottom = subimgtop + subimgHeight ;
	}
	//******************************************************
	// 提取图像的子图像

 	for(i=subimgtop; i<subimgbottom; i++)
 	{
		pSImage = gSubImageData + (i-subimgtop)*subimgWidth;
		pImage  = (unsigned char *)pSrc + i*imgWidth;
		for(j=subimgleft; j<subimgright; j++)
		{
			pSImage[j-subimgleft] = pImage[j];
		}
	}
}
