/////////////////////////////////////////////////////////////////////////////////////////
//qsImgLib.h
//图像处理常用函数的定义头文件
//北京嵌视科技有限公司      http://shop111408299.taobao.com/
/////////////////////////////////////////////////////////////////////////////////////////
#ifndef		__QSIMGLIB_H__
#define		__QSIMGLIB_H__

#define 	QS_ZERO						0.0000001
#define 	QS_PI						3.1415926

#define		RETURN_SUCESS				1
#define		RETURN_FAILURE				0

#ifdef __cplusplus
extern "C" {
#endif



/////////////////////////////////////////////////////////////////////////////////////////
//JPEG解码函数
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsJpegDecode(
		unsigned char **yuyv, 				//输出的解码后的YUYV图像
		unsigned char *jpeg_buf, 			//输入的相机采集到的MJPEG数据流
		int *width, 						//解码得到的图像宽度
		int *height);						//解码得到的图像高度

/////////////////////////////////////////////////////////////////////////////////////////
//保存MJPEG码流得到JPEG图像
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsJpegSave(
		unsigned char *jpeg_buf,			//输入的相机采集到的MJPEG数据流
		int jpeg_size,						//MJPEG数据流的大小
		char file_name[1024]);				//文件名

		
		
		
/////////////////////////////////////////////////////////////////////////////////////////
//把内存数据写到指定的文件中
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsDataSave(
		const void *data_ptr,				//输入的待写入的数据地址
		const int len,						//数据长度
		const char filename[1024]);			//要写入的文件的文件名

/////////////////////////////////////////////////////////////////////////////////////////
//把指定的文件中的数据读取到内存中
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsDataLoad(
		void *data_ptr,						//读取的数据存放地址
		const int len,						//要读取的数据长度
		const char filename[1024]);			//要读取的文件的文件名
			  
/////////////////////////////////////////////////////////////////////////////////////////
//保存RGB数据为BMP图像
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsBmpRgbSave(
		const unsigned char* img_8u3_src,	//输入的RGB图像数据
		const int width,					//图像宽度
		const int height,					//图像高度
		const char filename[1024],			//文件名
		const int trans_flag);				//若输入图像为QT中的RGB888格式(原点左上角)，则需要进行格式转换，赋值1，否则赋值0
		
/////////////////////////////////////////////////////////////////////////////////////////
//载入RGB格式的BMP图像，如载入模板匹配的基准图等应用场合
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsBmpRgbLoad(
		unsigned char* img_8u3_load,		//载入的RGB图像数据
		int* width,							//读取的图像宽度
		int* height,						//读取的图像高度
		const char filename[1024],			//文件名
		const int trans_flag);				//若期望载入图像格式为QT中的RGB888格式(原点左上角)，则需要进行格式转换，赋值1，否则赋值0

/////////////////////////////////////////////////////////////////////////////////////////
//保存GRAY图像数据为BMP图像
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsBmpGraySave(
		const unsigned char* img_8u1_src,	//输入的GRAY图像数据
		const int width,					//图像宽度
		const int height,					//图像高度
		const char filename[1024],			//文件名
		const int trans_flag);				//若输入图像为QT中的图像格式(原点左上角)，则需要进行格式转换，赋值1，否则赋值0		

/////////////////////////////////////////////////////////////////////////////////////////
//载入灰度图像格式的BMP图像，如载入模板匹配的基准图等应用场合
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsBmpGrayLoad(
		unsigned char* img_8u1_load,		//载入的GRAY图像数据
		int* width,							//读取的图像宽度
		int* height,						//读取的图像高度
		const char filename[1024],			//文件名
		const int trans_flag);				//若期望载入图像格式为QT中的图像格式(原点左上角)，则需要进行格式转换，赋值1，否则赋值0
		
typedef struct
{
	int 	left;
	int 	right;
	int 	top;
	int 	bottom;
}QSRECT;

/////////////////////////////////////////////////////////////////////////////////////////
//求矩形框的宽、高、面积
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
int qsRectWidth(const QSRECT rc0);
int qsRectHeight(const QSRECT rc0);
int qsRectArea(const QSRECT rc0);

/////////////////////////////////////////////////////////////////////////////////////////
//将矩形框的各个元素限定在一定的取值范围内
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsRectSetRange(
		QSRECT *rc0, 						//初始化为输入矩形框，输出限制后的矩形框
		const int min_w, 					//限定矩形框(left,right)的最小值，通常设定为0
		const int min_h,					//限定矩形框(top,bottom)的最小值，通常设定为0
		const int max_w, 					//限定矩形框(left,right)的最大值，通常设定为IMAGE_WIDTH-1
		const int max_h); 					//限定矩形框(top,bottom)的最大值，通常设定为IMAGE_HEIGHT-1
		
/////////////////////////////////////////////////////////////////////////////////////////
//按比例缩放矩形框，返回值为缩放后的矩形框
//应用中得到缩放后的矩形框后需要利用qsRectSetRange对元素取值进行限制
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
QSRECT qsRectScale(
		const QSRECT rc0, 					//输入矩形框
		const float ratio);					//矩形框的缩放倍数

/////////////////////////////////////////////////////////////////////////////////////////
//求两个矩形框的交集的大小
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
int qsRectCrossArea(const QSRECT rc1, const QSRECT rc2);

/////////////////////////////////////////////////////////////////////////////////////////
//求两个矩形框的并集的大小
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
int qsRectUionArea(const QSRECT rc1, const QSRECT rc2);

/////////////////////////////////////////////////////////////////////////////////////////
//向图像中添加一个矩形框
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsAddRectToImg(
		unsigned char *img_8u3_src,			//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const QSRECT rc0,					//要添加的矩形框的位置
		const int line_thick,				//线条宽度，单位为像素
		const int r, 						//要添加的矩形框的颜色-red
		const int g, 						//要添加的矩形框的颜色-green
		const int b);						//要添加的矩形框的颜色-blue
		
/////////////////////////////////////////////////////////////////////////////////////////
//向图像中添加一个+
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsAddCrossToImg(
		unsigned char *img_8u3_src,			//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const int x0,	const int y0,		//要添加的+的位置
		const int half_cross_len,			//要添加的+的长度的一半，单位为像素
		const int half_cross_thick,			//要添加的+的宽度的一半，单位为像素
		const int r, 						//要添加的+的颜色-red
		const int g, 						//要添加的+的颜色-green
		const int b);						//要添加的+的颜色-blue
		
/////////////////////////////////////////////////////////////////////////////////////////
//把二值图像的前景部分（255）叠加显示到rgb图像中以供显示
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsBinaryOverlayImg(
		unsigned char *img_8u3_src,			//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const unsigned char *img_8u1_binary,//二值图像
		const int r, 						//叠加的颜色-red
		const int g, 						//叠加的颜色-green
		const int b,						//叠加的颜色-blue
		const float alfa);					//颜色混合因子或透明度, 0-1,1表示全覆盖

/////////////////////////////////////////////////////////////////////////////////////////
//只显示二值图像的前景部分（255）对应的场景图像
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsBinaryOcupyImg(
		unsigned char *img_8u3_src,			//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const unsigned char *img_8u1_binary);//二值图像		
		
/////////////////////////////////////////////////////////////////////////////////////////
//得到块图像数据,gray
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsBlockImageDataGray(
		const unsigned char *img_8u1_src,	//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const int left,						//块图像在整个图像中的起始位置——x方向
		const int top,						//块图像在整个图像中的起始位置——y方向
		const int block_width,				//要取得的块图像的宽度
		const int block_height,				//要取得的块图像的高度
		unsigned char *img_8u1_block);		//返回的块图像数据
		
/////////////////////////////////////////////////////////////////////////////////////////
//得到块图像数据,rgb
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsBlockImageDataRgb(
		const unsigned char *img_8u3_src,	//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const int left,						//块图像在整个图像中的起始位置——x方向
		const int top,						//块图像在整个图像中的起始位置——y方向
		const int block_width,				//要取得的块图像的宽度
		const int block_height,				//要取得的块图像的高度
		unsigned char *img_8u3_block);		//返回的块图像数据
		
/////////////////////////////////////////////////////////////////////////////////////////
//将4个灰度图像合并起来2×2显示，输出图像的大小与输入图像大小一致
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsMerge4Image(
		unsigned char *img_8u1_src1,		//输入灰度图像数据1
		unsigned char *img_8u1_src2,		//输入灰度图像数据2
		unsigned char *img_8u1_src3,		//输入灰度图像数据3
		unsigned char *img_8u1_src4,		//输入灰度图像数据4
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		unsigned char *img_8u1_dst);		//输出的合并后的灰度图像数据
		
					

		
		
	
//////////////////////////////////////////////////////////////////////////
//MFC中BGR888格式（原点左下角）与QT中的RGB888格式(原点左上角)之间的相互转换
//Author：EmbedVison
//////////////////////////////////////////////////////////////////////////
void qsRgbFormatWinQtTrans(
		const unsigned char *img_8u3_in,	//输入的win或qt图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		unsigned char *img_8u3_out);		//输出的qt或win的图像数据

//////////////////////////////////////////////////////////////////////////
//MFC中灰度图像格式（原点左下角）与QT中的灰度图像格式(原点左上角)之间的相互转换
//Author：EmbedVison
//////////////////////////////////////////////////////////////////////////
void qsGrayFormatWinQtTrans(
		const unsigned char *img_8u1_in,	//输入的win或qt图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		unsigned char *img_8u1_out);		//输出的qt或win的图像数据
		
/////////////////////////////////////////////////////////////////////////////////////////
//YUV转RGB函数
//Author：EmbedVison
//////////////////////////////////////////////////////////////////////////
void qsYuv2Rgb(
		const unsigned char *img_8u2_yuyv,	//输入图像YUYV
		unsigned char *img_8u3_rgb, 		//输出图像RGB
		const int img_width, 				//图像宽度
		const int img_height);				//图像高度
		
/////////////////////////////////////////////////////////////////////////////////////////
//YUV转Gray函数
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsYuv2Gray(
		const unsigned char *img_8u2_yuyv,	//输入图像YUYV
		unsigned char *img_8u1_gray,		//输出图像Gray
		const int img_width, 				//图像宽度
		const int img_height);				//图像高度
		
/////////////////////////////////////////////////////////////////////////////////////////
//把RGB图像转换为灰度图像的函数，在嵌入式应用中利用qsYuv2Gray获得灰度图像的效率更高
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsRgb2Gray(
		const unsigned char *img_8u3_src,	//输入源RGB图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		unsigned char *img_8u1_dst);		//输出的变换后的GRAY图像数据
		
/////////////////////////////////////////////////////////////////////////////////////////
//把灰度图像转换为RGB图像的函数，以供图像在MFC或者Qt的界面上显示及调试算法用
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsGray2Rgb(
		const unsigned char *img_8u1_src,	//输入源GRAY图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		unsigned char *img_8u3_dst);		//输出的变换后的RGB图像数据


	

	
	
/////////////////////////////////////////////////////////////////////////////////////////
//求取旋转、缩放后的图像的函数，只输出与输入图像同样大小区域的图像
//即：超出部分裁剪掉、不足部分以0补充，旋转中心设定为图像中心
//此函数可用于由基准样本产生不同尺度与角度下的训练样本
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsTransformImg(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		unsigned char *img_8u1_dst, 		//输出的变换后的图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const float scale, 					//需要缩放的因子
		const float sita);					//需要旋转的角度
		
/////////////////////////////////////////////////////////////////////////////////////////
//将图像宽度、高度归一化到标准尺寸
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsNormalizeImg(
		const unsigned char *img_8u1_src,	//输入源图像数据
		const int img_width_src,			//输入图像宽度
		const int img_height_src,			//输入图像高度
		const int img_width_dst,			//指定的输出图像宽度	
		const int img_height_dst,			//指定的输出图像高度
		unsigned char *img_8u1_dst);		//输出图像
		
/////////////////////////////////////////////////////////////////////////////////////////
//求图像的灰度直方图
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsHistogram(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		int *hist);							//输出的直方图
void qsHistogramRect(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const QSRECT rcROI,					//指定的统计区域
		int *hist);							//输出的直方图
		
/////////////////////////////////////////////////////////////////////////////////////////
//高斯平滑函数，模板为[1,2,1]*[1,2,1]'
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsGaussSmooth(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		unsigned char *img_8u1_dst, 		//输出的平滑后的图像
		unsigned char *img_8u1_temp);		//图像缓存
		
/////////////////////////////////////////////////////////////////////////////////////////
//均值平滑函数，平滑区域的大小为：w0*h0，窗口大小为3*3——400*400，且小于img_width,img_height
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsMeanSmooth(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int w0,						//平滑区域宽度
		const int h0,						//平滑区域高度
		unsigned char *img_8u1_dst,			//输出的平滑后的图像
		unsigned char *img_8u1_buffer);		//图像缓存
		
/////////////////////////////////////////////////////////////////////////////////////////
//求取sobel梯度幅度图像函数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsSobelMag(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		unsigned char *img_8u1_mag, 		//输出的梯度幅度图像
		unsigned char *table_sobel_sqrt);	//sobel开平方查找表缓存，大小至少为136900
		
/////////////////////////////////////////////////////////////////////////////////////////
//求取sobel梯度幅度、梯度方向图像函数，其中梯度方向像素值×2=角度值，即90代表180度，30代表60度。。。
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsSobelMagOri(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		unsigned char *img_8u1_mag, 		//输出的梯度幅度图像
		unsigned char *img_8u1_ori, 		//输出的梯度方向图像
		unsigned char *table_sobel_sqrt);	//sobel开平方查找表缓存，大小至少为136900
		
/////////////////////////////////////////////////////////////////////////////////////////
//求取输入图像的空间显著性图像，即每一个像素点相对于局部邻域的显著性
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsSpatialSaliency(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int scale_x,					//关注的尺度大小,x方向，代表关注的目标宽度为scale_x
		const int scale_y,					//关注的尺度大小,y方向，代表关注的目标高度为scale_y
		unsigned char *img_8u1_saliency, 	//输出的显著性图像
		unsigned char *img_8u1_temp1, 		//缓存图像
		unsigned char *img_8u1_temp2,		//缓存图像
		int *img_32u1_temp);				//存放显著性值的缓存图像，大小为img_width*img_height*sizeof(int)
		
/////////////////////////////////////////////////////////////////////////////////////////
//利用形态学高帽运算得到亮目标的残差图像，目标相对周围越亮残差值越大
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsTopHat(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int scale_x,					//关注的尺度大小,x方向，代表关注的目标宽度为scale_x，考虑到效率取值3-5即可
		const int scale_y,					//关注的尺度大小,y方向，代表关注的目标高度为scale_y，考虑到效率取值3-5即可
		unsigned char *img_8u1_residual); 	//输出的残差图像
		
/////////////////////////////////////////////////////////////////////////////////////////
//利用形态学低帽运算得到暗目标的残差图像，目标相对周围越暗残差值越大
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsBottomHat(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int scale_x,					//关注的尺度大小,x方向，代表关注的目标宽度为scale_x，考虑到效率取值3-5即可
		const int scale_y,					//关注的尺度大小,y方向，代表关注的目标高度为scale_y，考虑到效率取值3-5即可
		unsigned char *img_8u1_residual); 	//输出的残差图像
	
/////////////////////////////////////////////////////////////////////////////////////////
//阈值分割，二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsSegmentThd(
		const unsigned char *img_8u1_src, 	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int thd,						//分割阈值
		const int object_mode,				//关注亮目标时赋值1；关注暗目标时赋值0
		unsigned char *img_8u1_binary); 	//输出的二值图像
		
/////////////////////////////////////////////////////////////////////////////////////////
//利用弓弦法求取自适应阈值，通常用于残差图、显著性图等直方图具有单峰的图像的阈值分割
//参考文献：Unimodal thresholding, Pattern Recognition, 2001
//返回值：求取的分割阈值
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsTriangularThreshold(
		const int *hist, 					//图像的灰度直方图
		const float exp_ratio); 			//降低直方图对比度的幂指数
	
/////////////////////////////////////////////////////////////////////////////////////////
//利用基于过渡区域的ISODATA方法求取自适应阈值，通常用于直方图具有双峰或多峰的图像的阈值分割
//参考文献：Supervised grayscale thresholding based on transition regions, Image and Vision Computing, 2008
//返回值：求取的分割阈值
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsIsodataThreshold(
		const int *hist, 					//图像的灰度直方图
		const float min_ratio_low,			//限定属于低亮度区域的最小比例
		const float min_ratio_high); 		//限定属于高亮度区域的最小比例
		
/////////////////////////////////////////////////////////////////////////////////////////
//二值图像的形态学腐蚀操作，二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsBinaryErose(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		unsigned char *img_8u1_dst); 		//输出的二值图像

/////////////////////////////////////////////////////////////////////////////////////////
//二值图像的形态学膨胀操作，二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsBinaryDilate(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		unsigned char *img_8u1_dst); 		//输出的二值图像
		
/////////////////////////////////////////////////////////////////////////////////////////
//二值图像的形态学开运算操作，可用于去除噪声。二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsBinaryOpen(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int noise_level,				//噪声层次，表示一共进行了noise_level次腐蚀、noise_level次膨胀
		unsigned char *img_8u1_dst, 		//输出的二值图像
		unsigned char *img_8u1_buffer); 	//图像缓存

/////////////////////////////////////////////////////////////////////////////////////////
//二值图像的形态学闭运算操作，可用于填充小孔。二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsBinaryClose(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int hole_level,				//小孔层次，表示一共进行了hole_level次膨胀、hole_level次腐蚀
		unsigned char *img_8u1_dst, 		//输出的二值图像
		unsigned char *img_8u1_buffer); 	//图像缓存
		
/////////////////////////////////////////////////////////////////////////////////////////
//得到二值图像的外边缘，二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsBinaryExternalEdge(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		unsigned char *img_8u1_dst); 		//输出的边缘图像
		
/////////////////////////////////////////////////////////////////////////////////////////
//得到二值图像的内边缘，二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsBinaryInternalEdge(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		unsigned char *img_8u1_dst); 		//输出的边缘图像

/////////////////////////////////////////////////////////////////////////////////////////
//尺寸滤波器，去除二值图像中面积太大或面积太小的连通区域。二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
void qsSizeFilter(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int min_size,					//保留的连通区域的最小面积（像素数）
		const int max_size,					//保留的连通区域的最大面积（像素数）
		unsigned char *img_8u1_dst,			//输出的经过尺寸滤波后的二值图像
		unsigned char *img_8u1_buf,			//图像缓存
		int *pt_object,						//缓存，大小为img_width*img_height*sizeof(int)
		int *pt_stack);						//缓存，大小为img_width*img_height*sizeof(int)

/////////////////////////////////////////////////////////////////////////////////////////
//连通成分分析（Connected Component Analysis），得到二值图像中连通区域的外界矩形框及其面积
//返回值：满足要求的连通区域的个数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
int qsCCA(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int min_size,					//限定的连通区域的最小面积（像素数）
		const int max_size,					//限定的连通区域的最大面积（像素数）
		const int max_object_num,			//限定的连通区域的最大数量
		QSRECT	rc[],						//输出的连通区域的外接矩形框
		int size[],							//输出的连通区域的面积（像素数）
		unsigned char *img_8u1_buf,			//图像缓存
		int *pt_object,						//缓存，大小为img_width*img_height*sizeof(int)
		int *pt_stack);						//缓存，大小为img_width*img_height*sizeof(int)
		

//特征词结构体		
typedef struct
{
	//特征位置
	int						x, y;
	//HOG特征描述
	unsigned char			descriptor_HOG[45];
}QsFeatureCode;

//匹配对结构体
typedef struct
{
	//参考图像中特征点的位置
	int						ref_x, ref_y;
	//对应的当前图像中特征点的位置
	int						real_x, real_y;
	//匹配误差
	float error;
} QsMatchPair;

/////////////////////////////////////////////////////////////////////////////////////////
//FAST特征提取与描述，求取的特征较密集，适合于目标识别
//返回值：输出的特征数量
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsFeatureDetectFast(
		const unsigned char *img_8u1_src,	//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const int similar_thd,				//求取fast特征点时判定相对亮暗的阈值
		const int mx_feature_num,			//限定输出的特征点的最大数量，即最多输出角点响应值最大的mx_feature_num个角点
		const int descriptor_rotation_flag,	//特征描述子是否需要满足旋转不变性，赋值0表示不需要，其他值表明需要旋转不变性
		unsigned char *img_8u1_responce,	//图像缓存，用于存储FAST响应值
		QsFeatureCode features[]);			//输出的特征位置、及其HOG描述向量	
		
/////////////////////////////////////////////////////////////////////////////////////////
//FAST特征提取与描述，求取的特征较稀疏、且空间分布较均匀，适合于图像匹配
//返回值：输出的特征数量
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsFeatureDetectFastSparse(
		const unsigned char *img_8u1_src,	//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const int similar_thd,				//求取fast特征点时判定相对亮暗的阈值
		const int mx_feature_num,			//限定输出的特征点的最大数量，即最多输出角点响应值最大的mx_feature_num个角点
		const int descriptor_rotation_flag,	//特征描述子是否需要满足旋转不变性，赋值0表示不需要，其他值表明需要旋转不变性
		unsigned char *img_8u1_responce,	//图像缓存，用于存储FAST响应值
		QsFeatureCode features[]);			//输出的特征位置、及其HOG描述向量

/////////////////////////////////////////////////////////////////////////////////////////
//基于KL距离的HOG特征描述子匹配
//返回值：输出匹配点对的数量
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsFeatureMatchHog(
		const QsFeatureCode feature_ref[], 	//基准图像中的特征点集
		const int num_ref, 					//基准图像中的特征点数量
		const QsFeatureCode feature_real[], //当前图像中的特征点集
		const int num_real, 				//当前图像中的特征点数量
		const int max_displacement,			//容许的最大特征对间的距离
		int *dis_matrix,					//距离矩阵缓存，大小不小于num_basic*num_real
		QsMatchPair match_pairs[]);			//输出的匹配特征点对
	
/////////////////////////////////////////////////////////////////////////////////////////
//利用最小二乘法(LSE)求匹配对的仿射变换参数
//      x_ref = p0 * x_real + p1 * y_real + p2
//      y_ref = p3 * x_real + p4 * y_real + p5
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsFeatureTransformParaLse(
		const QsMatchPair match_points[],	//匹配特征点对
		const int match_num,				//匹配特征点对的数量
		float affine_para[]);				//输出的仿射变换参数
	
/////////////////////////////////////////////////////////////////////////////////////////
//利用RANSAC求匹配对的仿射变换参数
//      x_ref = p0 * x_real + p1 * y_real + p2
//      y_ref = p3 * x_real + p4 * y_real + p5
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsFeatureTransformParaRansac(
		const QsMatchPair match_points[],	//匹配特征点对
		const int match_num,				//匹配特征点对的数量
		const int ransac_num,				//进行ransac的次数
		float affine_para[]);				//输出的仿射变换参数

/////////////////////////////////////////////////////////////////////////////////////////
//利用RANSAC求匹配对的仿射变换参数
//		affine_para_src = [p0, p1, p2, p3, p4, p5],  affine_para_inverse = [q0, q1, q2, q3, q4, q5]
//      x_ref = p0 * x_real + p1 * y_real + p2,		y_ref = p3 * x_real + p4 * y_real + p5
//      x_real = q0 * x_ref + q1 * y_ref + q2,      y_real = q3 * x_ref + q4 * y_ref + q5
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
void qsFeatureTransformParaInverse(
		const float affine_para_src[],		//输入的仿射变换参数
		float affine_para_inverse[]);		//输出的仿射变换参数

/////////////////////////////////////////////////////////////////////////////////////////
//基于仿射变换参数求两幅图像的残差图像
//      x_ref = p0 * x_real + p1 * y_real + p2
//      y_ref = p3 * x_real + p4 * y_real + p5
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////			
void qsFrameDifferenceAffine(
		const unsigned char *img_8u1__real, //当前时刻图像数据
		const unsigned char *img_8u1_ref, 	//基准图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const float *affine_para, 			//仿射变换参数
		unsigned char *img_8u1_diff);		//输出的残差图像数据
/////////////////////////////////////////////////////////////////////////////////////////
//启动跟踪器
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsObjectTrackStart(
		const unsigned char *img_8u3_src,	//初始帧图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const QSRECT rc_init);				//初始化目标所在的范围

/////////////////////////////////////////////////////////////////////////////////////////
//在当前帧图像中执行跟踪器
//返回值：当前图像中目标的外接矩形框位置
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
QSRECT qsObjectTrackExcute(
		unsigned char *img_8u3_src,			//当前帧图像数据
		const int img_width,				//图像宽度
		const int img_height);				//图像高度

/////////////////////////////////////////////////////////////////////////////////////////
//停止跟踪器
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsObjectTrackStop();
//圆对象, 方程为(x-x0)^2 +  (y-y0)^2 = r^2
typedef struct
{
	float 	x, y;							//中心点
	float 	r;								//半径
}QSCIRCLE;

/////////////////////////////////////////////////////////////////////////////////////////
//创建一个圆对象
//返回值：创建的圆对象
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
QSCIRCLE qsCircleCreate(
		const float x,						//圆中心点的x坐标
		const float y,						//圆中心点的y坐标
		const float r);						//圆半径

/////////////////////////////////////////////////////////////////////////////////////////
//向图像中添加一个圆
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsAddCircleToImg(
		unsigned char *img_8u3_src,			//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const QSCIRCLE circle,				//圆对象
		const int line_thick,				//圆线条的宽度
		const int r, 						//要添加的圆线条的颜色-red
		const int g, 						//要添加的圆线条的颜色-green
		const int b);						//要添加的圆线条的颜色-blue

/////////////////////////////////////////////////////////////////////////////////////////
//向图像中添加一个实心圆
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsAddHardCircleToImg(
		unsigned char *img_8u3_src,			//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const QSCIRCLE circle,				//圆对象
		const int r, 						//要添加的实心圆的颜色-red
		const int g, 						//要添加的实心圆的颜色-green
		const int b,						//要添加的实心圆的颜色-blue
		const float alfa);					//颜色混合因子或透明度, 0-1,1表示全覆盖	

/////////////////////////////////////////////////////////////////////////////////////////
//得到圆的参数限定的二值图像区域，二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsCircleArea(
		unsigned char *img_8u1_binary,		//输出的二值图像
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const QSCIRCLE circle);				//圆对象

/////////////////////////////////////////////////////////////////////////////////////////
//得到两个圆的参数限定的环形的二值图像区域，二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsCircleRaceArea(
		unsigned char *img_8u1_binary,		//输出的二值图像
		unsigned char *img_8u1_buf,			//图像缓存
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const QSCIRCLE circle1,				//圆1对象
		const QSCIRCLE circle2);			//圆2对象

/////////////////////////////////////////////////////////////////////////////////////////
//利用最小二乘法(LSE)求圆的参数
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsCircleParaLse(
		const int x_point[],				//圆上像素点的x坐标
		const int y_point[],				//圆上像素点的y坐标
		const int num_point,				//圆上像素点的数量
		QSCIRCLE *circle);					//输出的圆的参数

/////////////////////////////////////////////////////////////////////////////////////////
//利用去1法求圆的参数
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsCircleParaRobust(
		const int x_point[],				//圆上像素点的x坐标
		const int y_point[],				//圆上像素点的y坐标
		const int num_point,				//圆上像素点的数量
		const float outlier_ratio,			//需要去除的坏点的比例[0, 0.5]
		QSCIRCLE *circle);					//输出的圆的参数

/////////////////////////////////////////////////////////////////////////////////////////
//从二值图像（阈值分割图像）中找圆
//返回值：圆的个数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
int qsCircleFromBinaryImg(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int min_radius,				//限定的圆的最小半径（像素数）
		const int max_radius,				//限定的圆的最大半径（像素数）
		const int max_circle_num,			//限定的圆的最大数量
		const float max_bias_ratio,			//限定圆周上的点的相对偏离误差中值的最大值（相对偏离误差/半径，0.01-0.2）
		QSCIRCLE circles[],					//输出的圆的参数
		unsigned char *img_8u1_buf,			//图像缓存
		int *pt_object,						//缓存，大小为img_width*img_height*sizeof(int)
		int *pt_stack);						//缓存，大小为img_width*img_height*sizeof(int)

/////////////////////////////////////////////////////////////////////////////////////////
//从灰度图像中直接提取圆，适用于目标图像不太容易进行二值分割的场合
//直接从灰度图中找效率较低，且效果也不一定好，建议能进行二值分割的尽量想办法分割
//返回值：圆的个数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
int qsCircleFromGrayImg(
		const unsigned char *img_8u1_gray,	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int min_radius,				//限定的圆的最小半径（像素数）
		const int max_radius,				//限定的圆的最大半径（像素数）
		const int max_circle_num,			//限定的圆的最大数量
		const float max_bias_ratio,			//限定圆周上的点的相对偏离误差中值的最大值（相对偏离误差/半径，0.01-0.2）
		QSCIRCLE circles[],					//输出的圆的参数
		unsigned char *img_8u1_edge,		//输出的边缘图像
		unsigned char *img_8u1_buf,			//图像缓存，大小为136900与img_width*img_height的最大值
		int *pt_object,						//缓存，大小为img_width*img_height*sizeof(int)
		int *pt_stack);						//缓存，大小为img_width*img_height*sizeof(int)

/////////////////////////////////////////////////////////////////////////////////////////
//将两个圆参数限定的环形图像区域拉伸，得到矩形的长条区域。
//由于工件上的字符是按照顺时针排列的，因此拉伸方向为顺时针
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
int qsCircleRaceStretchImg(
		const unsigned char *img_8u1_gray,	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const QSCIRCLE circle_out,			//外圆参数
		const QSCIRCLE circle_in,			//内圆参数
		int *stretched_img_width,			//输出的拉伸后的图像的宽度（圆周方向）： (circle_out.r * 2 * QS_PI)
		int *stretched_img_height,			//输出的拉伸后的图像的高度（半径方向）：(circle_out.r - circle_in.r)
		unsigned char *img_8u1_stretched);	//输出的大小为stretched_img_width*stretched_img_height的拉伸后的图像


//椭圆对象，椭圆零件、或圆形零件在运动中/非正视成像时表现为椭圆
//方程为[(x-x0)cos(sita)+(y-y0)sin(sita)]^2        [-(x-x0)sin(sita)+(y-y0)cos(sita)]^2
//      -----------------------------------    +   ------------------------------------   =  1
//                     rx^2                                          ry^2
typedef struct
{
	float 	x, y;							//中心点
	float 	rx;								//长轴
	float	ry;								//短轴
	float	sita;							//rx和x轴线的夹角,单位为弧度[0, 2*QS_PI)
}QSELLIPSE;

/////////////////////////////////////////////////////////////////////////////////////////
//创建一个椭圆对象
//返回值：创建的椭圆对象
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
QSELLIPSE qsEllipseCreate(
		const float x,						//椭圆中心点的x坐标
		const float y,						//椭圆中心点的y坐标
		const float rx,						//椭圆x方向的轴的长度的一半
		const float ry,						//椭圆y方向的轴的长度的一半
		const float sita);					//rx和x轴线的夹角,单位为角度

/////////////////////////////////////////////////////////////////////////////////////////
//向图像中添加一个椭圆
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsAddEllipseToImg(
		unsigned char *img_8u3_src,			//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		QSELLIPSE ellipse,					//椭圆参数
		const int line_thick,				//线条的宽度
		const int r, 						//要添加的圆线条的颜色-red
		const int g, 						//要添加的圆线条的颜色-green
		const int b);						//要添加的圆线条的颜色-blue

/////////////////////////////////////////////////////////////////////////////////////////
//得到椭圆的参数限定的二值图像区域，二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsEllipseArea(
		unsigned char *img_8u1_binary,		//输出的二值图像
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		QSELLIPSE ellipse);					//椭圆的参数

/////////////////////////////////////////////////////////////////////////////////////////
//得到两个椭圆的参数限定的椭环形的二值图像区域，二值图像中前景为255，背景为0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void qsEllipseRaceArea(
		unsigned char *img_8u1_binary,		//输出的二值图像
		unsigned char *img_8u1_buf,			//图像缓存
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		QSELLIPSE ellipse1,					//椭圆1的参数
		QSELLIPSE ellipse2);				//椭圆2的参数

/////////////////////////////////////////////////////////////////////////////////////////
//利用最小二乘法(LSE)求椭圆的参数
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsEllipseParaLse(
		const int x_point[],				//椭圆上像素点的x坐标
		const int y_point[],				//椭圆上像素点的y坐标
		const int num_point,				//椭圆上像素点的数量
		QSELLIPSE *ellipse);				//输出的椭圆的参数

/////////////////////////////////////////////////////////////////////////////////////////
//利用去1法求椭圆的参数
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int qsEllipseParaRobust(
		const int x_point[],				//椭圆上像素点的x坐标
		const int y_point[],				//椭圆上像素点的y坐标
		const int num_point,				//椭圆上像素点的数量
		const float outlier_ratio,			//需要去除的坏点的比例[0, 0.5]
		QSELLIPSE *ellipse);				//输出的椭圆的参数

/////////////////////////////////////////////////////////////////////////////////////////
//从二值图像（阈值分割图像）中找椭圆
//返回值：椭圆的个数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
int qsEllipseFromBinaryImg(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int min_radius,				//限定的椭圆的最小半径(rx+ry/2)（像素数）
		const int max_radius,				//限定的椭圆的最大半径(rx+ry/2)（像素数）
		const int max_ellipse_num,			//限定的椭圆的最大数量
		const float max_bias_ratio,			//限定椭圆周上的点的相对偏离误差中值的最大值（相对偏离误差/长短轴，0.01-0.2）
		QSELLIPSE ellipses[],				//输出的椭圆的参数
		unsigned char *img_8u1_buf,			//图像缓存
		int *pt_object,						//缓存，大小为img_width*img_height*sizeof(int)
		int *pt_stack);						//缓存，大小为img_width*img_height*sizeof(int)

/////////////////////////////////////////////////////////////////////////////////////////
//从灰度图像中直接提取椭圆，适用于目标图像不太容易进行二值分割的场合
//直接从灰度图中找效率较低，且效果也不一定好，建议能进行二值分割的尽量想办法分割
//返回值：圆的个数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
int qsEllipseFromGrayImg(
		const unsigned char *img_8u1_gray,	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int min_radius,				//限定的椭圆的最小半径(rx+ry/2)（像素数）
		const int max_radius,				//限定的椭圆的最大半径(rx+ry/2)（像素数）
		const int max_ellipse_num,			//限定的椭圆的最大数量
		const float max_bias_ratio,			//限定椭圆周上的点的相对偏离误差中值的最大值（相对偏离误差/长短轴，0.01-0.2）
		QSELLIPSE ellipses[],				//输出的椭圆的参数
		unsigned char *img_8u1_edge,		//输出的边缘图像
		unsigned char *img_8u1_buf,			//图像缓存，大小为136900与img_width*img_height的最大值
		int *pt_object,						//缓存，大小为img_width*img_height*sizeof(int)
		int *pt_stack);						//缓存，大小为img_width*img_height*sizeof(int)

/////////////////////////////////////////////////////////////////////////////////////////
//将两个椭圆参数限定的环形图像区域拉伸，得到矩形的长条区域。
//由于工件上的字符是按照顺时针排列的，因此拉伸方向为顺时针
//返回值：成功返回RETURN_SUCESS，失败返回RETURN_FAILURE
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
int qsEllipseRaceStretchImg(
		const unsigned char *img_8u1_gray,	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const QSELLIPSE ellipse_out,		//外椭圆参数
		const QSELLIPSE ellipse_in,			//内椭圆参数
		int *stretched_img_width,			//输出的拉伸后的图像的宽度（圆周方向）： (ellipse_out.rx+ ellipse_out.ry) * QS_PI
		int *stretched_img_height,			//输出的拉伸后的图像的高度（半径方向）：(ellipse_out.rx+ellipse_out.ry - ellipse_in.rx-ellipse_in.ry)/2
		unsigned char *img_8u1_stretched);	//输出的大小为stretched_img_width*stretched_img_height的拉伸后的图像







											
//线段对象
typedef struct
{
	float 	x1;								//控制点1的x坐标
	float 	y1;								//控制点1的y坐标
	float 	x2;								//控制点2的x坐标
	float 	y2;								//控制点2的y坐标
}QSLINE;											

/////////////////////////////////////////////////////////////////////////////////////////
//创建一个线段对象
//返回值：创建的线段对象
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
QSLINE qsLineCreate(
		float 	x1,							//控制点1的x坐标
		float 	y1,							//控制点1的y坐标
		float 	x2,							//控制点2的x坐标
		float 	y2);						//控制点2的y坐标

/////////////////////////////////////////////////////////////////////////////////////////
//获得线段对象的长度（测量）
//返回值：线段的长度
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
float qsLineLength(QSLINE line);

/////////////////////////////////////////////////////////////////////////////////////////
//向图像中添加一个线段
//Author：EmbedVison
/////////////////////////////////////////////////////////////////////////////////////////
void qsAddLineToImg(
		unsigned char *img_8u3_src,			//输入源图像数据
		const int img_width,				//图像宽度
		const int img_height,				//图像高度
		const QSLINE line,					//线段对象
		const int line_thick,				//线条宽度
		const int r, 						//要添加的线段的颜色-red
		const int g, 						//要添加的线段的颜色-green
		const int b);						//要添加的线段的颜色-blue

/////////////////////////////////////////////////////////////////////////////////////////
//从二值图像（阈值分割图像）中找线段
//返回值：线段的个数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
int qsLineFromBinaryImg(
		const unsigned char *img_8u1_binary, //输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int min_length,				//限定的线段的最小长度（像素数）
		const float error_bound,			//线段上的点偏离直线的误差上界，单位为像素
		const int max_line_num,				//限定的线段的最大数量
		QSLINE lines[],						//输出的线段的参数
		unsigned char *img_8u1_buf,			//图像缓存
		int *pt_object,						//缓存，大小为img_width*img_height*sizeof(int)
		int *pt_stack);						//缓存，大小为img_width*img_height*sizeof(int)

/////////////////////////////////////////////////////////////////////////////////////////
//从灰度图像中直接提取线段，适用于目标图像不太容易进行二值分割的场合
//直接从灰度图中找效率较低，且效果也不一定好，建议能进行二值分割的尽量想办法分割
//返回值：线段的个数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////		
int qsLineFromGrayImg(
		const unsigned char *img_8u1_gray,	//输入源图像数据
		const int img_width, 				//图像宽度
		const int img_height, 				//图像高度
		const int min_length,				//限定的线段的最小长度（像素数）
		const float error_bound,			//线段上的点偏离直线的误差上界，单位为像素
		const int max_line_num,				//限定的线段的最大数量
		QSLINE lines[],						//输出的线段的参数
		unsigned char *img_8u1_edge,		//输出的边缘图像
		unsigned char *img_8u1_buf,			//图像缓存，大小为136900与img_width*img_height的最大值
		int *pt_object,						//缓存，大小为img_width*img_height*sizeof(int)
		int *pt_stack);						//缓存，大小为img_width*img_height*sizeof(int)



#ifdef __cplusplus
}
#endif


#endif
