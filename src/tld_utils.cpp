#include <tld_utils.h>


void drawBox(Mat& image, CvRect box, Scalar color, int thick)
{
	rectangle( image, cvPoint(box.x, box.y), cvPoint(box.x+box.width,box.y+box.height),color, thick);
} 

void drawPoints(Mat& image, vector<Point2f> points,Scalar color)
{
	for(vector<Point2f>::const_iterator i = points.begin(), ie = points.end(); i != ie; ++i)
	{
		Point center( cvRound(i->x ), cvRound(i->y));
		circle(image,*i,2,color,1);
	}
}

Mat createMask(const Mat& image, CvRect box)
{
	Mat mask = Mat::zeros(image.rows,image.cols,CV_8U);
	drawBox(mask,box,Scalar::all(255),CV_FILLED);
	return mask;
}

float median(vector<float> v)
{
    int n = floor(v.size() / 2);
    nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

vector<int> index_shuffle(int begin,int end)
{
	vector<int> indexes(end-begin);
	for (int i=begin;i<end;i++)
		indexes[i]=i;

	random_shuffle(indexes.begin(),indexes.end());
	return indexes;
}

double myTemplateMatch(const MyMat * pTemplate, const MyMat * src, int w, int h)
{
	int i, j, m, n;
	double dSumT; //Ä£°åÔªËØµÄÆœ·œºÍ
	double dSumS; //ÍŒÏñ×ÓÇøÓòÔªËØµÄÆœ·œºÍ
	double dSumST; //ÍŒÏñ×ÓÇøÓòºÍÄ£°åµÄµã»ý	
	double R;
	//ŒÇÂŒµ±Ç°µÄ×îŽóÏìÓŠ
	double MaxR;
	//×îŽóÏìÓŠ³öÏÖÎ»ÖÃ
	int nMaxX;
	int nMaxY;
    int nHeight = /*src->rows*/ h;
    int nWidth = /*src->cols*/ w;
	//Ä£°åµÄžß¡¢¿í
    int nTplHeight = /*pTemplate->rows*/ h;
    int nTplWidth = /*pTemplate->cols*/ w;

	//ŒÆËã dSumT
	dSumT = 0;
	for (m = 0; m < nTplHeight; m++)
	{
		for (n = 0; n < nTplWidth; n++)
		{
			// Ä£°åÍŒÏñµÚmÐÐ£¬µÚnžöÏóËØµÄ»Ò¶ÈÖµ
            int nGray =/**pTemplate->ptr(m, n)*/ *(pTemplate->data + m * nTplWidth + n);
			dSumT += (double)nGray*nGray;
		}
	}

	//ÕÒµœÍŒÏñÖÐ×îŽóÏìÓŠµÄ³öÏÖÎ»ÖÃ
	MaxR = 0;
	for (i = 0; i < nHeight - nTplHeight + 1; i++)
	{
		for (j = 0; j < nWidth - nTplWidth + 1; j++)
		{
			dSumST = 0;
			dSumS = 0;
			for (m = 0; m < nTplHeight; m++)
			{
				for (n = 0; n < nTplWidth; n++)
				{

					// Ô­ÍŒÏñµÚi+mÐÐ£¬µÚj+nÁÐÏóËØµÄ»Ò¶ÈÖµ
                    int nGraySrc = /**src->ptr(i + m, j + n)*/ *(src->data + (i + m) * nTplWidth + j + n);

					// Ä£°åÍŒÏñµÚmÐÐ£¬µÚnžöÏóËØµÄ»Ò¶ÈÖµ
                    int nGrayTpl = /**pTemplate->ptr(m, n)*/ *(pTemplate->data + m * nTplWidth + n);
					dSumS += (double)nGraySrc*nGraySrc;
					dSumST += (double)nGraySrc*nGrayTpl;
				}
			}
			R = dSumST / (sqrt(dSumS)*sqrt(dSumT));//ŒÆËãÏà¹ØÏìÓŠ

			if (R > MaxR)
			{
				MaxR = R;
				nMaxX = j;
				nMaxY = i;
			}
		}
	}
return MaxR;

}


//均值
int meanDev(unsigned char* src, int w, int h)
{
    int sum = 0;
    int num = w * h;
    int mean = 0;
    int i, j;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            sum += *(src + i * w + j);
        }
    }

    mean = sum / num;
    return mean;
}

//标准差
double StDev(unsigned char* src, int w, int h, int mean)
{
    int sum = 0;
    int num = w * h;
    int i, j;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            int temp = (*(src + i * w + j) - mean);
            sum += temp * temp;
        }
    }
    double st = sqrt((double)sum / num);
    return st;
}


int MyIntegral(const unsigned char * src, int width, int height, int * dest, int * sqdest)
{
    int destW = width + 1;
    int destH = height + 1;
    memset(dest, 0, destW * destH * sizeof(char));
    memset(sqdest, 0, destW * destH * sizeof(char));

    int dx = 0;
    int dy = 0;

    for (int i = 0; i < height; i++)
    {
        dy = i + 1;
        for (int j = 0; j < width; j++)
        {
            dx = j + 1;
            int x11 = (dy - 1) * destW + (dx -1);
            int x12 = (dy - 1) * destW + dx;
            int x21 = dy * destW + dx - 1;

            int sum1 = dest[x11];
            int sum2 = dest[x12];
            int sum3 = dest[x21];

            int sum11 = sqdest[x11];
            int sum22 = sqdest[x12];
            int sum33 = sqdest[x21];

            int sum4 = src[i * width + j];

            int sum = sum4 + sum3 + sum2 - sum1;
            int sqsum = sum4 * sum4 + sum33 + sum22 - sum11;

            dest[dy * destW + dx] = sum;
            sqdest[dy * destW + dx] = sqsum;
        }
    }


    return 0;
}

void my_resize(const unsigned char *dataSrc, unsigned char *dataDst,int src_width, int src_height, int width, int height)
{
	double xRatio = (double)src_width / width;
	double yRatio = (double)src_height / height;

	for (int i = 0; i < height; i++)
	{
        double srcY = (i + 0.5) * yRatio - 0.5;//源图像“虚”坐标的y值
		int IntY = (int)srcY;//向下取整
		double v = srcY - IntY;//获取小数部分
		double v1 = 1.0 - v;
		for (int j = 0; j < width; j++)
		{
            double srcX = (j + 0.5) * xRatio - 0.5;//源图像“虚”坐标的x值
		    int IntX = (int)srcX;//向下取整
		    double u = srcX - IntX;//获取小数部分
		    double u1 = 1.0 - u;

		    int Index00 = IntY * src_width + IntX;//得到原图左上角的像素位置
		    int Index10;                            //原图左下角的像素位置
		    if (IntY < src_height - 1)
		    {
		        Index10 = Index00 + src_width;
		    }
		    else
		    {
		        Index10 = Index00;
		    }
		    int Index01;                            //原图右上角的像素位置
		    int Index11;                            //原图右下角的像素位置
		    if (IntX < src_width - 1)
		    {
		        Index01 = Index00 + 1;
		        Index11 = Index10 + 1;
		    }
		    else
		    {
		        Index01 = Index00;
		        Index11 = Index10;
		    }
		    int temp0 = (int)(v1 * (u * dataSrc[Index01] + u1 * dataSrc[Index00]) +
		                    v * (u * dataSrc[Index11] + u1 * dataSrc[Index10]));
			*(dataDst + i*width + j) = temp0;
		}
	}
}

void imgRoi(const unsigned char *src, ScaleBox srcbox, unsigned char *dst, RectBox dstbox)
{
    int i, j;
    for(i = 0; i < dstbox.height; i++)
    {
        for(j = 0; j < dstbox.width; j ++)
        {
            *(dst + i * dstbox.width + j) = *(src + (i + dstbox.y) * srcbox.width + j + dstbox.x);
        }
    }
}

/************************************************
函  数  名: getGaussianArray
功       能 : 获取高斯分布数组 
输入参数 :
arr_size   :矩阵大小
sigma      :
输出参数 :
array      :二维数组
/**********************************************/
double **getGaussianArray(int arr_size, double sigma)
{
    int i, j;
    // [1] 初始化权值数组
    double **array = new double*[arr_size];
    for (i = 0; i < arr_size; i++) {
        array[i] = new double[arr_size];
    }
    // [2] 高斯分布计算
    int center_i, center_j;
    center_i = center_j = arr_size / 2;
    double pi = 3.141592653589793;
    double sum = 0.0f;
    // [2-1] 高斯函数
    for (i = 0; i < arr_size; i++ ) {
        for (j = 0; j < arr_size; j++) {
            array[i][j] = 
                //后面进行归一化，这部分可以不用
                //0.5f *pi*(sigma*sigma) * 
                exp( -(1.0f)* ( ((i-center_i)*(i-center_i)+(j-center_j)*(j-center_j)) /
                                                             (2.0f*sigma*sigma) ));
            sum += array[i][j];
        }
    }
    // [2-2] 归一化求权值
    for (i = 0; i < arr_size; i++) {
        for (j = 0; j < arr_size; j++) {
            array[i][j] /= sum;
            //printf(" [%.15f] ", array[i][j]);
        }
        //printf("\n");
    }
    return array;
}


/************************************************
函  数  名: extendEdge
功       能 : 图像边缘扩充(对称-映射)
输入参数 :
_src         :输入图像
w            :输入图像宽
h             :输入图像高
_size        :要扩充边缘大小(两边个 _size / 2)
输出参数 :
_dst :输出图像
/**********************************************/
void extendEdge(const unsigned char *_src, unsigned char *_dst, int w, int h, int _size)
{
    int center = _size / 2;
    int size = _size  - 1;  
    int ww = w + size;
    int hh = h + size;
    //unsigned char src[ww * hh];
    int i, j;

    //center
    for ( i = 0; i < h; i++)
    {
        for ( j = 0; j < w; j++)
        {
            /* code */
            *(_dst + (i + center) * ww + (j + center)) = *(_src + i * w + j);
        }
        /* code */
    }

    //left
    //printf("i : %d i:%d\n", centor, hh - centor);
    for (i = center; i < hh - center; i++)
    {
        int size = _size - 1;
        for ( j = 0; j < center; j++)
        {
            int len = i * ww + j;
            int lenn = len + size;
            //printf("w : %d ww:%d size:%d\n", len, lenn, size);
            *(_dst + len) = *(_dst + lenn);
            size = size - 2;
           
        }
       
    }

    //right
    for (i = center; i < hh - center; i++)
    {
        int size = 2;
        for ( j = ww - center; j < ww; j++)
        {
            *(_dst + i * ww + j) = *(_dst + i * ww + j - size);
            size = size + 2;   
        }
    }

    //up
    size = _size - 1;
    for (i = 0; i < center; i++)
    {
        for ( j = 0; j < ww; j++)
        {
            *(_dst + i * ww + j) = *(_dst + (i + size) * ww + j);   
        }  
        size = size - 2;       
    }

    //down
    size = 2;
    for (i = hh - center; i < hh; i++)
    {
        for ( j = 0; j < ww; j++)
        {
            *(_dst + i * ww + j) = *(_dst + (i - size) * ww + j);
        }      
        size = size + 2; 
    }
    
}


/************************************************
函  数  名: gaussian
功       能 : 获取高斯分布数组 
输入参数 :
_src         :输入图像
w            :输入图像宽
h             :输入图像高
_size        :高斯核大小
sigma      :
输出参数 :
_dst         :输出图像
/**********************************************/
void myGaussian(const unsigned char *_src, unsigned char *_dst, int w, int h, int _size, double sigma)
{
    //获得高斯核
    double **_array;
    _array = getGaussianArray(_size, sigma);

    int ww = w + _size - 1;
    int hh = h + _size - 1;

    //扩充图像边缘,方便计算边缘
    unsigned char dst[ww * hh];
    extendEdge(_src, dst, w, h, _size);

    // [1] 扫描
    int center = _size / 2;
    for (int i = 0; i < hh; i++) 
    {
        for (int j = 0; j < ww; j++) 
        {
            // [2] 忽略边缘
                        if (i > (_size / 2) - 1 && j > (_size / 2) - 1 &&
                i < hh - (_size / 2) && j < ww - (_size / 2))
            {
                // [3] 找到图像输入点f(i,j),以输入点为中心与核中心对齐
                //     核心为中心参考点 卷积算子=>高斯矩阵180度转向计算
                //     x y 代表卷积核的权值坐标   i j 代表图像输入点坐标
                //     卷积算子     (f*g)(i,j) = f(i-k,j-l)g(k,l)          f代表图像输入 g代表核
                //     带入核参考点 (f*g)(i,j) = f(i-(k-ai), j-(l-aj))g(k,l)   ai,aj 核参考点
                //     加权求和  注意：核的坐标以左上0,0起点
                double sum = 0.0;
                for (int k = 0; k < _size; k++) 
                {
                    for (int l = 0; l < _size; l++) 
                    {
                        sum += *(dst + (i-k+center) * ww + j-l+center) * _array[k][l];
                    }
                }
                // 放入中间结果,计算所得的值与没有计算的值不能混用
                *(_dst + (i - center) * w + j - center) = sum;
            }
        }
    }
}
