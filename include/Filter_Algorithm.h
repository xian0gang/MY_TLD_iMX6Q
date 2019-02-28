#ifndef Filter_Algorithm_h
#define Filter_Algorithm_h


//腐蚀运算
void Erosion_Filter(unsigned char* data, int width, int height);

//膨胀运算
void Dilation_Filter(unsigned char* data, int width, int height);

//3*3均值滤波
void Mean_Filter(unsigned char * pSrc, unsigned char * pDst, int search_width, int search_height);

#endif


