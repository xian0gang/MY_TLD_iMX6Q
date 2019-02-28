#ifndef NCC_TRACK_H_
#define NCC_TRACK_H_

void achieveTemplate(unsigned char *pSrc,unsigned char *pDst,int img_width,int img_height,int tmp_width,int tmp_height,unsigned short col,unsigned short row);
int crossCorr(unsigned char* img,unsigned char *templ,int *corr,int searchWidth,int searchHeight);
void integral(int searchWidth,int searchHeight);
void variance(float *mean,float *var);
float matchTemplate(int *newRow,int *newCol,int imgWidth,int imgHeight,int searchWidth,int searchHeight);
int TempThreshold(unsigned char *pSrc,int	srcWidth,int srcHeight,float *srcAvr,float *srcSigma);
void init_NCC();
void TrackInit(unsigned short *x,unsigned short *y);
void setNCCImagePtr(unsigned char *img);
void setNCCTemplPtr(unsigned char *img,int width,int height);
int RectThreshold(unsigned char *pSrc,int srcWidth,int srcHeight,int top,int bottom,int left,int right,float *srcAvr,float *srcSigma);
float MatchUsingNCC(int *newRow,int *newCol,int imgWidth,int imgHeight,int searchWidth,int searchHeight);
void NCCScale(const unsigned char*  pSrc1,const unsigned char*  pSrc2,unsigned char *  pDst1,unsigned char *  pDst2,int srcWidth,	int srcHeight,int tmpWidth,	int tmpHeight);
void ImageScale(unsigned char  *pSrc,unsigned char  *pDst,int srcWidth,int srcHeight);
float FullScreenNCC(int *newRow,int *newCol,int imgWidth,int imgHeight);
void updateNCCPos(int row,int col);
float getNewPos(int *row,int *col);
float avr_constantf(float *array,int length);
float TrackTarget(unsigned char *pSrc,unsigned char *pTmp,unsigned char *pBuf,	int img_width,int img_height,int tmp_width,int tmp_height,	unsigned short *x,unsigned short *y);
void TrackLevel(int level);
void GetSubImage(unsigned char *pSrc,unsigned char *pDst,int imgWidth,int imgHeight,	unsigned short subimgWidth,unsigned short subimgHeight,unsigned short col,unsigned short row);
//void RunLengthDetect();
//void AutoTarDetect(unsigned short StartRow,unsigned short EndRow,unsigned short StartCol,unsigned short EndCol);
//void Maxarea_target();
void Init_Addr(void);
void UnInit_Addr(void);


#endif
