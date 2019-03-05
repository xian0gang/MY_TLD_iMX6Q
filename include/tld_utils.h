#include <opencv2/opencv.hpp>
#pragma once

using namespace cv;
using namespace std;
//金字塔 缩放倍数
#define MULTIPLE 4
//模板大小
#define PATCH_SIZE 15

struct ScaleBox
{
    int width;
    int height;
};

struct RectBox
{
    int x;
    int y;
    int width;
    int height;
};

struct BoundingBox
{
    int x;
    int y;
    int width;
    int height;
    float overlap;        //Overlap with current Bounding Box
    int sidx;             //scale index
};

struct MyMat
{
    unsigned char data[PATCH_SIZE*PATCH_SIZE];
};




void drawBox(cv::Mat& image, CvRect box, cv::Scalar color = cvScalarAll(255), int thick=1); 

void drawPoints(cv::Mat& image, std::vector<cv::Point2f> points,cv::Scalar color=cv::Scalar::all(255));

cv::Mat createMask(const cv::Mat& image, CvRect box);

float median(std::vector<float> v);

std::vector<int> index_shuffle(int begin,int end);
double myTemplateMatch(const MyMat * pTemplate,const MyMat * src, int w, int h);
int MyIntegral(const unsigned char * src, int width, int height, int * dest, int * sqdest);
double StDev(unsigned char* src, int w, int h, int mean);
int meanDev(unsigned char* src, int w, int h);
void my_resize(const unsigned char *dataSrc, unsigned char *dataDst, int src_width, int src_height, int width, int height);
void imgRoi(const unsigned char *src, ScaleBox srcbox, unsigned char *dst, RectBox dstbox);
double **getGaussianArray(int arr_size, double sigma);
void myGaussian(const unsigned char *_src, unsigned char *_dst, int w, int h, int _size, double sigma);
