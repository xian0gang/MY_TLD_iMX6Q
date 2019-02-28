#include <opencv2/opencv.hpp>
#pragma once

using namespace cv;
using namespace std;

#define MULTIPLE 4

void drawBox(cv::Mat& image, CvRect box, cv::Scalar color = cvScalarAll(255), int thick=1); 

void drawPoints(cv::Mat& image, std::vector<cv::Point2f> points,cv::Scalar color=cv::Scalar::all(255));

cv::Mat createMask(const cv::Mat& image, CvRect box);

float median(std::vector<float> v);

std::vector<int> index_shuffle(int begin,int end);
void separateGaussianFilter(const Mat &src, Mat &dstt, int ksize, double sigma);
double myTemplateMatch(const Mat * pTemplate,const Mat * src);
int MyIntegral(unsigned char * src, int width, int height, int * dest, int * sqdest);
double StDev(unsigned char* src, int w, int h, int mean);
int meanDev(unsigned char* src, int w, int h);
void my_resize(const unsigned char *dataSrc, unsigned char *dataDst, int src_width, int src_height, int width, int height);