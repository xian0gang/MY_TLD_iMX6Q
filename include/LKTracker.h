#include<tld_utils.h>
#include "stdio.h"
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;

//使用金字塔LK光流法跟踪，
//所以类的成员变量很多都是OpenCV中calcOpticalFlowPyrLK()函数的参数
class LKTracker{
private:
  std::vector<cv::Point2f> pointsFB;
  cv::Size window_size;			//每个金字塔层的搜索窗口尺寸
  int level;					//最大的金字塔层数
  std::vector<uchar> status;	//数组。如果对应特征的光流被发现，数组中的每一个元素都被设置为 1， 否则设置为 0
  std::vector<uchar> FB_status;
  std::vector<float> similarity; //相似度
  std::vector<float> FB_error;
  float simmed;
  float fbmed;
  cv::TermCriteria term_criteria;
  float lambda;
  void normCrossCorrelation(const cv::Mat& img1,const cv::Mat& img2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);
  bool filterPts(std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2);
public:
  LKTracker();
  bool trackf2f(const cv::Mat& img1, const cv::Mat& img2,
                std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2);
  float getFB(){return fbmed;}
};

