#ifndef    TLD_H
#define    TLD_H

#include <opencv2/opencv.hpp>
#include <tld_utils.h>
#include <LKTracker.h>
#include <FerNNClassifier.h>
#include <fstream>
#include <patchgenerator.h>
#include <pthread.h>
#include <condition_variable>
#include <mutex>

//Bounding Boxes
//struct BoundingBox : public cv::Rect
//{
//    BoundingBox(){}
//    BoundingBox(cv::Rect r): cv::Rect(r){}
//public:
//    float overlap;        //Overlap with current Bounding Box
//    int sidx;             //scale index
//};

//Detection structure
struct DetStruct 
{
    std::vector<int> bb;
    std::vector<std::vector<int> > patt;
    std::vector<float> conf1;
    std::vector<float> conf2;
    std::vector<std::vector<int> > isin;
    std::vector<cv::Mat> patch;
  };
//Temporal structure
struct TempStruct 
{
	std::vector<std::vector<int> > patt;
	std::vector<float> conf;
};

struct OComparator
{
    OComparator(const std::vector<BoundingBox>& _grid):grid(_grid){}
    std::vector<BoundingBox> grid;
    bool operator()(int idx1,int idx2)
    {
        return grid[idx1].overlap > grid[idx2].overlap;
    }
};
	
struct CComparator
{
	CComparator(const std::vector<float>& _conf):conf(_conf){}
	std::vector<float> conf;
	bool operator()(int idx1,int idx2)
	{
	    return conf[idx1]> conf[idx2];
	}
};


void read(const cv::FileNode& file);
//Methods
//void init(const cv::Mat& frame1,const cv::Rect &box, FILE* bb_file);
void init(const unsigned char* frame1, ScaleBox srcbox, const Rect& box);
//void generatePositiveData(const cv::Mat& frame, int num_warps);
void generatePositiveData(const unsigned char* frame, ScaleBox box, int num_warps);
//void generateNegativeData(const cv::Mat& frame);
void generateNegativeData(const unsigned char* frame, ScaleBox box);
void processFrame(const cv::Mat& img1,const cv::Mat& img2,std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2,
  BoundingBox& bbnext,bool& lastboxfound, bool tl,FILE* bb_file);
void track(const cv::Mat& img1, const cv::Mat& img2,std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2);
//void detect(const cv::Mat& frame);
void detect(const unsigned char* frame, ScaleBox box);
void clusterConf(const std::vector<BoundingBox>& dbb,const std::vector<float>& dconf,std::vector<BoundingBox>& cbb,std::vector<float>& cconf);
void evaluate();
void learn(const cv::Mat& img);
//Tools
//void buildGrid(const cv::Mat& img, const cv::Rect& box);
void buildGrid(ScaleBox sbox, BoundingBox box);
float bbOverlap(const BoundingBox& box1,const BoundingBox& box2);
void getOverlappingBoxes(int num_closest);
void getBBHull();
//void getPattern(const cv::Mat& img, cv::Mat& pattern,cv::Scalar& mean,cv::Scalar& stdev);
void getPattern(const unsigned char* img, ScaleBox sbox, unsigned char* pattern);
void bbPoints(std::vector<cv::Point2f>& points, const BoundingBox& bb);
void bbPredict(const std::vector<cv::Point2f>& points1,const std::vector<cv::Point2f>& points2,
  const BoundingBox& bb1,BoundingBox& bb2);
int getVar(const BoundingBox& box,const cv::Mat& sum,const cv::Mat& sqsum);
bool bbComp(const BoundingBox& bb1,const BoundingBox& bb2);
int clusterBB(const std::vector<BoundingBox>& dbb,std::vector<int>& indexes);
int ppp();
int detect_d2();
int detect_d3();
int detect_d4();
int MyGetVar(const BoundingBox& box,const int* sum,const int* sqsum);

//*******************************************
int TLD_Pthread_destory(void);

int TLD_Pthread_create(void);


#endif
