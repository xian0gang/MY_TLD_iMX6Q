#include <opencv2/opencv.hpp>
#include <tld_utils.h>
#include <iostream>
#include <sstream>
#include <TLD.h>
#include <stdio.h>
using namespace cv;
using namespace std;
//Global variables
Rect box;
bool drawing_box = false;
bool gotBB = false;
bool tl = true;
bool rep = false;
bool fromfile=false;
string video;


//读取记录bounding box的文件，获得bounding box的四个参数：左上角坐标x，y和宽高
/*如在\datasets\06_car\init.txt中：记录了初始目标的bounding box，内容如下
142,125,232,164
*/
void readBB(char* file)
{
  ifstream bb_file (file);
  string line;
  getline(bb_file,line);
  istringstream linestream(line);
  string x1,y1,x2,y2;
  getline (linestream,x1, ',');
  getline (linestream,y1, ',');
  getline (linestream,x2, ',');
  getline (linestream,y2, ',');
  int x = atoi(x1.c_str());// = (int)file["bb_x"];
  int y = atoi(y1.c_str());// = (int)file["bb_y"];
  int w = atoi(x2.c_str())-x;// = (int)file["bb_w"];
  int h = atoi(y2.c_str())-y;// = (int)file["bb_h"];
  box = Rect(x,y,w,h);
}
//bounding box mouse callback
//鼠标的响应就是得到目标区域的范围，用鼠标选中bounding box。
void mouseHandler(int event, int x, int y, int flags, void *param)
{
	switch( event )
	{
		case CV_EVENT_MOUSEMOVE:
		if (drawing_box)
		{
			box.width = x-box.x;
			box.height = y-box.y;
		}
		break;
		case CV_EVENT_LBUTTONDOWN:
		drawing_box = true;
		box = Rect( x, y, 0, 0 );
		break;
		case CV_EVENT_LBUTTONUP:
		drawing_box = false;
		if( box.width < 0 )
		{
			box.x += box.width;
			box.width *= -1;
		}
		if( box.height < 0 )
		{
			box.y += box.height;
			box.height *= -1;
		}
        gotBB = true; //已经获得bounding box
		break;
	}
}

void print_help(char** argv)
{
  printf("use:\n     %s -p /path/parameters.yml\n",argv[0]);
  printf("-s    source video\n-b        bounding box file\n-tl  track and learn\n-r     repeat\n");
}

//åˆ†æè¿è¡Œç¨‹åºæ—¶ç„å‘½ä»¤è¡Œå‚æ•°
void read_options(int argc, char** argv,VideoCapture& capture,FileStorage &fs)
{
	for (int i=0;i<argc;i++)
	{
		if (strcmp(argv[i],"-b")==0)
		{
			if (argc>i)
			{
                readBB(argv[i+1]); //是否指定初始的bounding box
				gotBB = true;
			}
			else
			print_help(argv);
		}
        if (strcmp(argv[i],"-s")==0) //从视频文件中读取
		{
			if (argc>i)
			{
				video = string(argv[i+1]);
				capture.open(video);
				fromfile = true;
			}
			else
				print_help(argv);

		}
        if (strcmp(argv[i],"-p")==0) //读取参数文件parameters.yml
		{
			if (argc>i)
			{
				fs.open(argv[i+1], FileStorage::READ);
			}
			else
				print_help(argv);
		}
		if (strcmp(argv[i],"-no_tl")==0)
		{
			tl = false;
		}
		if (strcmp(argv[i],"-r")==0)
		{
			rep = true;
		}
	}
}
/*
运行程序时：
%To run from camera
./run_tld -p ../parameters.yml
%To run from file
./run_tld -p ../parameters.yml -s ../datasets/06_car/car.mpg
%To init bounding box from file
./run_tld -p ../parameters.yml -s ../datasets/06_car/car.mpg -b ../datasets/06_car/init.txt
%To train only in the first frame (no tracking, no learning)
./run_tld -p ../parameters.yml -s ../datasets/06_car/car.mpg -b ../datasets/06_car/init.txt -no_tl
%To test the final detector (Repeat the video, first time learns, second time detects)
./run_tld -p ../parameters.yml -s ../datasets/06_car/car.mpg -b ../datasets/06_car/init.txt -r
*/
//感觉就是对起始帧进行初始化工作，然后逐帧读入图片序列，进行算法处理。
/*
int main(int argc, char * argv[])
{
	VideoCapture capture;
	capture.open(0);

	ppp();
	// detect_d();

	//OpenCV的C++接口中，用于保存图像的imwrite只能保存整数数据，且需作为图像格式。当需要保存浮
	//点数据或XML/YML文件时，OpenCV的C语言接口提供了cvSave函数，但这一函数在C++接口中已经被删除。
	//取而代之的是FileStorage类。
	FileStorage fs;

	//Read options
	//分析命令行参数
	read_options(argc,argv,capture,fs);

	//Init camera
	if (!capture.isOpened())
	{
		cout << "capture device failed to open!" << endl;
		return 1;
	}

	//Register mouse callback to draw the bounding box
	cvNamedWindow("TLD",CV_WINDOW_AUTOSIZE);

	//用鼠标选中初始目标的bounding box
	cvSetMouseCallback( "TLD", mouseHandler, NULL );
	//TLD framework
	// TLD tld;
	//Read parameters file
	// tld.read(fs.getFirstTopLevelNode());
	read(fs.getFirstTopLevelNode());
	Mat frame;
	Mat last_gray;
	Mat first;
	printf("fromfile");
	if (fromfile)//如果指定为从文件读取
	{printf("fromfile1");
		capture >> frame;		//读当前帧						//è¯»å½“å‰å¸§
		cvtColor(frame, last_gray, CV_RGB2GRAY);//转换为灰度图像		//è½¬æ¢ä¸ºç°åº¦å›¾åƒ
		frame.copyTo(first);//转换为灰度图像							//æ‹·è´ä½œä¸ºç¬¬ä¸€å¸§
	}
	else//如果为读取摄像头，则设置获取的图像大小为320x240
	{printf("fromfile2");
		capture.set(CV_CAP_PROP_FRAME_WIDTH,320);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	}

	///Initialization
	GETBOUNDINGBOX://标号：获取bounding box
	while(!gotBB)
	{
		if (!fromfile)
		{
			capture >> frame;
		}
		else
			first.copyTo(frame);
		cvtColor(frame, last_gray, CV_RGB2GRAY);
	    	drawBox(frame,box); //把bounding box 画出来
		imshow("TLD", frame);
		if (cvWaitKey(33) == 'q')
			return 0;
	}

	//由于图像片（min_win 为15x15像素）是在bounding box中采样得到的，所以box必须比min_win要大
	if (min(box.width,box.height) < (int)fs.getFirstTopLevelNode()["min_win"])
	{
		cout << "Bounding box too small, try again." << endl;
		gotBB = false;
		goto GETBOUNDINGBOX;
	}

	//Remove callback
	//如果已经获得第一帧用户框定的box了，就取消鼠标响应
	cvSetMouseCallback( "TLD", NULL, NULL );
	printf("Initial Bounding Box = x:%d y:%d w:%d h:%d\n",box.x,box.y,box.width,box.height);

	//Output file
	FILE  *bb_file = fopen("bounding_boxes.txt","w");

	//TLD initialization
	// tld.init(last_gray,box,bb_file);

	Mat last_gray_resize;
	imwrite("last_gray.bmp",last_gray);
	resize(last_gray, last_gray_resize, Size(last_gray.cols/2, last_gray.rows/2));
	imwrite("last_gray_resize.bmp",last_gray_resize);

	box.x = box.x / 2;
	box.y = box.y / 2;
	box.width = box.width / 2;
	box.height = box.height / 2;
	printf("box:%d %d %d %d \n", box.x, box.y, box.width, box.height);
		imwrite("box.bmp",last_gray_resize(box));
	init(last_gray_resize, box, bb_file);
	// init(last_gray,box,bb_file);

	///Run-time
	Mat current_gray;
	BoundingBox pbox;
	vector<Point2f> pts1;
	vector<Point2f> pts2;
	bool status=true;             	//记录跟踪成功与否的状态 lastbox been found
	int frames = 1;					//记录已过去帧数
	int detections = 1;				//记录成功检测到的目标box数目

	REPEAT:
	while(capture.read(frame))
	{
		//get frame
		cvtColor(frame, current_gray, CV_RGB2GRAY);
		
		double t = (double)getTickCount();
		//Process Frame
		processFrame(last_gray,current_gray,pts1,pts2,pbox,status,tl,bb_file);
	    	t=(double)getTickCount()-t;
    		printf("xiangang->all Run-time: %gms\n", t*1000/getTickFrequency());

		//Draw Points
	   	if (status)  //如果跟踪成功
		{
			drawPoints(frame,pts1);
	        	drawPoints(frame,pts2,Scalar(0,255,0)); //当前的特征点用蓝色点表示
			drawBox(frame,pbox);
			detections++;
		}
		//Display
		imshow("TLD", frame);
		//swap points and images
		//STL函数swap()用来交换两对象的值。其泛型化版本定义于<algorithm>;
	    	swap(last_gray,current_gray);
		pts1.clear();
		pts2.clear();
		frames++;
		printf("Detection rate: %d/%d\n",detections,frames);
		if (cvWaitKey(33) == 'q')
			break;
	}
	if (rep)
	{
		rep = false;
		tl = false;
		fclose(bb_file);
		bb_file = fopen("final_detector.txt","w");
		//capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
		capture.release();
		capture.open(video);
		goto REPEAT;
	}
	fclose(bb_file);
	return 0;
}
*/
