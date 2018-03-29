#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\imgcodecs.hpp>

typedef struct _CameraSpacePoint CameraSpacePoint;
/*********************************************************************/
//一些基础的宏定义

#define		GT_RES							int							//函数的返回值类型，标识了是否正确以及错误的类型；

#define		GT_RES_OK						1							//函数可能的返回值，表示运行正常；
#define		GT_RES_NODEVICE					2							//函数可能的返回值，表示没有发现设备；
#define		GT_RES_DEVICEERROR				3							//函数可能的返回值，表示设备没有正常运行；
#define		GT_RES_DEVICENOTREADY			4
#define		GT_RES_ERROR					5

#define		COLORWIDTH						1920						//彩色图像宽度
#define		COLORHEIGHT						1080						//彩色图像高度
#define		DEPTHWIDTH						512							//深度图像宽度
#define		DEPTHHEIGHT						424							//深度图像高度

#define		IPADDR							"192.168.1.4"				//UR5 IP地址
#define		PORT1							"30000"						//端口port1
#define		PORT2							"30001"						//端口port2

#define		PI								3.14159265359
#define		POSE1							0.1,-0.1,0.5,PI,0,PI		//关键路径节点1
#define		POSE2							0.2,-0.5,0.5,PI,0,PI		//关键路径节点2
#define		TCPPOSE							0.0,0.0,0.16,0.0,0.0,0.0	//TCP位置
#define		PAYLOAD							1.0,0.0,0.0,0.05			//负载大小及重心位置

#define		COUNT_TIMEOUT					100							//设置NN的超时次数
#define		TIME_TOWAIT						15000						//设置UR5和kinect的超时时间15000

#define		MODELFILE						"test1"						//caffeNet的模型文件
#define		TRAINEDFILE						"test13"					//训练好的参数文件
#define		MEANFILE						"test2"						//平均值文件

#define		COLORSPACEUP					470							//彩色空间（二维）约束的上边界
#define		COLORSPACEDOWN					820							//下边界
#define		COLORSPACELEFT					790							//左边界
#define		COLORSPACERIGHT					1140						//右边界

#define		COLORGRAPHWIDTH					100							//局部彩色图像宽度
#define		COLORGRAPHHEIGHT				100							//局部彩色图像高度
#define		DEPTHGRAPHWIDTH					50							//局部深度图像宽度
#define		DEPTHGRAPHHEIGHT				50							//局部深度图像高度


#define		BATCHSIZE						30							//一组数据个数
#define		COLORSCALE						1							//彩色图像的放大倍数

#define		IMAGEPATH						"../dataset/picture/Image"	//用于存储图像数据的路径
#define		DEPTHPATH						"../dataset/picture/Depth"	//用于存储深度数据的路径
#define		PREDICTPATH						"../dataset/Predict.txt"	//预测每次抓取的成功率的文件路径
#define		RESULTPATH						"../dataset/Result.txt"		//每次抓取结果的文件路径

#define		COLOR2ROBOT11					0.32133						//像素坐标系到机器人坐标系的转换矩阵元素
#define		COLOR2ROBOT12					-0.93730	
#define		COLOR2ROBOT13					-0.10239
#define		COLOR2ROBOT14					0.13511
#define		COLOR2ROBOT21					-0.94147	
#define		COLOR2ROBOT22					-0.30964	
#define		COLOR2ROBOT23					-0.11571	
#define		COLOR2ROBOT24					-0.45551	
#define		COLOR2ROBOT31					0.06166	
#define		COLOR2ROBOT32					0.16098	
#define		COLOR2ROBOT33					-0.93716	
#define		COLOR2ROBOT34					0.90480	
#define		ANGLEBIAS						0							//抓取角度从图像到机器人坐标系的偏置

#define		COLORFORMAT						CV_8UC1						//图像在cv mat中的存储格式，8位，一个通道的灰度图
#define		DEPTHFORMAT						CV_16UC1					//深度图像在cv::mat中的存储格式，16位，一个通道的灰度图

struct GraspPose
{
	unsigned int x;
	unsigned int y;
	double theta;
};

struct Graphics
{
	cv::Mat *ColorImg;
	cv::Mat *DepthImg;
	cv::Mat *CloudPointsImg;
};



