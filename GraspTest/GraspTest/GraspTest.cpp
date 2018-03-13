// GraspTest.cpp : 定义控制台应用程序的入口点。
//


#include "NN.h"
#include "RobotDriver.h"
#include "KinectDriver.h"
#include "utils.h"
#include "Common.h"
#include <process.h>
#include <Windows.h>



//全局变量
Graphics			*Graph = NULL;
KinectDriver		*Kinect = NULL;
RobotDriver			*UR5 = NULL;
NN					*NNet = NULL;
Pose3D				*pos = NULL;
bool				Success = false;
HANDLE				HeventUR5 = NULL;
HANDLE				HeventKinect = NULL;
HANDLE				HeventNN = NULL;




//初始化，包括对Kinect、UR5、NN的初始化内容
GT_RES	InitGT(std::string Caffe_Path)
{
	GT_RES	res_val;
	pos = new Pose3D;
	//分配event
	HeventUR5 = CreateEvent(NULL, FALSE, FALSE, NULL);
	HeventKinect = CreateEvent(NULL, FALSE, FALSE, NULL);
	HeventNN = CreateEvent(NULL, FALSE, FALSE, NULL);
	//初始化并启动Kinect
	Graph = new Graphics;
	Graph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
	Graph->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);
	Graph->DepthInColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, DEPTHFORMAT);
	Kinect = new KinectDriver;
	res_val = Kinect->OpenKinect();
	if (res_val != GT_RES_OK)
	{
		printf("Kinect Init failed with error:%02x\n",res_val);
		return res_val;
	}
	//Load and save image from Kinect
	res_val = Kinect->GetKinectImage(Graph);
	if (res_val != GT_RES_OK)
	{
		printf("Getting Kinect image failed with error:%02x\n",res_val);
		return res_val;
	}

	//初始化并启动UR5机器人
	UR5 = new RobotDriver;
	res_val = UR5->OpenUR();
	if (res_val != GT_RES_OK)
	{
		printf("UR5 Init failed with error:%02x\n", res_val);
		return res_val;
	}
	printf("UR5 init OK\n");
	//初始化神经网络model
	NNet = new NN(MODELFILE,TRAINEDFILE,MEANFILE);
	if (NNet != NULL)
	{
		printf("Nueral Network Init failed with error:%02x.\n", res_val);
		return GT_RES_ERROR;
	}
	return GT_RES_OK;
}

//去初始化
GT_RES	UinitGT()
{
	GT_RES res;
	//close event
	CloseHandle(HeventUR5);
	CloseHandle(HeventKinect);
	CloseHandle(HeventNN);
	//uninitialize Kinect sensor
	if (Kinect)
	{
		res = Kinect->CloseKinect();
		if (res != GT_RES_OK)
		{
			printf("Kinect Uninit failed with error:%02x\n", res);
			return res;
		}
	}
	else
	{
		printf("Kinect have not initialized.\n");
		return GT_RES_DEVICENOTREADY;
	}
	//uninitialize UR5 Robot
	if (UR5)
	{
		res = UR5->CloseUR();
		if (res != GT_RES_OK)
		{
			printf("UR5 Uninit failed with error:%02x\n", res);
			return res;
		}
	}
	else
	{
		printf("UR5 have not initialized.\n");
		return GT_RES_DEVICENOTREADY;
	}
	//uninitialize NN
	delete Graph->DepthImg;
	delete Graph->ColorImg;
	delete Graph->DepthInColorImg;
	delete Kinect;
	delete UR5;
	delete Graph;
	delete pos;
	delete NNet;
	Kinect = NULL;
	Graph = NULL;
	UR5 = NULL;
	pos = NULL;
	NNet = NULL;
	return GT_RES_OK;
}

//控制机器人UR5抓取的线程
unsigned	__stdcall ThreadUR5(void *param)
{
	GT_RES res;
	DWORD eventstat;
	if (!UR5)
	{
		printf("UR5 have not initialized.\n");
		return -1;
	}
	//check if all events are initialized
	if (!(HeventUR5 && HeventKinect && HeventNN))
	{
		printf("HeventUR5 have not initialized.\n");
		return -1;
	}
	while (1)
	{
		//wait for NN event trigger
		eventstat = WaitForSingleObject(HeventNN,TIME_TOWAIT);
		if (eventstat == WAIT_TIMEOUT)
		{
			printf("NN get pose have no response.\n");
			return -1;
		}
		//UR5 move and grasp
		res = UR5->MoveGrasp(pos,&Success);
		if (res != GT_RES_OK)
		{
			printf("UR5 MoveGrasp failed with error:%02x\n", res);
			return -1;
		}
		//set UR5 event
		SetEvent(HeventUR5);
	}
	return 0;
}

//控制Kinect的线程
unsigned	__stdcall	ThreadKinect(void *param)
{
	GT_RES res;
	DWORD eventstat;												//event status for waiting event trigger
	if (!Kinect)
	{
		printf("Kinect have not initialized.\n");
		return 0;
	}
	//check if all events are initialized
	if (!(HeventUR5 && HeventKinect && HeventNN))
	{
		printf("HeventUR5 have not initialized.\n");
		return 0;
	}
	while (1)
	{
		//wait for UR5 event trigger
		eventstat = WaitForSingleObject(HeventUR5, TIME_TOWAIT);
		if (eventstat == WAIT_TIMEOUT)
		{
			printf("UR5 robot have no response.\n");
			return -1;
		}
		//get gray and depth data from kinect
		res = Kinect->GetKinectImage(Graph); 
		if (res != GT_RES_OK)
		{
			printf("Get Kinect Image failed with error:%02x\n", res);
			return -1;
		}
		//set kinect event
		SetEvent(HeventKinect);
	}
	return 0;
}

//控制NN的线程
unsigned __stdcall ThreadNN(void *param)
{
	GT_RES res;
	DWORD eventstat;
	int count = 0;
	//check if all events are initialized
	if (!(HeventUR5 && HeventKinect && HeventNN))
	{
		printf("HeventUR5 have not initialized.\n");
		return 0;
	}
	while (1)
	{
		//check Kinect event trigger
		eventstat = WaitForSingleObject(HeventKinect,0);
		if (eventstat == WAIT_TIMEOUT)
		{
			//if no kinect trigger ,then runNN;
			count++;
			res = NNet->NNRun();
			if (res != GT_RES_OK)
			{
				printf("NNet run error:%02x!\n", res);
				return -1;
			}
			//if count time out, so kinect is down.
			if (count > COUNT_TIMEOUT)
			{
				printf("Kinect have no response.\n");
				return -1;
			}
			continue;
		}
		count = 0;
		//GetPose
		res = NNet->GetPose(pos);
		if (res != GT_RES_OK)
		{
			printf("Get NN Pose failed with error:%02x\n", res);
			return -1;
		}
		res = NNet->UpdateGraphics(Graph);
		if (res != GT_RES_OK)
		{
			printf("Update graphics failed with error:%02x\n", res);
			return -1;
		}
		SetEvent(HeventNN);
	}
	return 0;
}

#if 1
int main()
{
	GT_RES	res_val;
	HANDLE HandleUR5, HandleKinect, HandleNN;
	std::string Caffe_Path = "test";
	res_val = InitGT(Caffe_Path);
	if (res_val != GT_RES_OK)
	{
		printf("Grasp Test Init failed with error:%02x\n",res_val);
		exit(1);
	}
	//create three thread for robot、kinect、NN.
	HandleUR5 = (HANDLE)_beginthreadex(NULL, 0, ThreadUR5, NULL, 0, NULL);
	HandleKinect = (HANDLE)_beginthreadex(NULL, 0, ThreadKinect, NULL, 0, NULL);
	HandleNN = (HANDLE)_beginthreadex(NULL, 0, ThreadNN, NULL, 0, NULL);

	//wait for all thread stop, and make sure the three thread is over.
	WaitForSingleObject(HandleUR5, INFINITE);
	WaitForSingleObject(HandleKinect, INFINITE);
	WaitForSingleObject(HandleNN, INFINITE);

	res_val = UinitGT();

	CloseHandle(HandleUR5);
	CloseHandle(HandleKinect);
	CloseHandle(HandleNN);

    return 0;
}

#else
int main()
{
	//test initiliazation
	GT_RES	res_val;
	pos = new Pose3D;
	//分配event
	HeventUR5 = CreateEvent(NULL, FALSE, FALSE, NULL);
	HeventKinect = CreateEvent(NULL, FALSE, FALSE, NULL);
	HeventNN = CreateEvent(NULL, FALSE, FALSE, NULL);
	//初始化并启动Kinect
	Graph = new Graphics;
	Graph->Rgba = new RGBQUAD[COLORWIDTH*COLORHEIGHT];
	Graph->depth = new UINT16[DEPTHWIDTH*DEPTHHEIGHT];
	Graph->depth2 = new UINT16[COLORWIDTH*COLORHEIGHT];
	Kinect = new KinectDriver;
	res_val = Kinect->OpenKinect();
	if (res_val != GT_RES_OK)
	{
		printf("Kinect Init failed with error:%02x\n", res_val);
		return res_val;
	}
	//初始化并启动UR5机器人
	UR5 = new RobotDriver;
	res_val = UR5->OpenUR();
	if (res_val != GT_RES_OK)
	{
		printf("UR5 Init failed with error:%02x\n", res_val);
		return res_val;
	}
	//初始化神经网络model
	NNet = new NN(MODELFILE, TRAINEDFILE, MEANFILE);
	if (NNet != NULL)
	{
		printf("Nueral Network Init failed.\n", res_val);
		return GT_RES_ERROR;
	}
	return GT_RES_OK;
}
#endif