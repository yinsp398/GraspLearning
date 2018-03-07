#pragma once
#include "Common.h"
#include <Kinect.h>

struct Graphics;


class KinectDriver
{
private:
	IKinectSensor *			m_pKinectSensor = NULL;
	IColorFrameReader *		m_pColorFrameReader = NULL;
	IDepthFrameReader *		m_pDepthFrameReader = NULL;
	ICoordinateMapper *		m_pCoordinateMapper = NULL;
	DepthSpacePoint	*		m_pDepthInColorFrame = NULL;
public:

	KinectDriver();
	~KinectDriver();

	//启动KinectSensor
	GT_RES	OpenKinect();

	//从Kinect获取图像，RGBD四维数据
	GT_RES	GetKinectImage(Graphics *Graph);

	//关闭KinectSensor
	GT_RES	CloseKinect();

private:
	//初始化KinectSensor
	GT_RES	InitKinect();

	//释放Kinect
	GT_RES	UInitKinect();
#if 0
	//彩色图像转化为灰度图
	GT_RES	RGBA2G(int Width, int Heigh, Graphics *Graph, RGBQUAD *pBuffer);
#endif
	// Safe release for interfaces
	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};



