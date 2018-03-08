#pragma once
#include "Common.h"
#include <Kinect.h>
#include <opencv2/core/core.hpp>
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

	//获取彩色图像
	GT_RES	GetColorImage(RGBQUAD *ColorImg);

	//获取深度图像
	GT_RES	GetDepthImage(UINT16 *DepthImg, UINT16 *DepthInColorImg);

	//转换深度图到Mat格式
	GT_RES	DepthConvertMat(const UINT16* pBuffer, cv::Mat *pImg);

	//转换彩色图到Mat格式
	GT_RES	RGBConvertMat(const BYTE* pBuffer, cv::Mat *pImg);

	//转换彩色图到灰度图
	GT_RES	RGBAConvertG(BYTE *pGray, const RGBQUAD *pBuffer, const unsigned int Width, const unsigned int Height);

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



